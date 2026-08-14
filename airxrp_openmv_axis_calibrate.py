# AirXRP OpenMV optical-flow axis calibration
#
# RUN THIS ON THE XRP WITH ALL PROPELLERS REMOVED.
#
# The OpenMV must already be running the optical-flow I2C program at 0x42.
# This script learns how the camera's image axes are mounted relative to:
#   - drone right
#   - drone forward
#
# It writes:
#   /openmv_axis_calibration.txt
#
# LED:
#   Blue   = startup
#   Green  = hold still
#   Cyan   = move the whole drone slowly RIGHT
#   Yellow = move the whole drone slowly FORWARD
#   Purple = calibration saved
#   Red    = calibration failed
#
# A small 10-20 cm movement is enough. Keep the camera level and do not yaw.

from machine import I2C, Pin
import struct
import time
import math
import os

try:
    import neopixel
except ImportError:
    neopixel = None


# ============================================================
# SETTINGS
# ============================================================

I2C_BUS = 1
SDA_PIN = 38
SCL_PIN = 39
I2C_FREQUENCY = 400000
OPENMV_ADDRESS = 0x42

PACKET_SIZE = 16
PACKET_MAGIC = 0xA5
PACKET_VERSION = 1

QUALITY_MIN = 0.35
MIN_FLOW_MAGNITUDE = 0.12
READ_PERIOD_MS = 25

STILL_SECONDS = 2
COUNTDOWN_SECONDS = 4
MOTION_SECONDS = 2.5
PAUSE_SECONDS = 3

CALIBRATION_FILE = "/openmv_axis_calibration.txt"
STATUS_FILE = "/openmv_axis_cal_status.txt"
ERROR_FILE = "/openmv_axis_cal_error.txt"

RGB_LED_PIN = 37
LED_OFF = (0, 0, 0)
LED_STARTUP = (0, 0, 255)
LED_STILL = (0, 255, 0)
LED_RIGHT = (0, 255, 255)
LED_FORWARD = (255, 160, 0)
LED_DONE = (180, 0, 180)
LED_ERROR = (255, 0, 0)


# ============================================================
# HELPERS
# ============================================================

_pixel = None


def init_led():
    global _pixel

    if neopixel is None:
        return

    try:
        _pixel = neopixel.NeoPixel(
            Pin(RGB_LED_PIN, Pin.OUT),
            1,
        )
        set_led(LED_OFF)
    except Exception:
        _pixel = None


def set_led(color):
    if _pixel is None:
        return

    try:
        _pixel[0] = color
        _pixel.write()
    except Exception:
        pass


def flush_file(file_object):
    try:
        file_object.flush()
    except Exception:
        pass

    try:
        os.sync()
    except Exception:
        pass


def write_status(message):
    text = str(message)
    print(text)

    try:
        with open(STATUS_FILE, "w") as file_object:
            file_object.write(text)
            file_object.write("\n")
            flush_file(file_object)
    except Exception:
        pass


def packet_checksum(packet):
    value = 0

    for index in range(PACKET_SIZE - 1):
        value ^= packet[index]

    return value


def read_packet(i2c):
    packet = i2c.readfrom_mem(
        OPENMV_ADDRESS,
        0,
        PACKET_SIZE,
    )

    if len(packet) != PACKET_SIZE:
        raise RuntimeError("Wrong OpenMV packet length")

    if packet[0] != PACKET_MAGIC:
        raise RuntimeError("Wrong OpenMV packet magic")

    if packet[1] != PACKET_VERSION:
        raise RuntimeError("Wrong OpenMV packet version")

    if packet[15] != packet_checksum(packet):
        raise RuntimeError("OpenMV packet checksum mismatch")

    (
        _magic,
        _version,
        sequence,
        flags,
        dx_x100,
        dy_x100,
        quality_x1000,
        fps_x10,
        frame_dt_ms,
    ) = struct.unpack_from(
        "<BBBBhhHHH",
        packet,
        0,
    )

    return {
        "sequence": sequence,
        "valid": bool(flags & 0x01),
        "dx": dx_x100 / 100.0,
        "dy": dy_x100 / 100.0,
        "quality": quality_x1000 / 1000.0,
        "fps": fps_x10 / 10.0,
        "frame_dt_ms": frame_dt_ms,
    }


def magnitude(x_value, y_value):
    return math.sqrt(
        x_value * x_value
        + y_value * y_value
    )


def countdown(name, color):
    set_led(color)

    for remaining in range(
        COUNTDOWN_SECONDS,
        0,
        -1,
    ):
        write_status(
            "{}\n"
            "keep_camera_level\n"
            "begin_in={}s".format(
                name,
                remaining,
            )
        )
        time.sleep(1)


def collect_motion(i2c, label):
    samples = []
    start_ms = time.ticks_ms()
    last_sequence = None

    while time.ticks_diff(
        time.ticks_ms(),
        start_ms,
    ) < int(MOTION_SECONDS * 1000):

        loop_ms = time.ticks_ms()

        try:
            data = read_packet(i2c)

            if data["sequence"] == last_sequence:
                time.sleep_ms(READ_PERIOD_MS)
                continue

            last_sequence = data["sequence"]

            flow_magnitude = magnitude(
                data["dx"],
                data["dy"],
            )

            if (
                data["valid"]
                and data["quality"] >= QUALITY_MIN
                and flow_magnitude >= MIN_FLOW_MAGNITUDE
            ):
                samples.append((
                    data["dx"],
                    data["dy"],
                    flow_magnitude,
                    data["quality"],
                ))

        except Exception:
            pass

        elapsed_ms = time.ticks_diff(
            time.ticks_ms(),
            loop_ms,
        )

        sleep_ms = READ_PERIOD_MS - elapsed_ms

        if sleep_ms > 0:
            time.sleep_ms(sleep_ms)

    if len(samples) < 12:
        raise RuntimeError(
            "{} produced too few usable samples: {}"
            .format(label, len(samples))
        )

    # Keep strongest 60% of movement samples.
    samples.sort(
        key=lambda sample: sample[2],
        reverse=True,
    )

    keep_count = max(
        10,
        int(len(samples) * 0.60),
    )

    kept = samples[:keep_count]

    sum_x = 0.0
    sum_y = 0.0
    sum_weight = 0.0

    for dx, dy, flow_magnitude, quality in kept:
        weight = flow_magnitude * quality
        sum_x += dx * weight
        sum_y += dy * weight
        sum_weight += weight

    mean_x = sum_x / sum_weight
    mean_y = sum_y / sum_weight

    mean_magnitude = magnitude(
        mean_x,
        mean_y,
    )

    if mean_magnitude < MIN_FLOW_MAGNITUDE:
        raise RuntimeError(
            "{} motion vector was too small".format(label)
        )

    unit_x = mean_x / mean_magnitude
    unit_y = mean_y / mean_magnitude

    return {
        "mean_x": mean_x,
        "mean_y": mean_y,
        "magnitude": mean_magnitude,
        "unit_x": unit_x,
        "unit_y": unit_y,
        "samples": len(samples),
        "kept": len(kept),
    }


# ============================================================
# MAIN
# ============================================================

def main():
    init_led()
    set_led(LED_STARTUP)

    i2c = I2C(
        I2C_BUS,
        sda=Pin(SDA_PIN),
        scl=Pin(SCL_PIN),
        freq=I2C_FREQUENCY,
    )

    devices = i2c.scan()

    if OPENMV_ADDRESS not in devices:
        raise RuntimeError(
            "OpenMV not found at 0x42"
        )

    set_led(LED_STILL)

    write_status(
        "OPENMV_AXIS_CALIBRATION_READY\n"
        "REMOVE_PROPELLERS\n"
        "hold_drone_level_and_still"
    )

    time.sleep(STILL_SECONDS)

    # --------------------------------------------------------
    # RIGHT
    # --------------------------------------------------------

    countdown(
        "MOVE_DRONE_SLOWLY_RIGHT",
        LED_RIGHT,
    )

    right = collect_motion(
        i2c,
        "right",
    )

    write_status(
        "RIGHT_CAPTURED\n"
        "mean_dx={:.3f}\n"
        "mean_dy={:.3f}\n"
        "now_return_and_hold_still".format(
            right["mean_x"],
            right["mean_y"],
        )
    )

    set_led(LED_STILL)
    time.sleep(PAUSE_SECONDS)

    # --------------------------------------------------------
    # FORWARD
    # --------------------------------------------------------

    countdown(
        "MOVE_DRONE_SLOWLY_FORWARD",
        LED_FORWARD,
    )

    forward = collect_motion(
        i2c,
        "forward",
    )

    # --------------------------------------------------------
    # Solve camera -> body transformation
    # --------------------------------------------------------

    dot = (
        right["unit_x"] * forward["unit_x"]
        + right["unit_y"] * forward["unit_y"]
    )

    dot = max(-1.0, min(1.0, dot))

    angle_deg = math.degrees(
        math.acos(dot)
    )

    determinant = (
        right["unit_x"] * forward["unit_y"]
        - forward["unit_x"] * right["unit_y"]
    )

    if angle_deg < 45.0 or angle_deg > 135.0:
        raise RuntimeError(
            "Right/forward motions were not distinct enough; "
            "measured angle={:.1f}deg".format(
                angle_deg
            )
        )

    if abs(determinant) < 0.35:
        raise RuntimeError(
            "Camera-axis calibration matrix is nearly singular"
        )

    # Inverse of:
    #
    # [ right_unit_x    forward_unit_x ]
    # [ right_unit_y    forward_unit_y ]
    #
    # Converts OpenMV dx/dy into:
    # + body-right
    # + body-forward

    inverse_00 = (
        forward["unit_y"]
        / determinant
    )

    inverse_01 = (
        -forward["unit_x"]
        / determinant
    )

    inverse_10 = (
        -right["unit_y"]
        / determinant
    )

    inverse_11 = (
        right["unit_x"]
        / determinant
    )

    # --------------------------------------------------------
    # Save calibration
    # --------------------------------------------------------

    with open(CALIBRATION_FILE, "w") as file_object:

        file_object.write(
            "OPENMV_AXIS_CAL_V1\n"
        )

        file_object.write(
            "right_unit_x={}\n".format(
                right["unit_x"]
            )
        )

        file_object.write(
            "right_unit_y={}\n".format(
                right["unit_y"]
            )
        )

        file_object.write(
            "forward_unit_x={}\n".format(
                forward["unit_x"]
            )
        )

        file_object.write(
            "forward_unit_y={}\n".format(
                forward["unit_y"]
            )
        )

        file_object.write(
            "inverse_00={}\n".format(
                inverse_00
            )
        )

        file_object.write(
            "inverse_01={}\n".format(
                inverse_01
            )
        )

        file_object.write(
            "inverse_10={}\n".format(
                inverse_10
            )
        )

        file_object.write(
            "inverse_11={}\n".format(
                inverse_11
            )
        )

        file_object.write(
            "right_mean_magnitude={}\n".format(
                right["magnitude"]
            )
        )

        file_object.write(
            "forward_mean_magnitude={}\n".format(
                forward["magnitude"]
            )
        )

        file_object.write(
            "axis_angle_deg={}\n".format(
                angle_deg
            )
        )

        file_object.write(
            "determinant={}\n".format(
                determinant
            )
        )

        file_object.write(
            "valid=1\n"
        )

        flush_file(file_object)

    set_led(LED_DONE)

    write_status(
        "OPENMV_AXIS_CALIBRATION_SAVED\n"
        "right=({:.3f},{:.3f})\n"
        "forward=({:.3f},{:.3f})\n"
        "axis_angle_deg={:.1f}\n"
        "saved={}".format(
            right["unit_x"],
            right["unit_y"],
            forward["unit_x"],
            forward["unit_y"],
            angle_deg,
            CALIBRATION_FILE,
        )
    )

    while True:
        time.sleep_ms(1000)


try:
    main()

except Exception as error:

    init_led()
    set_led(LED_ERROR)

    write_status(
        "OPENMV_AXIS_CALIBRATION_ERROR\n{}"
        .format(error)
    )

    try:
        with open(
            ERROR_FILE,
            "w",
        ) as file_object:

            import sys

            sys.print_exception(
                error,
                file_object,
            )

            flush_file(file_object)

    except Exception:
        pass

    while True:
        time.sleep_ms(1000)
