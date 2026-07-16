# AirXRP OpenMV optical-flow damping SEND-ONLY controller
#
# IMPORTANT:
#   This version does NOT read MSP_RC, MSP_ATTITUDE, AUX3, or throttle.
#   It only SENDS roll/pitch override commands.
#
# Why:
#   Your MSP_SET_RAW_RC override works, but MSP_RC readback is unreliable/no-response.
#   So Betaflight itself will be the activation gate:
#     - AUX3 LOW  -> MSP OVERRIDE mode inactive -> Betaflight ignores these commands
#     - AUX3 HIGH -> MSP OVERRIDE mode active   -> Betaflight applies roll/pitch
#
# Betaflight setup:
#   set msp_override_channels_mask = 3
#   save
#
# Modes tab:
#   MSP OVERRIDE / RX MSP OVERRIDE assigned to AUX3.
#
# SAFETY:
#   REMOVE PROPS for bench test.
#   AUX3 LOW should immediately give manual roll/pitch back.
#
# Channel order:
#   roll, pitch, yaw, throttle, aux1, aux2, aux3, aux4

import sys
import time
import struct
import math
import os

try:
    import neopixel
except ImportError:
    neopixel = None

sys.path.append("/main")

from machine import I2C, Pin
from msp import MSP


# ============================================================
# SETTINGS
# ============================================================

AXIS_CALIBRATION_FILE = "/openmv_axis_calibration.txt"

I2C_BUS = 1
SDA_PIN = 38
SCL_PIN = 39
I2C_FREQUENCY = 400000
OPENMV_ADDRESS = 0x42

PACKET_SIZE = 16
PACKET_MAGIC = 0xA5
PACKET_VERSION = 1

FLOW_QUALITY_MIN = 0.30
FLOW_FPS_MIN = 30.0
FLOW_STALE_MS = 150
FLOW_MAX_ABS_PER_FRAME = 4.0

# This version uses optical-flow rate instead of raw per-frame displacement.
# At 45-50 FPS, hover drift can be tiny per frame, so dx/dy alone looks small.
FLOW_RATE_DEADBAND_PX_S = 1.5
FLOW_FILTER_ALPHA = 0.35

# Conservative first hover values.
# Units are us per pixel/second.
FLOW_KP_US_PER_PX_S = 0.35
MAX_RC_OFFSET_US = 10.0
OUTPUT_SLEW_US_PER_S = 50.0

# Manual hover trim centers.
# Change these, not the random 1500s lower in the file.
# Start with tiny changes: +/-2 to +/-5 us.
ROLL_CENTER_US = 1500
PITCH_CENTER_US = 1500
# TRIM_SAFE_NOTE: invalid flow returns to these centers, not 1500/1500.

LOOP_PERIOD_MS = 20
PRINT_PERIOD_MS = 250
NEUTRAL_FRAMES_ON_ERROR = 20

# If one or two OpenMV frames are bad, keep the last correction briefly
# instead of snapping back to trim immediately.
INVALID_HOLD_MS = 250

# XRP onboard RGB LED.
# Green blink = program running / valid flow loop alive.
# Orange blink = program running but flow currently invalid.
# Red = fatal error.
RGB_LED_PIN = 37
LED_OFF = (0, 0, 0)
LED_RUNNING = (0, 80, 0)
LED_INVALID = (80, 35, 0)
LED_ERROR = (80, 0, 0)
LED_BLINK_PERIOD_MS = 500

# Flight data logging.
# Logs to XRP flash. Keep tests short so the file doesn't get huge.
LOG_ENABLED = True
LOG_PERIOD_MS = 50
LOG_FLUSH_EVERY_N_LINES = 10
LOG_PREFIX = "/flow_flight_log_"


# ============================================================
# LED HELPERS
# ============================================================

_led = None
_led_on = False
_last_led_toggle_ms = 0


def init_led():
    global _led

    if neopixel is None:
        return

    try:
        _led = neopixel.NeoPixel(
            Pin(RGB_LED_PIN, Pin.OUT),
            1,
        )
        set_led(LED_OFF)
    except Exception:
        _led = None


def set_led(color):
    if _led is None:
        return

    try:
        _led[0] = color
        _led.write()
    except Exception:
        pass


def heartbeat_led(color, now_ms):
    global _led_on
    global _last_led_toggle_ms

    if _led is None:
        return

    if time.ticks_diff(now_ms, _last_led_toggle_ms) >= LED_BLINK_PERIOD_MS:
        _last_led_toggle_ms = now_ms
        _led_on = not _led_on

        if _led_on:
            set_led(color)
        else:
            set_led(LED_OFF)


# ============================================================
# LOGGING HELPERS
# ============================================================

def next_log_filename():
    try:
        existing = os.listdir("/")
    except Exception:
        existing = []

    index = 0

    while True:
        name = "flow_flight_log_{:03d}.csv".format(index)
        if name not in existing:
            return "/" + name
        index += 1


def open_log_file():
    if not LOG_ENABLED:
        return None, None

    filename = next_log_filename()
    file_object = open(filename, "w")

    file_object.write(
        "t_ms,valid,reason,seq,dx,dy,quality,fps,frame_dt_ms,age_ms,"
        "right_rate,forward_rate,filtered_right,filtered_forward,"
        "roll_rc,pitch_rc\n"
    )

    file_object.flush()

    print("Logging to", filename)

    return file_object, filename


def write_log_line(
    file_object,
    t_ms,
    valid,
    reason,
    flow,
    result,
):
    if file_object is None:
        return

    if flow is None:
        sequence = -1
        dx = 0.0
        dy = 0.0
        quality = 0.0
        fps = 0.0
        frame_dt_ms = 0
        age_ms = 999999
    else:
        sequence = flow["sequence"]
        dx = flow["dx"]
        dy = flow["dy"]
        quality = flow["quality"]
        fps = flow["fps"]
        frame_dt_ms = flow["frame_dt_ms"]
        age_ms = flow["age_ms"]

    file_object.write(
        "{},{},{},{},{:.3f},{:.3f},{:.3f},{:.1f},{},{}"
        ",{:.3f},{:.3f},{:.3f},{:.3f},{:.1f},{:.1f}\n".format(
            t_ms,
            int(valid),
            reason,
            sequence,
            dx,
            dy,
            quality,
            fps,
            frame_dt_ms,
            age_ms,
            result["body_right"],
            result["body_forward"],
            result["filtered_right"],
            result["filtered_forward"],
            result["roll_rc"],
            result["pitch_rc"],
        )
    )


# ============================================================
# BASIC HELPERS
# ============================================================

def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def apply_deadband(value, deadband):
    if abs(value) <= deadband:
        return 0.0
    if value > 0:
        return value - deadband
    return value + deadband


def slew(current, target, maximum_change):
    if target > current + maximum_change:
        return current + maximum_change
    if target < current - maximum_change:
        return current - maximum_change
    return target


def parse_key_value_file(filename):
    values = {}

    with open(filename, "r") as file_object:
        for line in file_object:
            line = line.strip()

            if not line or "=" not in line:
                continue

            key, value = line.split("=", 1)
            values[key.strip()] = value.strip()

    return values


def packet_checksum(packet):
    value = 0
    for index in range(PACKET_SIZE - 1):
        value ^= packet[index]
    return value


# ============================================================
# OPENMV FLOW READER
# ============================================================

class OpenMVFlow:
    def __init__(self):
        self.i2c = I2C(
            I2C_BUS,
            sda=Pin(SDA_PIN),
            scl=Pin(SCL_PIN),
            freq=I2C_FREQUENCY,
        )

        devices = self.i2c.scan()

        if OPENMV_ADDRESS not in devices:
            raise RuntimeError("OpenMV not found at 0x42")

        self.last_sequence = None
        self.last_new_ms = None

    def read(self, now_ms):
        packet = self.i2c.readfrom_mem(
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

        if sequence != self.last_sequence:
            self.last_sequence = sequence
            self.last_new_ms = now_ms

        age_ms = (
            999999
            if self.last_new_ms is None
            else time.ticks_diff(now_ms, self.last_new_ms)
        )

        return {
            "sequence": sequence,
            "camera_valid": bool(flags & 0x01),
            "dx": dx_x100 / 100.0,
            "dy": dy_x100 / 100.0,
            "quality": quality_x1000 / 1000.0,
            "fps": fps_x10 / 10.0,
            "frame_dt_ms": frame_dt_ms,
            "age_ms": age_ms,
        }


# ============================================================
# MSP SEND-ONLY
# ============================================================

class BetaflightSendOnly:
    def __init__(self):
        self.msp = MSP(
            uart_id=0,
            tx_pin=0,
            rx_pin=1,
            baudrate=115200,
        )

    def send_roll_pitch(self, roll_rc, pitch_rc):
        self.msp.send_raw_rc([
            int(round(clamp(roll_rc, 1000, 2000))),
            int(round(clamp(pitch_rc, 1000, 2000))),
            1500,  # yaw neutral placeholder
            1000,  # throttle low placeholder
            1000,
            1000,
            1000,
            1000,
        ])

    def send_neutral(self):
        self.send_roll_pitch(ROLL_CENTER_US, PITCH_CENTER_US)


# ============================================================
# CONTROLLER
# ============================================================

class FlowDampingController:
    def __init__(self, calibration):
        self.inverse_00 = float(calibration["inverse_00"])
        self.inverse_01 = float(calibration["inverse_01"])
        self.inverse_10 = float(calibration["inverse_10"])
        self.inverse_11 = float(calibration["inverse_11"])

        self.filtered_right = 0.0
        self.filtered_forward = 0.0
        self.roll_offset = 0.0
        self.pitch_offset = 0.0

    def reset(self):
        self.filtered_right = 0.0
        self.filtered_forward = 0.0
        self.roll_offset = 0.0
        self.pitch_offset = 0.0

    def update(self, dx, dy, frame_dt_ms, dt_seconds):
        body_right_per_frame = (
            self.inverse_00 * dx
            + self.inverse_01 * dy
        )

        body_forward_per_frame = (
            self.inverse_10 * dx
            + self.inverse_11 * dy
        )

        # Convert from pixels/frame to pixels/second.
        # This makes the controller respond to actual speed instead of
        # looking weak simply because the camera is running at ~45-50 FPS.
        frame_dt_s = max(0.001, frame_dt_ms / 1000.0)

        body_right = body_right_per_frame / frame_dt_s
        body_forward = body_forward_per_frame / frame_dt_s

        self.filtered_right += (
            FLOW_FILTER_ALPHA
            * (body_right - self.filtered_right)
        )

        self.filtered_forward += (
            FLOW_FILTER_ALPHA
            * (body_forward - self.filtered_forward)
        )

        effective_right = apply_deadband(
            self.filtered_right,
            FLOW_RATE_DEADBAND_PX_S,
        )

        effective_forward = apply_deadband(
            self.filtered_forward,
            FLOW_RATE_DEADBAND_PX_S,
        )

        # ROLL SIGN FLIPPED:
        # Use this if the drone visually drifts left while the controller
        # is commanding roll below 1500 / left correction.
        target_roll_offset = clamp(
            FLOW_KP_US_PER_PX_S * effective_right,
            -MAX_RC_OFFSET_US,
            MAX_RC_OFFSET_US,
        )

        target_pitch_offset = clamp(
            -FLOW_KP_US_PER_PX_S * effective_forward,
            -MAX_RC_OFFSET_US,
            MAX_RC_OFFSET_US,
        )

        maximum_change = OUTPUT_SLEW_US_PER_S * dt_seconds

        self.roll_offset = slew(
            self.roll_offset,
            target_roll_offset,
            maximum_change,
        )

        self.pitch_offset = slew(
            self.pitch_offset,
            target_pitch_offset,
            maximum_change,
        )

        return {
            "body_right": body_right,
            "body_forward": body_forward,
            "filtered_right": self.filtered_right,
            "filtered_forward": self.filtered_forward,
            "roll_rc": ROLL_CENTER_US + self.roll_offset,
            "pitch_rc": PITCH_CENTER_US + self.pitch_offset,
        }


# ============================================================
# VALIDITY
# ============================================================

def flow_is_valid(flow):
    if flow is None:
        return False, "no_flow"

    if flow["age_ms"] > FLOW_STALE_MS:
        return False, "stale_flow"

    if not flow["camera_valid"]:
        return False, "camera_invalid"

    if flow["quality"] < FLOW_QUALITY_MIN:
        return False, "low_quality"

    if flow["fps"] < FLOW_FPS_MIN:
        return False, "low_fps"

    if (
        abs(flow["dx"]) > FLOW_MAX_ABS_PER_FRAME
        or abs(flow["dy"]) > FLOW_MAX_ABS_PER_FRAME
    ):
        return False, "flow_too_large"

    return True, "ok"


# ============================================================
# MAIN
# ============================================================

def main():
    init_led()
    set_led(LED_RUNNING)

    calibration = parse_key_value_file(AXIS_CALIBRATION_FILE)

    if calibration.get("valid", "0") != "1":
        raise RuntimeError("Missing valid OpenMV axis calibration")

    camera = OpenMVFlow()
    fc = BetaflightSendOnly()
    controller = FlowDampingController(calibration)

    print("OPENMV FLOW DAMPING 10US RATE-HOLD ROLL-FLIPPED READY")
    print("Betaflight AUX3 MSP OVERRIDE is the activation gate.")
    print("AUX3 low = BF ignores roll/pitch override.")
    print("AUX3 high = BF applies roll/pitch override.")
    print("REMOVE PROPS for bench test.")

    start_ms = time.ticks_ms()
    last_loop_ms = start_ms
    last_print_ms = start_ms
    last_log_ms = start_ms
    log_file, log_filename = open_log_file()
    log_lines_since_flush = 0
    last_good_ms = None

    result = {
        "body_right": 0.0,
        "body_forward": 0.0,
        "filtered_right": 0.0,
        "filtered_forward": 0.0,
        "roll_rc": ROLL_CENTER_US,
        "pitch_rc": PITCH_CENTER_US,
    }

    while True:
        loop_start_ms = time.ticks_ms()

        dt_seconds = max(
            0.001,
            time.ticks_diff(loop_start_ms, last_loop_ms) / 1000.0,
        )

        last_loop_ms = loop_start_ms

        flow = None

        try:
            flow = camera.read(loop_start_ms)
            valid, reason = flow_is_valid(flow)
        except Exception as error:
            valid = False
            reason = "openmv_read_error"

        if valid:
            result = controller.update(
                flow["dx"],
                flow["dy"],
                flow["frame_dt_ms"],
                dt_seconds,
            )

            fc.send_roll_pitch(
                result["roll_rc"],
                result["pitch_rc"],
            )

            last_good_ms = loop_start_ms

        else:
            # A single bad optical-flow frame should not instantly remove
            # the correction. Hold the last command briefly, then go neutral
            # if the camera stays bad.
            holding_last = (
                last_good_ms is not None
                and time.ticks_diff(loop_start_ms, last_good_ms) <= INVALID_HOLD_MS
            )

            if holding_last:
                reason = reason + "_hold"
                fc.send_roll_pitch(
                    result["roll_rc"],
                    result["pitch_rc"],
                )

            else:
                controller.reset()

                result = {
                    "body_right": 0.0,
                    "body_forward": 0.0,
                    "filtered_right": 0.0,
                    "filtered_forward": 0.0,
                    "roll_rc": ROLL_CENTER_US,
                    "pitch_rc": PITCH_CENTER_US,
                }

                fc.send_neutral()

        if valid:
            heartbeat_led(LED_RUNNING, loop_start_ms)
        else:
            heartbeat_led(LED_INVALID, loop_start_ms)

        if time.ticks_diff(loop_start_ms, last_log_ms) >= LOG_PERIOD_MS:
            write_log_line(
                log_file,
                time.ticks_diff(loop_start_ms, start_ms),
                valid,
                reason,
                flow,
                result,
            )

            log_lines_since_flush += 1

            if (
                log_file is not None
                and log_lines_since_flush >= LOG_FLUSH_EVERY_N_LINES
            ):
                log_file.flush()
                log_lines_since_flush = 0

            last_log_ms = loop_start_ms

        if time.ticks_diff(loop_start_ms, last_print_ms) >= PRINT_PERIOD_MS:
            print(
                "valid={} reason={} "
                "dx={:+.2f} dy={:+.2f} q={:.2f} "
                "right_rate={:+.1f} forward_rate={:+.1f} "
                "roll={} pitch={}".format(
                    valid,
                    reason,
                    flow["dx"] if flow else 0.0,
                    flow["dy"] if flow else 0.0,
                    flow["quality"] if flow else 0.0,
                    result["body_right"],
                    result["body_forward"],
                    int(round(result["roll_rc"])),
                    int(round(result["pitch_rc"])),
                )
            )

            last_print_ms = loop_start_ms

        elapsed_ms = time.ticks_diff(
            time.ticks_ms(),
            loop_start_ms,
        )

        sleep_ms = LOOP_PERIOD_MS - elapsed_ms

        if sleep_ms > 0:
            time.sleep_ms(sleep_ms)


try:
    main()

except KeyboardInterrupt:
    init_led()
    set_led(LED_OFF)
    fc = BetaflightSendOnly()
    for _ in range(NEUTRAL_FRAMES_ON_ERROR):
        fc.send_neutral()
        time.sleep_ms(20)
    print("Stopped and sent neutral.")

except Exception as error:
    init_led()
    set_led(LED_ERROR)
    print("OPENMV FLOW DAMPING SEND-ONLY ERROR:", error)

    try:
        fc = BetaflightSendOnly()
        for _ in range(NEUTRAL_FRAMES_ON_ERROR):
            fc.send_neutral()
            time.sleep_ms(20)
    except Exception:
        pass

    raise
