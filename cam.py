# XRP Controller: read OpenMV RT1062 optical flow over Qwiic 1
#
# RUN THIS FILE ON THE XRP.
#
# Wiring:
#   XRP GPIO39 / Qwiic 1 SCL -> OpenMV P4
#   XRP GPIO38 / Qwiic 1 SDA -> OpenMV P5
#   XRP GND                  -> OpenMV GND
#   Regulated XRP 5 V        -> OpenMV VIN
#
# This is a TEST/LOGGER only. It does not command the flight controller.

from machine import I2C, Pin
import struct
import time


I2C_BUS = 1
SDA_PIN = 38
SCL_PIN = 39
I2C_FREQUENCY = 400000

OPENMV_ADDRESS = 0x42
PACKET_SIZE = 16
PACKET_MAGIC = 0xA5
PACKET_VERSION = 1

READ_PERIOD_MS = 25
STALE_AFTER_MS = 300


def packet_checksum(packet):
    checksum = 0
    for index in range(PACKET_SIZE - 1):
        checksum ^= packet[index]
    return checksum


def read_packet(i2c):
    packet = i2c.readfrom_mem(
        OPENMV_ADDRESS,
        0,
        PACKET_SIZE,
    )

    if len(packet) != PACKET_SIZE:
        raise ValueError(
            "Wrong packet length: {}".format(len(packet))
        )

    if packet[0] != PACKET_MAGIC:
        raise ValueError(
            "Wrong magic: 0x{:02X}".format(packet[0])
        )

    if packet[1] != PACKET_VERSION:
        raise ValueError(
            "Wrong version: {}".format(packet[1])
        )

    expected_checksum = packet_checksum(packet)

    if packet[15] != expected_checksum:
        raise ValueError("Checksum mismatch")

    (
        magic,
        version,
        sequence,
        flags,
        dx_x100,
        dy_x100,
        quality_x1000,
        fps_x10,
        dt_ms,
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
        "frame_dt_ms": dt_ms,
    }


i2c = I2C(
    I2C_BUS,
    sda=Pin(SDA_PIN),
    scl=Pin(SCL_PIN),
    freq=I2C_FREQUENCY,
)

print("Scanning XRP Qwiic 1...")
devices = i2c.scan()
print(
    "Found:",
    ["0x{:02X}".format(address) for address in devices],
)

if OPENMV_ADDRESS not in devices:
    raise RuntimeError(
        "OpenMV not found at 0x42. "
        "Check P4/P5, common ground, camera power, and OpenMV script."
    )

print("OpenMV found. Reading optical flow...")

last_sequence = None
last_new_packet_ms = time.ticks_ms()
good_packets = 0
bad_packets = 0


while True:
    loop_start_ms = time.ticks_ms()

    try:
        data = read_packet(i2c)

        if data["sequence"] != last_sequence:
            last_sequence = data["sequence"]
            last_new_packet_ms = loop_start_ms
            good_packets += 1

        age_ms = time.ticks_diff(
            loop_start_ms,
            last_new_packet_ms,
        )

        stale = age_ms > STALE_AFTER_MS

        print(
            "dx={:+.2f}  dy={:+.2f}  "
            "quality={:.3f}  fps={:.1f}  "
            "valid={}  stale={}  age={}ms".format(
                data["dx"],
                data["dy"],
                data["quality"],
                data["fps"],
                data["valid"],
                stale,
                age_ms,
            )
        )

    except Exception as error:
        bad_packets += 1
        print(
            "OpenMV read error:",
            error,
            "good=",
            good_packets,
            "bad=",
            bad_packets,
        )

    used_ms = time.ticks_diff(
        time.ticks_ms(),
        loop_start_ms,
    )
    sleep_ms = READ_PERIOD_MS - used_ms

    if sleep_ms > 0:
        time.sleep_ms(sleep_ms)
