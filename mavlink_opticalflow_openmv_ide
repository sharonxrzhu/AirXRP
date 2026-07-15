# OpenMV Cam RT1062 optical flow -> XRP over I2C
#
# Run this on the OpenMV Cam RT1062.
# P4 = I2C1 SCL, P5 = I2C1 SDA.
# The XRP reads a 16-byte packet from I2C address 0x42.
#
# This replaces the original MAVLink UART3 output.

import csi
import image
import machine
import struct
import time

I2C_ADDRESS = 0x42
FRAME_SIZE = (64, 64)
QUALITY_THRESHOLD = 0.30
PRINT_PERIOD_MS = 500

PACKET_SIZE = 16
MAGIC = 0xA5
VERSION = 1


def value_of(obj, name):
    value = getattr(obj, name)
    return value() if callable(value) else value


def clamp(value, low, high):
    return max(low, min(high, value))


def checksum(packet):
    result = 0
    for byte in packet[:-1]:
        result ^= byte
    return result


# Camera setup.
cam = csi.CSI()
cam.reset()
cam.pixformat(csi.GRAYSCALE)
cam.framesize(FRAME_SIZE)
cam.snapshot(time=2000)

# Focus once while aimed at the ground from the intended hover height.
try:
    cam.ioctl(csi.IOCTL_TRIGGER_AUTO_FOCUS)
    cam.ioctl(csi.IOCTL_WAIT_ON_AUTO_FOCUS, 5000)
    print("Autofocus complete")
except Exception as error:
    print("Autofocus skipped:", error)

# Reference frame for phase-correlation optical flow.
previous = image.Image(cam.width(), cam.height(), cam.pixformat())
previous.draw_image(cam.snapshot())

# I2C target memory exposed to the XRP.
shared = bytearray(PACKET_SIZE)
target = machine.I2CTarget(1, addr=I2C_ADDRESS, mem=shared)

try:
    green = machine.LED("LED_GREEN")
    red = machine.LED("LED_RED")
except Exception:
    green = None
    red = None

sequence = 0
last_frame_ms = time.ticks_ms()
last_print_ms = last_frame_ms

print("OpenMV optical flow ready")
print("I2C address: 0x%02X" % I2C_ADDRESS)
print("Frame size:", FRAME_SIZE)

while True:
    current = cam.snapshot()
    displacement = previous.find_displacement(current)
    previous.draw_image(current)

    dx = float(value_of(displacement, "x_translation"))
    dy = float(value_of(displacement, "y_translation"))
    quality = float(value_of(displacement, "response"))

    now_ms = time.ticks_ms()
    dt_ms = max(1, time.ticks_diff(now_ms, last_frame_ms))
    last_frame_ms = now_ms
    fps = 1000.0 / dt_ms

    valid = quality >= QUALITY_THRESHOLD
    send_dx = dx if valid else 0.0
    send_dy = dy if valid else 0.0

    sequence = (sequence + 1) & 0xFF
    flags = 1 if valid else 0

    packet = bytearray(PACKET_SIZE)
    struct.pack_into(
        "<BBBBhhHHH",
        packet,
        0,
        MAGIC,
        VERSION,
        sequence,
        flags,
        int(clamp(round(send_dx * 100), -32768, 32767)),
        int(clamp(round(send_dy * 100), -32768, 32767)),
        int(clamp(round(quality * 1000), 0, 1000)),
        int(clamp(round(fps * 10), 0, 65535)),
        int(clamp(dt_ms, 0, 65535)),
    )
    packet[14] = 0
    packet[15] = checksum(packet)

    shared[:] = packet

    if green is not None:
        if valid:
            green.on()
            red.off()
        else:
            green.off()
            red.on()

    if time.ticks_diff(now_ms, last_print_ms) >= PRINT_PERIOD_MS:
        print(
            "dx=%+.2f dy=%+.2f quality=%.3f fps=%.1f valid=%d"
            % (dx, dy, quality, fps, 1 if valid else 0)
        )
        last_print_ms = now_ms
