"""
Reusable OpenMV optical-flow hover controller for AirXRP.

This holds horizontal drift (roll/pitch). It is NOT a true altitude-hold
controller. Altitude is controlled by the pilot or by the fixed throttle
already being sent by the Drone object.
"""

import struct
import time

from machine import I2C, Pin


def _clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def _apply_deadband(value, deadband):
    if abs(value) <= deadband:
        return 0.0
    if value > 0:
        return value - deadband
    return value + deadband


def _slew(current, target, maximum_change):
    if target > current + maximum_change:
        return current + maximum_change
    if target < current - maximum_change:
        return current - maximum_change
    return target


def _parse_key_value_file(filename):
    values = {}
    with open(filename, "r") as file_object:
        for line in file_object:
            line = line.strip()
            if not line or "=" not in line:
                continue
            key, value = line.split("=", 1)
            values[key.strip()] = value.strip()
    return values


class FlowHover:
    """Blocking optical-flow X/Y hold, designed to be called by Drone.hover()."""

    PACKET_SIZE = 16
    PACKET_MAGIC = 0xA5
    PACKET_VERSION = 1

    def __init__(
        self,
        drone,
        calibration_file="/openmv_axis_calibration.txt",
        i2c_bus=1,
        sda_pin=38,
        scl_pin=39,
        i2c_frequency=400000,
        openmv_address=0x42,
        quality_min=0.30,
        fps_min=30.0,
        stale_ms=150,
        max_abs_per_frame=4.0,
        rate_deadband_px_s=1.5,
        filter_alpha=0.35,
        kp_us_per_px_s=0.35,
        max_rc_offset_us=10.0,
        output_slew_us_per_s=50.0,
        invalid_hold_ms=250,
        loop_period_ms=20,
        print_period_ms=250,
    ):
        self.drone = drone
        self.openmv_address = openmv_address
        self.quality_min = quality_min
        self.fps_min = fps_min
        self.stale_ms = stale_ms
        self.max_abs_per_frame = max_abs_per_frame
        self.rate_deadband_px_s = rate_deadband_px_s
        self.filter_alpha = filter_alpha
        self.kp_us_per_px_s = kp_us_per_px_s
        self.max_rc_offset_us = max_rc_offset_us
        self.output_slew_us_per_s = output_slew_us_per_s
        self.invalid_hold_ms = invalid_hold_ms
        self.loop_period_ms = loop_period_ms
        self.print_period_ms = print_period_ms

        calibration = _parse_key_value_file(calibration_file)
        if calibration.get("valid", "0") != "1":
            raise RuntimeError("Missing valid OpenMV axis calibration")

        self.inverse_00 = float(calibration["inverse_00"])
        self.inverse_01 = float(calibration["inverse_01"])
        self.inverse_10 = float(calibration["inverse_10"])
        self.inverse_11 = float(calibration["inverse_11"])

        self.i2c = I2C(
            i2c_bus,
            sda=Pin(sda_pin),
            scl=Pin(scl_pin),
            freq=i2c_frequency,
        )

        if self.openmv_address not in self.i2c.scan():
            raise RuntimeError("OpenMV not found at 0x42")

        self.last_sequence = None
        self.last_new_ms = None
        self.reset()

    def reset(self):
        self.filtered_right = 0.0
        self.filtered_forward = 0.0
        self.roll_offset = 0.0
        self.pitch_offset = 0.0

    def _packet_checksum(self, packet):
        value = 0
        for index in range(self.PACKET_SIZE - 1):
            value ^= packet[index]
        return value

    def _read_flow(self, now_ms):
        packet = self.i2c.readfrom_mem(
            self.openmv_address,
            0,
            self.PACKET_SIZE,
        )

        if len(packet) != self.PACKET_SIZE:
            raise RuntimeError("Wrong OpenMV packet length")
        if packet[0] != self.PACKET_MAGIC:
            raise RuntimeError("Wrong OpenMV packet magic")
        if packet[1] != self.PACKET_VERSION:
            raise RuntimeError("Wrong OpenMV packet version")
        if packet[15] != self._packet_checksum(packet):
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
        ) = struct.unpack_from("<BBBBhhHHH", packet, 0)

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

    def _flow_is_valid(self, flow):
        if flow is None:
            return False, "no_flow"
        if flow["age_ms"] > self.stale_ms:
            return False, "stale_flow"
        if not flow["camera_valid"]:
            return False, "camera_invalid"
        if flow["quality"] < self.quality_min:
            return False, "low_quality"
        if flow["fps"] < self.fps_min:
            return False, "low_fps"
        if (
            abs(flow["dx"]) > self.max_abs_per_frame
            or abs(flow["dy"]) > self.max_abs_per_frame
        ):
            return False, "flow_too_large"
        return True, "ok"

    def _update(self, dx, dy, frame_dt_ms, dt_seconds):
        body_right_per_frame = self.inverse_00 * dx + self.inverse_01 * dy
        body_forward_per_frame = self.inverse_10 * dx + self.inverse_11 * dy

        frame_dt_seconds = max(0.001, frame_dt_ms / 1000.0)
        body_right = body_right_per_frame / frame_dt_seconds
        body_forward = body_forward_per_frame / frame_dt_seconds

        self.filtered_right += self.filter_alpha * (
            body_right - self.filtered_right
        )
        self.filtered_forward += self.filter_alpha * (
            body_forward - self.filtered_forward
        )

        effective_right = _apply_deadband(
            self.filtered_right,
            self.rate_deadband_px_s,
        )
        effective_forward = _apply_deadband(
            self.filtered_forward,
            self.rate_deadband_px_s,
        )

        # Preserves the roll-flipped behavior from the tested controller.
        target_roll_offset = _clamp(
            self.kp_us_per_px_s * effective_right,
            -self.max_rc_offset_us,
            self.max_rc_offset_us,
        )
        target_pitch_offset = _clamp(
            -self.kp_us_per_px_s * effective_forward,
            -self.max_rc_offset_us,
            self.max_rc_offset_us,
        )

        maximum_change = self.output_slew_us_per_s * dt_seconds
        self.roll_offset = _slew(
            self.roll_offset,
            target_roll_offset,
            maximum_change,
        )
        self.pitch_offset = _slew(
            self.pitch_offset,
            target_pitch_offset,
            maximum_change,
        )

        return {
            "body_right": body_right,
            "body_forward": body_forward,
            "roll_offset": self.roll_offset,
            "pitch_offset": self.pitch_offset,
        }

    def run(self, duration=5.0, throttle=None, print_status=True):
        """
        Hold horizontal position for duration seconds.

        This blocks, just like move_forward(). It leaves throttle unchanged
        unless a throttle value is supplied.
        """
        if duration is None or duration <= 0:
            raise ValueError("duration must be greater than zero")

        if throttle is not None:
            self.drone.throttle = int(self.drone.clamp(
                throttle,
                self.drone.min_throttle,
                self.drone.max_throttle,
            ))

        self.reset()
        start_ms = time.ticks_ms()
        last_loop_ms = start_ms
        last_print_ms = start_ms
        last_good_ms = None
        valid_frames = 0
        invalid_frames = 0
        last_reason = "starting"

        result = {
            "body_right": 0.0,
            "body_forward": 0.0,
            "roll_offset": 0.0,
            "pitch_offset": 0.0,
        }

        try:
            while time.ticks_diff(time.ticks_ms(), start_ms) < duration * 1000:
                loop_start_ms = time.ticks_ms()
                dt_seconds = max(
                    0.001,
                    time.ticks_diff(loop_start_ms, last_loop_ms) / 1000.0,
                )
                last_loop_ms = loop_start_ms
                flow = None

                try:
                    flow = self._read_flow(loop_start_ms)
                    valid, reason = self._flow_is_valid(flow)
                except Exception:
                    valid = False
                    reason = "openmv_read_error"

                if valid:
                    valid_frames += 1
                    result = self._update(
                        flow["dx"],
                        flow["dy"],
                        flow["frame_dt_ms"],
                        dt_seconds,
                    )
                    self.drone.roll = int(round(
                        1500 + self.drone.roll_trim + result["roll_offset"]
                    ))
                    self.drone.pitch = int(round(
                        1500 + self.drone.pitch_trim + result["pitch_offset"]
                    ))
                    last_good_ms = loop_start_ms
                else:
                    invalid_frames += 1
                    holding_last = (
                        last_good_ms is not None
                        and time.ticks_diff(loop_start_ms, last_good_ms)
                        <= self.invalid_hold_ms
                    )

                    if holding_last:
                        reason = reason + "_hold"
                    else:
                        self.reset()
                        self.drone.roll = 1500 + self.drone.roll_trim
                        self.drone.pitch = 1500 + self.drone.pitch_trim

                self.drone.yaw = 1500
                self.drone.send_current_channels()
                last_reason = reason

                if (
                    print_status
                    and time.ticks_diff(loop_start_ms, last_print_ms)
                    >= self.print_period_ms
                ):
                    print(
                        "hover valid={} reason={} dx={:+.2f} dy={:+.2f} "
                        "q={:.2f} roll={} pitch={}".format(
                            valid,
                            reason,
                            flow["dx"] if flow else 0.0,
                            flow["dy"] if flow else 0.0,
                            flow["quality"] if flow else 0.0,
                            self.drone.roll,
                            self.drone.pitch,
                        )
                    )
                    last_print_ms = loop_start_ms

                elapsed_ms = time.ticks_diff(time.ticks_ms(), loop_start_ms)
                sleep_ms = self.loop_period_ms - elapsed_ms
                if sleep_ms > 0:
                    time.sleep_ms(sleep_ms)
        finally:
            self.reset()
            self.drone.stop_motion()

        return {
            "valid_frames": valid_frames,
            "invalid_frames": invalid_frames,
            "last_reason": last_reason,
        }
