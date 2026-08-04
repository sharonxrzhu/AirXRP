"""AirXRP Drone library with reusable OpenMV horizontal-hover support."""

from XRPLib.defaults import *
from msp import MSP
from flow_hover import FlowHover
import time


class Drone:
    def __init__(self):
        self.msp = MSP()

        self.roll = 1500
        self.pitch = 1500
        self.throttle = 1000
        self.yaw = 1500

        self.aux1 = 1000
        self.aux2 = 1000
        self.aux3 = 1000
        self.aux4 = 1000

        self.max_pitch_offset = 100
        self.max_roll_offset = 100

        self.min_throttle = 1000
        self.hover_throttle = 1500
        self.max_throttle = 1700
        self.throttle_step = 5

        self.roll_trim = 0
        self.pitch_trim = 0
        self.max_trim = 80

        self.stream_delay_ms = 20
        self._flow_hover = None

    def clamp(self, value, min_value, max_value):
        if value < min_value:
            return min_value
        if value > max_value:
            return max_value
        return value

    def send_current_channels(self):
        # Correct MSP RC order:
        # roll, pitch, yaw, throttle, aux1, aux2, aux3, aux4
        channels = [
            self.roll,
            self.pitch,
            self.yaw,
            self.throttle,
            self.aux1,
            self.aux2,
            self.aux3,
            self.aux4,
        ]
        self.msp.send_raw_rc(channels)

    def stream_for(self, seconds):
        start = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), start) < seconds * 1000:
            self.send_current_channels()
            time.sleep_ms(self.stream_delay_ms)

    def set_pitch_trim(self, trim):
        self.pitch_trim = int(self.clamp(
            trim,
            -self.max_trim,
            self.max_trim,
        ))

    def set_roll_trim(self, trim):
        self.roll_trim = int(self.clamp(
            trim,
            -self.max_trim,
            self.max_trim,
        ))

    def set_pitch_speed(self, speed):
        speed = self.clamp(speed, -1.0, 1.0)
        offset = int(speed * self.max_pitch_offset)
        self.pitch = 1500 + self.pitch_trim + offset
        self.send_current_channels()

    def set_roll_speed(self, speed):
        speed = self.clamp(speed, -1.0, 1.0)
        offset = int(speed * self.max_roll_offset)
        self.roll = 1500 + self.roll_trim + offset
        self.send_current_channels()

    def set_hover_throttle(self, throttle):
        self.hover_throttle = int(self.clamp(
            throttle,
            self.min_throttle,
            self.max_throttle,
        ))

    def set_throttle(self, throttle):
        self.throttle = int(self.clamp(
            throttle,
            self.min_throttle,
            self.max_throttle,
        ))
        self.send_current_channels()

    def ramp_throttle_to(self, target_throttle, step_delay_ms=50):
        target_throttle = int(self.clamp(
            target_throttle,
            self.min_throttle,
            self.max_throttle,
        ))

        while self.throttle != target_throttle:
            if self.throttle < target_throttle:
                self.throttle += self.throttle_step
                if self.throttle > target_throttle:
                    self.throttle = target_throttle
            else:
                self.throttle -= self.throttle_step
                if self.throttle < target_throttle:
                    self.throttle = target_throttle

            self.send_current_channels()
            time.sleep_ms(step_delay_ms)

    def takeoff(
        self,
        throttle=None,
        duration=1.5,
        takeoff_throttle=None,
        punch_duration=0.35,
    ):
        if throttle is None:
            throttle = self.hover_throttle
        if takeoff_throttle is None:
            takeoff_throttle = throttle + 80

        throttle = int(self.clamp(
            throttle,
            self.min_throttle,
            self.max_throttle,
        ))
        takeoff_throttle = int(self.clamp(
            takeoff_throttle,
            self.min_throttle,
            self.max_throttle,
        ))

        self.enable_angle_mode()
        self.stop_motion()
        self.ramp_throttle_to(takeoff_throttle)
        self.stream_for(punch_duration)
        self.ramp_throttle_to(throttle)
        self.stream_for(duration)

    def land(self):
        self.stop_motion()
        self.ramp_throttle_to(self.min_throttle)

    def move_forward(self, speed=0.3, duration=1.0, throttle=None):
        if throttle is None:
            throttle = self.hover_throttle
        self.set_throttle(throttle)
        self.set_pitch_speed(abs(speed))
        self.stream_for(duration)
        self.stop_motion()

    def move_backward(self, speed=0.3, duration=1.0, throttle=None):
        if throttle is None:
            throttle = self.hover_throttle
        self.set_throttle(throttle)
        self.set_pitch_speed(-abs(speed))
        self.stream_for(duration)
        self.stop_motion()

    def move_right(self, speed=0.3, duration=1.0, throttle=None):
        if throttle is None:
            throttle = self.hover_throttle
        self.set_throttle(throttle)
        self.set_roll_speed(abs(speed))
        self.stream_for(duration)
        self.stop_motion()

    def move_left(self, speed=0.3, duration=1.0, throttle=None):
        if throttle is None:
            throttle = self.hover_throttle
        self.set_throttle(throttle)
        self.set_roll_speed(-abs(speed))
        self.stream_for(duration)
        self.stop_motion()

    def hover(self, duration=5.0, throttle=None, print_status=True):
        """
        Hold horizontal position using OpenMV optical flow.

        This is blocking, like move_forward(). It holds X/Y drift only;
        it does not measure or regulate altitude.

        Betaflight override masks:
          3  = XRP roll/pitch, pilot throttle
          11 = XRP roll/pitch/throttle
        """
        if throttle is None:
            throttle = self.hover_throttle

        self.throttle = int(self.clamp(
            throttle,
            self.min_throttle,
            self.max_throttle,
        ))

        self.enable_angle_mode()
        self.stop_motion()

        if self._flow_hover is None:
            self._flow_hover = FlowHover(self)

        return self._flow_hover.run(
            duration=duration,
            throttle=self.throttle,
            print_status=print_status,
        )

    def stop_motion(self):
        self.roll = 1500 + self.roll_trim
        self.pitch = 1500 + self.pitch_trim
        self.yaw = 1500
        self.send_current_channels()

    def hold_attitude(self, duration=1.0):
        self.stop_motion()
        self.stream_for(duration)

    def hold_neutral(self, duration=1.0):
        self.hold_attitude(duration)

    def stop_all(self):
        self.roll = 1500
        self.pitch = 1500
        self.yaw = 1500
        self.throttle = self.min_throttle
        self.send_current_channels()

    def get_aux2(self):
        try:
            channels = self.msp.get_rc()
        except Exception:
            return None
        if channels is None or len(channels) < 6:
            return None
        return channels[5]

    def get_aux3(self):
        try:
            channels = self.msp.get_rc()
        except Exception:
            return None
        if channels is None or len(channels) < 7:
            return None
        return channels[6]

    def enable_angle_mode(self):
        self.aux1 = 2000
        self.send_current_channels()
