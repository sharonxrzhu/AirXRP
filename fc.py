from XRPLib.defaults import *
from msp import MSP
import time


class Drone:
    def __init__(self):
        self.msp = MSP()

        # -------------------------
        # RC channel defaults
        # -------------------------

        self.roll = 1500
        self.pitch = 1500
        self.throttle = 1000
        self.yaw = 1500

        self.aux1 = 1000
        self.aux2 = 1000
        self.aux3 = 1000
        self.aux4 = 1000

        # -------------------------
        # Movement settings
        # -------------------------

        self.max_pitch_offset = 100
        self.max_roll_offset = 100

        # -------------------------
        # Throttle settings
        # -------------------------

        self.min_throttle = 1000
        self.hover_throttle = 1500
        self.max_throttle = 1700
        self.throttle_step = 5

        # -------------------------
        # Student trim settings
        # -------------------------
        # These are XRP-side RC offsets.
        #
        # Example:
        # set_pitch_trim(-15) means pitch channel becomes 1485
        # when holding neutral attitude.
        #
        # This does NOT permanently change Betaflight acc_trim_pitch.
        # It just changes the RC command XRP sends.

        self.roll_trim = 0
        self.pitch_trim = 0

        # Keep this lower than Betaflight's acc_trim range.
        # This is an RC command offset, so 300 would be huge.
        self.max_trim = 80

        # -------------------------
        # RC stream settings
        # -------------------------

        # 50 Hz RC stream
        self.stream_delay_ms = 20

    # -------------------------
    # Utility
    # -------------------------

    def clamp(self, value, min_value, max_value):
        if value < min_value:
            return min_value

        if value > max_value:
            return max_value

        return value

    def send_current_channels(self):
        channels = [
            self.roll,
            self.pitch,
            self.throttle,
            self.yaw,
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

    # -------------------------
    # Trim commands
    # -------------------------

    def set_pitch_trim(self, trim):
        """
        Set pitch trim as an XRP-side RC offset.

        Example:
        set_pitch_trim(-15) gives pitch = 1485 at neutral.

        Positive = trims forward
        Negative = trims backward
        """

        self.pitch_trim = int(self.clamp(
            trim,
            -self.max_trim,
            self.max_trim
        ))

    def set_roll_trim(self, trim):
        """
        Set roll trim as an XRP-side RC offset.

        Example:
        set_roll_trim(8) gives roll = 1508 at neutral.

        Positive = trims right
        Negative = trims left
        """

        self.roll_trim = int(self.clamp(
            trim,
            -self.max_trim,
            self.max_trim
        ))

    # -------------------------
    # Basic attitude commands
    # -------------------------

    def set_pitch_speed(self, speed):
        """
        Set pitch command using a speed from -1.0 to 1.0.

        Positive = forward
        Negative = backward
        """

        speed = self.clamp(speed, -1.0, 1.0)
        offset = int(speed * self.max_pitch_offset)

        self.pitch = 1500 + self.pitch_trim + offset
        self.send_current_channels()

    def set_roll_speed(self, speed):
        """
        Set roll command using a speed from -1.0 to 1.0.

        Positive = right
        Negative = left
        """

        speed = self.clamp(speed, -1.0, 1.0)
        offset = int(speed * self.max_roll_offset)

        self.roll = 1500 + self.roll_trim + offset
        self.send_current_channels()

    # -------------------------
    # Throttle / takeoff / land
    # -------------------------

    def set_hover_throttle(self, throttle):
        """
        Set estimated hover throttle.
        """

        self.hover_throttle = int(self.clamp(
            throttle,
            self.min_throttle,
            self.max_throttle
        ))

    def set_throttle(self, throttle):
        """
        Set throttle directly.
        """

        self.throttle = int(self.clamp(
            throttle,
            self.min_throttle,
            self.max_throttle
        ))

        self.send_current_channels()

    def ramp_throttle_to(self, target_throttle, step_delay_ms=50):
        """
        Smoothly ramp throttle to a target value.
        """

        target_throttle = int(self.clamp(
            target_throttle,
            self.min_throttle,
            self.max_throttle
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

    def takeoff(self, throttle=None, duration=1.5, takeoff_throttle=None, punch_duration=0.35):
        """
        Ramp up to a higher takeoff throttle briefly,
        then settle down to hover throttle and hold trimmed neutral attitude.

        throttle:
            Hover throttle after takeoff.

        takeoff_throttle:
            Higher throttle used briefly to lift off.

        punch_duration:
            How long to hold takeoff throttle before lowering to hover throttle.
        """

        if throttle is None:
            throttle = self.hover_throttle

        if takeoff_throttle is None:
            takeoff_throttle = throttle + 80

        throttle = int(self.clamp(
            throttle,
            self.min_throttle,
            self.max_throttle
        ))

        takeoff_throttle = int(self.clamp(
            takeoff_throttle,
            self.min_throttle,
            self.max_throttle
        ))

        self.enable_angle_mode()
        self.stop_motion()

        # Punch up to get off the ground
        self.ramp_throttle_to(takeoff_throttle)
        self.stream_for(punch_duration)

        # Settle back to hover throttle
        self.ramp_throttle_to(throttle)
        self.stream_for(duration)

    def land(self):
        """
        Center roll/pitch/yaw and ramp throttle down.
        """

        self.stop_motion()
        self.ramp_throttle_to(self.min_throttle)

    # -------------------------
    # Movement commands
    # -------------------------

    def move_forward(self, speed=0.3, duration=1.0, throttle=None):
        """
        Move forward while keeping throttle.
        """

        if throttle is None:
            throttle = self.hover_throttle

        self.set_throttle(throttle)
        self.set_pitch_speed(abs(speed))
        self.stream_for(duration)
        self.stop_motion()

    def move_backward(self, speed=0.3, duration=1.0, throttle=None):
        """
        Move backward while keeping throttle.
        """

        if throttle is None:
            throttle = self.hover_throttle

        self.set_throttle(throttle)
        self.set_pitch_speed(-abs(speed))
        self.stream_for(duration)
        self.stop_motion()

    def move_right(self, speed=0.3, duration=1.0, throttle=None):
        """
        Move right while keeping throttle.
        """

        if throttle is None:
            throttle = self.hover_throttle

        self.set_throttle(throttle)
        self.set_roll_speed(abs(speed))
        self.stream_for(duration)
        self.stop_motion()

    def move_left(self, speed=0.3, duration=1.0, throttle=None):
        """
        Move left while keeping throttle.
        """

        if throttle is None:
            throttle = self.hover_throttle

        self.set_throttle(throttle)
        self.set_roll_speed(-abs(speed))
        self.stream_for(duration)
        self.stop_motion()

    # -------------------------
    # Stop / hold
    # -------------------------

    def stop_motion(self):
        """
        Center roll, pitch, and yaw using student trim.
        Leave throttle unchanged.
        """

        self.roll = 1500 + self.roll_trim
        self.pitch = 1500 + self.pitch_trim
        self.yaw = 1500

        self.send_current_channels()

    def hold_attitude(self, duration=1.0):
        """
        Hold trimmed neutral attitude with current throttle.
        """

        self.stop_motion()
        self.stream_for(duration)

    def hold_neutral(self, duration=1.0):
        """
        Same as hold_attitude.
        Kept for backwards compatibility.
        """

        self.hold_attitude(duration)

    def stop_all(self):
        """
        Full stop: neutral motion and minimum throttle.
        Ignores trim.
        """

        self.roll = 1500
        self.pitch = 1500
        self.yaw = 1500
        self.throttle = self.min_throttle

        self.send_current_channels()

    # -------------------------
    # AUX read helpers
    # -------------------------

    def get_aux2(self):
        """
        Read AUX2 from the flight controller.

        Returns:
        AUX2 value, usually around 1000 to 2000.

        If it fails, returns None.
        """

        try:
            channels = self.msp.get_rc()
        except:
            return None

        if channels is None:
            return None

        if len(channels) < 6:
            return None

        return channels[5]

    def get_aux3(self):
        """
        Read AUX3 from the flight controller.

        Returns:
        AUX3 value, usually around 1000 to 2000.

        If it fails, returns None.
        """

        try:
            channels = self.msp.get_rc()
        except:
            return None

        if channels is None:
            return None

        if len(channels) < 7:
            return None

        return channels[6]

    # angle mode
    def enable_angle_mode(self):
        """
        Enable Betaflight Angle Mode.
        """

        self.aux1 = 2000
        self.send_current_channels()
