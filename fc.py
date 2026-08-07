"""
functions to use:
    move_forward()
    move_backward()
    move_left()
    move_right()
    hover()
"""

import time

from msp import MSP
from hover import FlowHover


class Drone:

    def __init__(
        self,
        hover_throttle=1500,
        min_throttle=1000,
        max_throttle=1700,
        max_pitch_offset=100,
        max_roll_offset=100,
        roll_center=1500,
        pitch_center=1500,
        stream_period_ms=20,
    ):
        self._msp = MSP()

        self._min_throttle = int(min_throttle)
        self._max_throttle = int(max_throttle)
        self._hover_throttle = int(self._clamp(
            hover_throttle,
            self._min_throttle,
            self._max_throttle,
        ))

        self._max_pitch_offset = int(max_pitch_offset)
        self._max_roll_offset = int(max_roll_offset)
        self._roll_center = int(roll_center)
        self._pitch_center = int(pitch_center)
        self._stream_period_ms = int(stream_period_ms)

        self._roll = self._roll_center
        self._pitch = self._pitch_center
        self._yaw = 1500
        self._throttle = self._min_throttle

        # AUX placeholders. Betaflight decides which MSP channels are used
        # through msp_override_channels_mask.
        self._aux1 = 1000
        self._aux2 = 1000
        self._aux3 = 1000
        self._aux4 = 1000

        self._flow_hover = None

    # ------------------------------------------------------------------
    # movement function to use
    # ------------------------------------------------------------------

    def move_forward(self, speed=0.3, duration=1.0, throttle=None):
        """Move forward for ``duration`` seconds, then center motion."""
        self._move(
            pitch_speed=abs(speed),
            roll_speed=0.0,
            duration=duration,
            throttle=throttle,
        )

    def move_backward(self, speed=0.3, duration=1.0, throttle=None):
        """Move backward for ``duration`` seconds, then center motion."""
        self._move(
            pitch_speed=-abs(speed),
            roll_speed=0.0,
            duration=duration,
            throttle=throttle,
        )

    def move_left(self, speed=0.3, duration=1.0, throttle=None):
        """Move left for ``duration`` seconds, then center motion."""
        self._move(
            pitch_speed=0.0,
            roll_speed=-abs(speed),
            duration=duration,
            throttle=throttle,
        )

    def move_right(self, speed=0.3, duration=1.0, throttle=None):
        """Move right for ``duration`` seconds, then center motion."""
        self._move(
            pitch_speed=0.0,
            roll_speed=abs(speed),
            duration=duration,
            throttle=throttle,
        )

    def hover(self, duration=5.0, throttle=None, print_status=False):
        """Hold horizontal X/Y position using OpenMV optical flow.

        This is not altitude hold. When throttle is included in Betaflight's
        MSP override mask, ``throttle`` is a fixed throttle command. Otherwise,
        the pilot continues to control throttle.
        """
        if throttle is None:
            throttle = self._hover_throttle

        self._set_throttle(throttle)
        self._set_roll_pitch(self._roll_center, self._pitch_center)

        if self._flow_hover is None:
            self._flow_hover = FlowHover(self)

        return self._flow_hover.run(
            duration=duration,
            print_status=print_status,
        )

    def stop(self, duration=0.20):
        """Stop horizontal motion while keeping the present throttle.

        The neutral command is streamed briefly so Betaflight receives more
        than one centered RC frame. This does not disarm or reduce throttle.
        """
        self._set_roll_pitch(self._roll_center, self._pitch_center)
        self._yaw = 1500
        self._stream_for(duration)

    # ------------------------------------------------------------------
    # Internal helpers used by the movement and hover controllers
    # ------------------------------------------------------------------

    @staticmethod
    def _clamp(value, minimum, maximum):
        return max(minimum, min(maximum, value))

    def _set_throttle(self, throttle):
        self._throttle = int(self._clamp(
            throttle,
            self._min_throttle,
            self._max_throttle,
        ))

    def _set_roll_pitch(self, roll, pitch):
        self._roll = int(round(self._clamp(roll, 1000, 2000)))
        self._pitch = int(round(self._clamp(pitch, 1000, 2000)))

    def _send(self):
        # MSP_SET_RAW_RC channel order:
        # roll, pitch, yaw, throttle, aux1, aux2, aux3, aux4
        self._msp.send_raw_rc([
            self._roll,
            self._pitch,
            self._yaw,
            self._throttle,
            self._aux1,
            self._aux2,
            self._aux3,
            self._aux4,
        ])

    def _stream_for(self, duration):
        if duration is None or duration < 0:
            raise ValueError("duration must be zero or greater")

        start_ms = time.ticks_ms()

        while time.ticks_diff(time.ticks_ms(), start_ms) < duration * 1000:
            self._send()
            time.sleep_ms(self._stream_period_ms)

    def _move(self, pitch_speed, roll_speed, duration, throttle):
        if duration is None or duration <= 0:
            raise ValueError("duration must be greater than zero")

        if throttle is None:
            throttle = self._hover_throttle

        pitch_speed = self._clamp(pitch_speed, -1.0, 1.0)
        roll_speed = self._clamp(roll_speed, -1.0, 1.0)

        pitch = self._pitch_center + int(
            pitch_speed * self._max_pitch_offset
        )
        roll = self._roll_center + int(
            roll_speed * self._max_roll_offset
        )

        self._set_throttle(throttle)
        self._set_roll_pitch(roll, pitch)
        self._yaw = 1500

        try:
            self._stream_for(duration)
        finally:
            self.stop()
