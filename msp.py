from machine import Pin, UART
import time

#betaflight msp command numbers
MSP_SET_RAW_RC = 200 # rc channel values to fc
MSP_ACC_CALIBRATION = 205 #calibrate accerameter
MSP_RC = 105 # current rc channel values for xrp

class MSP: #resuable object for communication through MSP
    #uart connection (xrp gpio tx = 0, rx = 1)
    def __init__(self, uart_id=0, tx_pin=0, rx_pin=1, baudrate=115200):
        self.uart = UART(uart_id, baudrate=baudrate, tx=Pin(tx_pin), rx=Pin(rx_pin))

    def checksum(self, data):
        c = 0
        for b in data:
            c ^= b
        return c

    def send(self, cmd, payload=b""):
        """
        Send MSP v1 packet:
        $ M < payload_size command payload checksum
        """
        size = len(payload)
        checksum_data = bytes([size, cmd]) + payload
        cs = self.checksum(checksum_data)

        packet = bytes([36, 77, 60, size, cmd]) + payload + bytes([cs])
        self.uart.write(packet) #xrp to fc

    def send_request(self, cmd):
        """
        Send MSP v1 request with no payload.
        """
        self.send(cmd, b"")

    def read_exact(self, num_bytes, timeout_ms=500):
        start = time.ticks_ms()
        data = b""

        while len(data) < num_bytes:
            if time.ticks_diff(time.ticks_ms(), start) > timeout_ms:
                return None

            if self.uart.any():
                chunk = self.uart.read(num_bytes - len(data))
                if chunk:
                    data += chunk

            time.sleep_ms(2)

        return data

    #fc to xrp
    def read_response(self, timeout_ms=500):
        """
        Read one MSP v1 response.
        Expected format:
        $ M > payload_size command payload checksum

        Returns:
        (cmd, payload) or None
        """
        start = time.ticks_ms()
        state = 0
        
        # look for header one byte at a time
        while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
            if not self.uart.any():
                time.sleep_ms(2)
                continue

            b = self.uart.read(1)
            if not b:
                continue

            b = b[0]

            if state == 0:
                if b == 36:  # $
                    state = 1

            elif state == 1:
                if b == 77:  # M
                    state = 2
                else:
                    state = 0

            elif state == 2:
                if b == 62 or b == 33:  # > normal response, ! error response
                    is_error = b == 33

                    header = self.read_exact(2, timeout_ms)
                    if header is None:
                        return None

                    payload_size = header[0]
                    cmd = header[1]

                    payload = self.read_exact(payload_size, timeout_ms)
                    if payload is None:
                        return None

                    rx_checksum = self.read_exact(1, timeout_ms)
                    if rx_checksum is None:
                        return None

                    rx_checksum = rx_checksum[0]
                    calc_checksum = self.checksum(bytes([payload_size, cmd]) + payload)

                    if calc_checksum != rx_checksum:
                        print("MSP checksum failed")
                        return None

                    if is_error:
                        print("MSP error response for command:", cmd)
                        return None

                    return cmd, payload

                else:
                    state = 0

        return None

    def request(self, cmd, timeout_ms=500):
        """
        Send an MSP request and wait for the matching response.
        Ignore delayed responses from other commands.
        """
    
        while self.uart.any():
            self.uart.read()
    
        self.send_request(cmd)
    
        start = time.ticks_ms()
    
        while True:
            elapsed = time.ticks_diff(time.ticks_ms(), start)
            remaining = timeout_ms - elapsed
    
            if remaining <= 0:
                return None
    
            response = self.read_response(remaining)
    
            if response is None:
                return None
    
            response_cmd, payload = response
    
            if response_cmd == cmd:
                return payload
    
            if response_cmd == MSP_SET_RAW_RC:
                continue
    
            print("Ignoring MSP response:", response_cmd)

    def u16_le(self, value):
        return bytes([value & 0xFF, (value >> 8) & 0xFF])

    def bytes_to_u16_le(self, low, high):
        return low | (high << 8)

    def send_raw_rc(self, channels):
        """
        Send RC channel override values to Betaflight.
        Example:
        [1500, 1530, 1000, 1500, 1000, 1000, 1000, 1000]
        """
        payload = b""

        for ch in channels:
            payload += self.u16_le(ch)

        self.send(MSP_SET_RAW_RC, payload)

    def get_rc_channels(self):
        """
        Ask Betaflight for current receiver channel values.

        Returns:
        [roll, pitch, throttle, yaw, aux1, aux2, aux3, aux4, ...]
        or None if no response.
        """
        payload = self.request(MSP_RC)

        if payload is None:
            return None

        channels = []

        for i in range(0, len(payload), 2):
            if i + 1 < len(payload):
                value = self.bytes_to_u16_le(payload[i], payload[i + 1])
                channels.append(value)

        return channels

    def get_rc(self):
        """
        Simple alias for get_rc_channels().
        """

        return self.get_rc_channels()
        
    def calibrate_accelerometer(self, timeout_ms=2000):
        """
        Tell Betaflight to calibrate the accelerometer.
    
        The drone must be completely still on a level surface.
        Motors must be disarmed.
        """
    
        payload = self.request(
            MSP_ACC_CALIBRATION,
            timeout_ms=timeout_ms
        )
    
        return payload is not None
