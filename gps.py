from machine import UART, Pin
import time
import math


class AirXRPGPS:
    def __init__(self):
        # UART 1, 9600 baud
        # GPIO8 = XRP TX -> GPS RX
        # GPIO9 = XRP RX <- GPS TX
        self.uart = UART(
            1,
            baudrate=9600,
            tx=Pin(8),
            rx=Pin(9),
            bits=8,
            parity=None,
            stop=1,
        )

        self.buffer = b""

        self.origin_lat = None
        self.origin_lon = None
        self.origin_alt = None

        self.last_location = None

    def _nmea_to_decimal(self, raw, direction):
        if not raw:
            return None

        dot = raw.find(".")
        if dot == -1:
            return None

        deg_len = dot - 2

        degrees = int(raw[:deg_len])
        minutes = float(raw[deg_len:])

        decimal = degrees + minutes / 60.0

        if direction in ("S", "W"):
            decimal = -decimal

        return decimal

    def _parse_gga(self, sentence):
        parts = sentence.split(",")

        if len(parts) < 10:
            return None

        fix_quality = parts[6]

        if fix_quality == "0":
            return None

        lat = self._nmea_to_decimal(parts[2], parts[3])
        lon = self._nmea_to_decimal(parts[4], parts[5])

        if lat is None or lon is None:
            return None

        try:
            altitude_m = float(parts[9])
        except:
            altitude_m = None

        return {
            "latitude": lat,
            "longitude": lon,
            "altitude_m": altitude_m,
        }

    def read_sentence(self, timeout_ms=1000):
        """
        Read one complete NMEA sentence from the GPS.
        No sleep delay.
        """

        start = time.ticks_ms()

        while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
            if self.uart.any():
                data = self.uart.read()
                if data:
                    self.buffer += data

            if b"\n" in self.buffer:
                line, self.buffer = self.buffer.split(b"\n", 1)

                try:
                    return line.decode("ascii", "ignore").strip()
                except:
                    return None

        return None

    def get_location(self, timeout_ms=3000):
        """
        Return latest GPS latitude, longitude, altitude.
        """

        start = time.ticks_ms()

        while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
            sentence = self.read_sentence(timeout_ms=500)

            if not sentence:
                continue

            if sentence.startswith("$GNGGA") or sentence.startswith("$GPGGA"):
                location = self._parse_gga(sentence)

                if location:
                    self.last_location = location
                    return location

        return self.last_location

    def set_origin(self):
        """
        Use the current GPS location as x=0, y=0, z=0.
        No sleep delay.
        """

        print("Waiting for GPS fix to set origin...")

        while True:
            location = self.get_location(timeout_ms=3000)

            if location:
                self.origin_lat = location["latitude"]
                self.origin_lon = location["longitude"]
                self.origin_alt = location["altitude_m"]

                print("Origin set")
                print("x: 0 m, y: 0 m, z: 0 m")
                return

            print("No GPS fix yet...")

    def get_xyz(self):
        """
        Return current position relative to origin.

        x = meters east
        y = meters north
        z = meters up
        """

        if self.origin_lat is None or self.origin_lon is None:
            self.set_origin()

        location = self.get_location(timeout_ms=3000)

        if not location:
            return None

        lat = location["latitude"]
        lon = location["longitude"]
        alt = location["altitude_m"]

        meters_per_deg_lat = 111_320.0
        meters_per_deg_lon = 111_320.0 * math.cos(math.radians(self.origin_lat))

        x = (lon - self.origin_lon) * meters_per_deg_lon
        y = (lat - self.origin_lat) * meters_per_deg_lat

        if alt is not None and self.origin_alt is not None:
            z = alt - self.origin_alt
        else:
            z = None

        return {
            "x": x,
            "y": y,
            "z": z,
        }

    def print_xyz_loop(self):
        """
        Constantly print x, y, z position relative to startup origin.
        No sleep delay.
        """

        self.set_origin()

        while True:
            pos = self.get_xyz()

            if pos:
                z_value = round(pos["z"], 2) if pos["z"] is not None else None

                print(
                    "x:",
                    round(pos["x"], 2),
                    "m, y:",
                    round(pos["y"], 2),
                    "m, z:",
                    z_value,
                    "m",
                )
            else:
                print("No GPS position")


gps = AirXRPGPS()
