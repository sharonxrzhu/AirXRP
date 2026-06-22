import sys
sys.path.append("/main")

from XRPLib.defaults import *
from fc import Drone
import time


drone = Drone()

# -------------------------
# Tuning values
# -------------------------

drone.set_pitch_trim(-47)
drone.set_roll_trim(-4)

# Begin in a safe stopped condition
drone.stop_all()

# -------------------------
# AUX3 mission trigger
# -------------------------

AUX3_HIGH = 1900
AUX3_LOW = 1300

mission_ready = True

print("AirXRP ready. Flip AUX3 high to start mission.")


while True:
    aux3 = drone.get_aux3()

    if aux3 is None:
        # Communication failed. Do not start anything.
        time.sleep_ms(100)
        continue

    if mission_ready and aux3 >= AUX3_HIGH:
        mission_ready = False

        print("AUX3 high, starting mission")

        drone.takeoff(
            settle_throttle=1495,
            takeoff_throttle=1500,
            punch_duration=0.35,
            duration=1.5
        )

        drone.hold_attitude(
            duration=5.0,
            throttle=1495
        )

        print("landing")
        drone.land()

        print("Mission complete. Flip AUX3 low to reset.")

    elif not mission_ready and aux3 <= AUX3_LOW:
        mission_ready = True
        drone.stop_all()

        print("Mission reset")

    time.sleep_ms(100)
