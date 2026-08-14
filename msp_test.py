from msp import MSP
import time

msp = MSP()

print("Testing MSP communication...")
print()

while True:
    rc = msp.get_rc_channels()

    if rc is None:
        print("FAIL: No MSP response from flight controller")
    else:
        print("SUCCESS: MSP communication working both ways")
        print("RC channels:", rc)

        if len(rc) >= 8:
            print(
                "Roll:", rc[0],
                "Pitch:", rc[1],
                "Yaw:", rc[2],
                "Throttle:", rc[3],
                "AUX1:", rc[4],
                "AUX2:", rc[5],
                "AUX3:", rc[6],
                "AUX4:", rc[7],
            )

    print()
    time.sleep(1)
