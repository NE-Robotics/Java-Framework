# This is a NetworkTables client (eg, the DriverStation/coprocessor side).
# You need to tell it the IP address of the NetworkTables server (the
# robot or simulator).
#
# When running, this will continue incrementing the value 'dsTime', and the
# value should be visible to other networktables clients and the robot.
#

import argparse
from os.path import basename
import logging
import time
import psutil
import random

import base64
import ntcore


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-p",
        "--protocol",
        type=int,
        choices=[3, 4],
        help="NT Protocol to use",
        default=4,
    )
    parser.add_argument("-ip", type=str, help="IP address to connect to")
    args = parser.parse_args()

    print(args.ip)

    inst = ntcore.NetworkTableInstance.getDefault()

    identity = basename(__file__)
    if args.protocol == 3:
        inst.startClient3(identity)
    else:
        inst.startClient4(identity)

    inst.setServer(args.ip)

    sd = inst.getTable("SmartDashboard")
    ml = inst.getTable("ML Detector")

    t_s = time.time()
    while True:
        print("FPGA Time:", sd.getNumber("robotTime", -1))

        ml.putNumberArray('Detections Array', [random.randint(0,1000), random.randint(0,1000), random.randint(0,1000), random.randint(0,1000)])

        ml.putNumber("Client Time", time.time() - t_s)
        ml.putNumber("CPU Usage", psutil.cpu_percent())
        ml.putNumber("RAM Usage", psutil.virtual_memory().percent)
        time.sleep(0.1)