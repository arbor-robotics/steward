# Copyright (C) 2015-2021 Swift Navigation Inc.
# Contact: https://support.swiftnav.com
#
# This source is subject to the license found in the file 'LICENSE' which must
# be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
"""
the :mod:`sbp.client.examples.simple` module contains a basic example of
reading SBP messages from a serial port, decoding BASELINE_NED messages and
printing them out.
"""

from sbp.client.drivers.pyserial_driver import PySerialDriver
from sbp.client import Handler, Framer
from sbp.navigation import SBP_MSG_UTC_TIME, SBP_MSG_POS_ECEF, MsgPosLLHCov
from sbp.observation import MsgObs
from sbp.imu import MsgImuRaw
import argparse


def main():
    parser = argparse.ArgumentParser(description="Swift Navigation SBP Example.")
    parser.add_argument(
        "-p",
        "--port",
        default=["/dev/ttyUSB0"],
        nargs=1,
        help="specify the serial port to use.",
    )
    args = parser.parse_args()

    # Open a connection to Piksi using the default baud rate (1Mbaud)
    with PySerialDriver(args.port[0], baud=115200) as driver:
        with Handler(Framer(driver.read, None, verbose=True)) as source:
            try:
                for msg, metadata in source:
                    print(type(msg))

                    if isinstance(msg, MsgObs):
                        # print(msg)
                        pass
                    elif isinstance(msg, MsgImuRaw):
                        # print(msg)
                        pass
                    elif isinstance(msg, MsgPosLLHCov):
                        print(msg)
                    # Print out the N, E, D coordinates of the baseline
                    # print("%.4f,%.4f,%.4f" % (msg.n * 1e-3, msg.e * 1e-3, msg.d * 1e-3))
            except KeyboardInterrupt:
                pass


if __name__ == "__main__":
    main()
