# EMFI Station - Orchestrate electromagnetic fault injection attacks
# Copyright (C) 2022 Niclas Kühnapfel
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
import ast
import sys
import logging
import argparse
from pprint import pprint
from typing import Optional, List, Tuple

from .config import Config
from .emfi_station import EMFIStation


def main():
    def device_info(param):
        if len(param) == 2:
            return param[0], param[1], 0
        if len(param) == 3:
            return param[0], param[1], int(param[2])
        raise ValueError(f'"{" ".join(param)}" is invalid device information')

    parser = argparse.ArgumentParser(description='EMFI Station - Orchestrate electromagnetic fault injection attacks')
    parser.add_argument('-i', '--host', type=str, default='0.0.0.0', help='Hostname or ip address')
    parser.add_argument('-p', '--port', type=int, default='8080', help='The http port, websocket port = http port + 1)')
    parser.add_argument('-s', '--simulate', action='store_true', default=False, help='Simulate connection to Marlin')
    parser.add_argument('-a', '--attack_dir', type=str, default='attacks', help='Attack scripts directory')
    parser.add_argument('-l', '--log_dir', type=str, default='log', help='Log files directory')
    # Devices:
    parser.add_argument('--marlin', type=str, nargs='*', default=('0483', '5740', 0),
                        help='The device information of marlin board. Format: vendor_id product_id [index]')
    parser.add_argument('--cal_cam', type=str, nargs='*', default=('0304', 'a16f', 0),
                        help='The device information of calibration camera. Format: vendor_id product_id [index]')
    parser.add_argument('--pos_cam', type=str, nargs='*', default=('299f', 'eb1a', 0),
                        help='The device information of positioning camera. Format: vendor_id product_id [index]')
    parser.add_argument('--world_cam', type=str, nargs='*', default=('0779', '045e', 0),
                        help='The device information of world camera. Format: vendor_id product_id [index]')  # ('0304', 'a16f', 0)
    # Misc:
    parser.add_argument('--safe_z', type=float, default=86.5,
                        help='Prevent the stage moving beyond the specified value regardless of the attack parameters')
    parser.add_argument('-v', '--verbosity', default=False, action='store_true', help='Enable info log level')
    args = parser.parse_args()

    config = Config(host=args.host,
                    http_port=args.port,
                    simulate=args.simulate,
                    attack_dir=args.attack_dir,
                    log_dir=args.log_dir,
                    marlin=device_info(args.marlin),
                    calibration_cam=device_info(args.cal_cam),
                    positioning_cam=device_info(args.pos_cam),
                    world_cam=device_info(args.world_cam),
                    safe_z=args.safe_z)

    pprint(config)
    level = logging.DEBUG if args.verbosity else logging.INFO
    logging.basicConfig(format='%(levelname)s:%(asctime)s:%(filename)s:%(message)s', stream=sys.stdout, level=level)
    # protocol.py logs all websocket messages as DEBUG, which we usually don't need
    logging.getLogger('websockets.server').setLevel(logging.INFO)

    EMFIStation(config)


if __name__ == '__main__':
    main()
