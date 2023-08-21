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

import logging
import time

import serial


class MarlinSerial:
    """
    Manages serial connection to Marlin-based controller board.
    """

    def __init__(self, vendor_id: str, product_id: str, idx: int, sim: bool = False) -> None:
        """
        Connects to Marlin-based controller board.
        :param vendor_id vendor id of device to connect to
        :param product_id product id of device to connect to
        :param idx device idx of device to connect to
        :param sim: Simulate serial connection if True.
        """
        self.log = logging.getLogger(__name__)
        self.vendor_id = vendor_id
        self.product_id = product_id
        self.idx = idx
        self.sim = sim
        self.ser = None
        self.open()

    def clear(self) -> None:
        """
        Clears serial input buffer.
        :return: None
        """
        if self.sim:
            self.log.info('Clearing serial buffer.')
        else:
            self.ser.flush()
            self.ser.reset_input_buffer()

    def read(self) -> bytes:
        """
        Reads a line from serial interface.
        :return: Message
        """
        if self.sim:
            time.sleep(0.5)
            if self.last_cmd == 'M114':  #  get current position
                self.last_cmd = None
                return b'X:0.00 Y:127.00 Z:145.00 E:0.00 Count X: 0 Y:10160 Z:116000\n'
            return b'ok\n'
        else:
            msg = self.ser.readline()
            self.log.debug('Read from serial port: {:s}'.format(str(msg)))
            return msg

    def reconnect(self):
        self.close()
        time.sleep(1)
        self.open()

    def open(self):
        if self.ser:
            self.close()
        if not self.sim:
            from emfi_station.utils import get_device_fd
            tty = get_device_fd(vendor=self.vendor_id, product=self.product_id, subsystem="tty", idx=self.idx)
            self.log.info(f"Connecting to TTY {tty}")
            self.ser = serial.Serial(port=tty, baudrate=115200, timeout=0.25)
            self.clear()

    def close(self) -> None:
        """
        Closes serial port.
        :return: None
        """
        self.log.info('Closing serial port.')
        if self.ser:
            self.ser.close()
            self.ser = None

    def cmd(self, cmd: str, retriable: bool = True) -> None:
        """
        Sends command via serial.
        :param cmd: Command string (e.g.: 'M122')
        :param retriable: whether this command can be retried after a connection reset or not.
                SAFETY: set this to false for move commands, catch SerialException and PortNotOpenError
                        and possibly home in exception handler
        :return: None
        """
        self.last_cmd = cmd
        if self.sim:
            self.log.info('Sending: {:s}'.format(str(cmd)))
        else:
            self.log.debug('Write to serial port: {:s}'.format(str(cmd)))
            while True:
                from serial import SerialException
                from serial import PortNotOpenError
                try:
                    self.ser.write((cmd + '\n').encode())
                    self.ser.flush()
                    break
                except SerialException or PortNotOpenError as e:
                    self.reconnect()
                    if not retriable:
                        raise e
