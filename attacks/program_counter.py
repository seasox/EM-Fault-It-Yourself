import time
import pickle

from RPi import GPIO
from bitstring import BitArray

from Comm import Comm, STATUS

import numpy as np
from chipshouter import ChipSHOUTER
from emfi_station import Attack

class ProgramCounter(Attack):

    cs: ChipSHOUTER

    def __init__(self):
        super().__init__(start_pos=(5, 63, 110),
                         end_pos=(14, 73, 110),
                         step_size=1,
                         max_target_temp=40,
                         cooling=1,
                         repetitions=3)
        self.response_before_fault = None
        self.response_after_fault = None
        self.aw = None

        # Parameters based on behavior of Device firmware, using BCM pins
        self.reg_size = 4  # in bytes
        self.regs = 8
        self.ready_pin = 9
        self.reset_pin = 0
        self.success_pin = 11

        self.__boot_up_time = 1.5
        self.__relay_time = .8

        # The end sequence acts like a "checksum", if it is not transferred correctly, we cant be sure about the results
        #self.fault_window_start_seq = BitArray(bytes=b"\x2a\x2a\x2a\x2a")
        #self.fault_window_end_seq = BitArray(bytes=b"\x13\x37\x13\x37")

        """self.device = Comm(miso_pin=self.miso_pin,
                           clk_pin=self.clk_pin,
                           reset_pin=self.reset_pin,
                           regs=[],
                           reg_size=[],
                           fault_window_start_seq=self.fault_window_start_seq,
                           fault_window_end_seq=self.fault_window_end_seq,
                           end_seq=BitArray())"""

        # Other parameters, watch out that dz is 0 if only one layer is attacked!
        self.dx, self.dy, self.dz = ((np.array(self.end_pos) - self.start_pos) // self.step_size)
        self.max_reset_tries = 5

    @staticmethod
    def name() -> str:
        return "Program Counter Attack"

    def init(self, aw) -> None:
        self.aw = aw
        self.cs = ChipSHOUTER("/dev/ttyUSB0")
        self.cs.voltage = 500
        self.cs.pulse.repeat = 2
        GPIO.setmode(GPIO.BCM)  # This is needed as adafruit uses BCM Mode
        GPIO.setup(self.ready_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.success_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.reset_pin, GPIO.OUT)
        GPIO.output(self.reset_pin, 1)

    def _read_attack_state(self):
        r, s = (GPIO.input(self.ready_pin), GPIO.input(self.success_pin))
        return r, s
    def shout(self) -> None:
        r, s = self._read_attack_state()
        while not r or s:
            print('not ready')
            self.log.warn(f'waiting for device to become ready: (r, s)={(r, s)}')
            r, s = self._read_attack_state()
        while True:
            try:
                if not self.cs.armed:
                    self.cs.armed = True
                    time.sleep(1)
                self.cs.pulse = True
            except Exception as e:
                self.log.error(e)
                continue
            return

    def was_successful(self) -> bool:
        time.sleep(0.1)
        r, s = self._read_attack_state()
        if r == s:
            print(f'r=s={s}')
        return r == 0 and s == 1

    def reset_target(self) -> None:
        # turn off device
        GPIO.output(self.reset_pin, 1)
        # wait until relay changes state
        time.sleep(self.__relay_time)
        GPIO.output(self.reset_pin, 0)
        # wait until relay changes state and device reboots
        time.sleep(self.__relay_time)
        time.sleep(self.__boot_up_time)
        r, s = self._read_attack_state()
        if s or not r:
            raise Exception('reset failed')

    def critical_check(self) -> bool:
        return True

    def shutdown(self) -> None:
        self.cs.armed = 0
        print("End...")

if __name__ == '__main__':
    ProgramCounter()