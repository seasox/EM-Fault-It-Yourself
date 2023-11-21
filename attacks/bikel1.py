import time

import numpy as np
from RPi import GPIO
from chipshouter import ChipSHOUTER

from Comm import Response, ResetRelay
from emfi_station import Attack
from emfi_station.utils import add_tuples

# import pyserial

# chip dimensions: 12x12 mm
stm32f4_delta = (12, 12, 0)
stm32f4_start = (99, 60, 80)
stm32f4_start_offset = (8, 0, 0)
stm32f4_end_offset = (0, -2, 0)

stm32f4_end = add_tuples(stm32f4_start, stm32f4_delta)

stm32f4_start = add_tuples(stm32f4_start, stm32f4_start_offset)
stm32f4_end = add_tuples(stm32f4_end, stm32f4_end_offset)

repetitions = 1000


class BikeL1(Attack):
    cs: ChipSHOUTER
    response_before_fault: Response
    response_after_fault: Response

    def __init__(self):
        super().__init__(start_pos=stm32f4_start,
                         end_pos=stm32f4_end,
                         step_size=1,
                         max_target_temp=60,
                         cooling=0.5,
                         repetitions=repetitions)
        self.aw = None

        # Parameters based on behavior of Device firmware, using BCM pins
        self.miso_pin = 9
        self.reset_pin = 0
        self.dut_prep_time = .01  # in seconds

        GPIO.setmode(GPIO.BCM)  # This is needed as adafruit uses BCM Mode
        GPIO.setup(self.miso_pin, GPIO.IN)

        self.reset = ResetRelay(self.reset_pin)

        # self.serial = pyserial.Serial(port="/dev/TODO", baudrate=38400, timeout=None)
        self.serial = None

        # Other parameters, watch out that dz is 0 if only one layer is attacked!
        self.dx, self.dy, self.dz = ((np.array(self.end_pos) - self.start_pos) / self.step_size)

    @staticmethod
    def name() -> str:
        return f"BIKEL1 ({stm32f4_start} => {stm32f4_end}, {repetitions})"

    def init(self, aw) -> None:
        self.aw = aw
        self.cs = ChipSHOUTER("/dev/ttyUSB0")
        self.cs.voltage = 500
        self.cs.pulse.repeat = 5

    def shout(self) -> None:
        self.log.info("waiting for rising flank")
        while not GPIO.input(self.miso_pin):
            continue
        while True:
            try:
                if not self.cs.armed:
                    self.cs.armed = True
                    time.sleep(.5)
                self.cs.pulse = True
            except Exception as e:
                self.log.error(e)
                time.sleep(5)
                self.cs.disconnect()
                time.sleep(5)
                self.cs.connect()
                continue
            return

    def was_successful(self) -> bool:
        if not GPIO.input(self.miso_pin):
            self.log.error("missed fault window")
            return False
        self.log.info("waiting for falling flank")
        while GPIO.input(self.miso_pin):
            continue

    def reset_target(self) -> None:
        self.reset.reset()

    def critical_check(self) -> bool:
        return True

    def shutdown(self) -> None:
        # from datetime import datetime
        # timestamp = datetime.now().strftime("%Y-%m-%d-%H:%M:%S")
        # _dir = Path("pickles")
        # if not _dir.exists():
        #    os.makedirs(_dir)
        # filename = f"data_{timestamp}.pickle"
        # filepath = _dir.joinpath(filename)
        # fp = open(filepath, "wb")
        # evaluate the data:
        # self.log.debug(f'writing progress to pickle due to shutdown')
        # pickle.dump(self.dps, fp)
        # storage_fp.close()
        self.cs.armed = 0
        self.reset.reset()
        print("End...")


if __name__ == '__main__':
    BikeL1()
