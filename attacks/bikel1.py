import json
import time
from datetime import datetime
from pathlib import Path

from RPi import GPIO
from bitstring import BitArray
from chipshouter import ChipSHOUTER

from Comm import Response, ResetRelay, Comm, STATUS
from emfi_station import Attack
from emfi_station.utils import add_tuples

# chip dimensions: 11x11 mm
#stm32f4_offset = (7, 0, 0)
#stm32f4_start = (105, 60, 80.5)
#stm32f4_start = add_tuples(stm32f4_start, stm32f4_offset)
#stm32f4_end = stm32f4_start
#repetitions = 2 ** 31

# chip dimensions: 11x11 mm
device_name = "stm32f4discovery"
stm32f4_delta = (11, 11, 0)
stm32f4_start = (105, 60, 80.5)
#stm32f4_start_offset = (7, 0, 0)
stm32f4_start_offset = (0, 0, 0)
#stm32f4_end_offset = (0, -8, 0)
stm32f4_end_offset = (0, 0, 0)

stm32f4_end = add_tuples(stm32f4_start, stm32f4_delta)
stm32f4_start = add_tuples(stm32f4_start, stm32f4_start_offset)
stm32f4_end = add_tuples(stm32f4_end, stm32f4_end_offset)

repetitions = 20

HAMMING_WEIGHT_MAX_THRESH = 95
HAMMING_WEIGHT_MIN_THRESH = 30

BIKE_H0_LEN_BYTE = 1541
BIKE_H0_LEN_BIT = BIKE_H0_LEN_BYTE * 8
BIKE_H0_PADDED = 2048
BIKE_H0_PADDED_BIT = BIKE_H0_PADDED * 8

NUM_SHOUTS = 3

class BikeL1(Attack):
    cs: ChipSHOUTER

    def __init__(self):
        super().__init__(start_pos=stm32f4_start,
                         end_pos=stm32f4_end,
                         max_target_temp=60,
                         cooling=0.5,
                         repetitions=repetitions)
        self.aw = None

        # Parameters based on behavior of Device firmware, using BCM pins
        self.reg_size = 4  # in bytes
        self.regs = ["r4", "r5"]
        self.miso_pin = 9
        self.mosi_pin = 10
        self.clk_pin = 11
        self.reset_pin = 0
        self.dut_prep_time = .01  # in seconds

        # the experiments to persist to disk
        self.experiments = []

        # the hamming weight of the target registers
        self.hw_h0 = 0
        self.sk = BitArray()
        self.num_shouts = NUM_SHOUTS

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

        self.cs.voltage = 150
        self.cs.pulse.repeat = 3
        self.cs.arm_timeout = 10  # 10 minutes arm timeout
        self.experiments = []
        GPIO.output(self.mosi_pin, 1)  # set control pin high (otherwise no fault window is started)
        
    def cs_reconnect(self):
        self.cs.disconnect()
        time.sleep(1)
        self.cs.connect()
        time.sleep(1)
        self.log.info(
            f"ChipSHOUTER settings: {self.cs.voltage.set} V/{self.cs.voltage.measured} V, {self.cs.pulse.repeat} pulses, armed: {self.cs.armed}")
        
        
    def do_shout(self):
        done = False
        while not done:
            try:
                if not self.cs.armed:
                    self.cs.armed = True
                    time.sleep(.5)
                self.cs.pulse = True
                done = True
            except Exception as e:
                self.log.error(e)
                self.cs_reconnect()
        
    def wait_fault_window_end(self) -> bool:
        time_taken = self.device.wait_fault_window_end()
        self.log.debug("Waiting for fault window end took %.02f seconds" % (time_taken))

        if time_taken < 0:  # a timeout, we cannot say anything about the device's state!
            self.log.info('Did not find fault window end sequence, resetting')
            self.experiments[-1]["response_after_fault"] = [STATUS.FAULT_WINDOW_TIMEOUT]
        return time_taken > 0

        
        
    def read_sk_part(self) -> bool:
        # we know the device sent the fault window end sequence
        time.sleep(self.dut_prep_time)  # wait for DUT to arrive at transfer()
        response_after_fault = self.device.read_regs()  # read the register values
        # This is necessary in bike attack, make sure to only use raw from here on
        response_after_fault.raw.byteswap(4)
        self.log.debug(f"Read register values: {response_after_fault.raw.hex}")

        time.sleep(self.dut_prep_time)  # wait for DUT to arrive at transfer()
        _data = self.device.read(self.end_seq.len // 8)  # read end sequence
        if _data != self.end_seq:  # make sure the end sequence was received
            self.log.info('Did not find end sequence, resetting')
            self.experiments[-1]['response_after_fault'] = [STATUS.END_SEQUENCE_NOT_FOUND]
            return False

        if len(self.sk) <= BIKE_H0_LEN_BIT:
            self.hw_h0 += response_after_fault.raw.bin.count("1")

        self.sk.append(response_after_fault.raw)  # TODO check order
        return True

    def shout(self) -> bool:
        while len(self.sk) < 2 * BIKE_H0_PADDED_BIT:
            self.log.debug("Waiting for start sequence...")
            time_taken = self.device.wait_fault_window_start()
            if time_taken < 0:
                self.log.info("Did not find start sequence, resetting")
                self.experiments[-1]['response_before_fault'] = [STATUS.RESET_UNSUCCESSFUL]
                return False
            self.log.debug("Found start sequence in %.02f seconds" % (time_taken))
            
            if self.num_shouts > 0:
                self.do_shout()
                self.num_shouts -= 1
            else:
                GPIO.output(self.mosi_pin, 0)  # set control pin low, now the DUT does not wait for fault
            
            success = self.wait_fault_window_end()
            if not success:
                return False
            success = self.read_sk_part()
            if not success:
                return False
        
        return True

    def was_successful(self) -> bool:
        assert(len(self.sk) == 2 * BIKE_H0_PADDED_BIT)
        self.log.info(
            f'Hamming weight of H0: {self.hw_h0}, Key length: {len(self.sk)}, {"%.02f" % (len(self.sk) / (2 * BIKE_H0_PADDED_BIT))} done')

        is_faulty = self.hw_h0 > HAMMING_WEIGHT_MAX_THRESH or self.hw_h0 < HAMMING_WEIGHT_MIN_THRESH
        
        # log success
        if self.hw_h0 > HAMMING_WEIGHT_MAX_THRESH:
            self.log.critical('Generated faulty H0 key (too many ones)!')
        if self.hw_h0 < HAMMING_WEIGHT_MIN_THRESH:
            self.log.critical('Generated faulty H0 key (too few ones)!')

        # read full H0, H1 key
        time.sleep(2)  # really make sure DUT arrives at transfer()
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
            "is_faulty": is_faulty,
            "hw_h0": self.hw_h0,
            "register_key": self.sk.bin,
            "stack_key": _data.bin,
            "position": self.aw.position
        }

        self.experiments[-1].update(result_dict)
        
        self.log.info(self.experiments[-1])

        if not is_faulty:
            self.log.info('Finished this iteration without a faulty key :(')
        else:
            if transmission_correct:
                self.log.info('Finished this iteration with faulty key GG ez 1.0')
                return True
            else:
                self.log.info('Finished this iteration with faulty key but transmission was not correct :(')

        return False

    def reset_target(self):
        self.log.info("Resetting target")
        self.num_shouts = NUM_SHOUTS
        self.hw_h0 = 0
        self.sk = BitArray()
        self.experiments.append({})
        GPIO.output(self.mosi_pin, 1)
        self.reset.reset()

    def critical_check(self) -> bool:
        return True

    def shutdown(self) -> None:
        self.cs.armed = 0
        self.reset.reset()
        path = Path('dp_json', f'bikel1_{datetime.now().isoformat()}.json')  # save the experiments to disk
        with path.open('w') as f:
            j = {
                "experiments": self.experiments,
                "settings": {
                    "cs": {
                        "voltage": self.cs.voltage.set,
                        "pulse_repeat": self.cs.pulse.repeat,
                    },
                    "experiment": {
                        "start": self.start_pos,
                        "end": self.end_pos,
                        "device_name": device_name,
                        "start_offset": stm32f4_start_offset,
                        "end_offset": stm32f4_end_offset,
                        "repetitions": repetitions,
                    }
                }
            }
            json.dump(j, f)
        self.log.info("End...")


def compare_with_stack_key(register_key: BitArray, stack_key: BitArray) -> bool:
    # FIXME when faulting, stack key contains garbage (that repeats in a pattern). h0_register contains the correct key, thus we can ignore it for now.
    # FIXME Maybe we do not clean up correctly in delay some time.
    h0_stack = stack_key[:BIKE_H0_LEN_BIT]
    h0_register = register_key[:BIKE_H0_LEN_BIT]
    h0_register.byteswap(4)
    return h0_register.bin == h0_stack.bin


if __name__ == '__main__':
    root = Path(__file__).parent.parent
    with open(root.joinpath("sk_2024-01-29T12:38:03.899731.json"), "r") as fh:
        d = json.load(fh)
        equal = compare_with_stack_key(BitArray(bin=d["register_key"]), BitArray(bin=d["stack_key"]))
        print(equal)
