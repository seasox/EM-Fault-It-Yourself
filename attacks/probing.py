import time
from dataclasses import dataclass, field
from enum import Enum
import pickle
from bitstring import BitArray

from Comm import Comm, Register, Response, STATUS

import numpy as np
from chipshouter import ChipSHOUTER
from emfi_station import Attack
from typing import Tuple, Dict, List


class BitFlip(Enum):
    ZERO_TO_ZERO = 0
    ONE_TO_ONE = 1
    ZERO_TO_ONE = 2
    ONE_TO_ZERO = 3

    def __str__(self):
        return self.name

    def __repr__(self):
        return self.name


class Metric(Enum):
    AnyFlipAnywhere = 1

    def __str__(self):
        return self.name

    def __repr__(self):
        return self.name


@dataclass
class Datapoint:
    response_before_fault: Response
    response_after_fault: Response
    attack_location: Tuple[int, int, int]
    reg_diff: Dict[str, List[BitFlip]] | None = field(init=False)
    __regs_before_fault: Dict[str, Register] | None = field(init=False)
    __regs_after_fault: Dict[str, Register] | None = field(init=False)

    def __post_init__(self):
        self.__regs_before_fault = self.response_before_fault.reg_data
        self.__regs_after_fault = self.response_after_fault.reg_data
        if self.__regs_before_fault is not None and self.__regs_after_fault is not None:
            self.__keys = set(self.__regs_before_fault.keys()).intersection(self.__regs_after_fault.keys())
            self.reg_diff: Dict[str, List[BitFlip]] = self.__calc_reg_diff()
        else:
            self.__keys = None
            self.reg_diff = None

    @staticmethod
    def _diff(pre: Register, post: Register) -> list[BitFlip]:
        flips = []
        for a, b in zip(pre.data.bin, post.data.bin):
            if a == b == "0":
                flips.append(BitFlip.ZERO_TO_ZERO)
            elif a == b == "1":
                flips.append(BitFlip.ONE_TO_ONE)
            elif a == "0" and b == "1":
                flips.append(BitFlip.ZERO_TO_ONE)
            else:
                flips.append(BitFlip.ONE_TO_ZERO)
        return flips

    def __calc_reg_diff(self) -> Dict[str, List[BitFlip]]:
        data = {}
        for reg_name in self.__keys:
            pre = self.__regs_before_fault[reg_name]
            post = self.__regs_after_fault[reg_name]
            if pre.data is not None and post.data is not None:
                data[reg_name] = self._diff(pre, post)
        return data

    def get_01_flips(self, reg_name):
        return self.reg_diff[reg_name].count(BitFlip.ZERO_TO_ONE)

    def get_10_flips(self, reg_name):
        return self.reg_diff[reg_name].count(BitFlip.ONE_TO_ZERO)

    def __repr__(self):
        s = ""
        for reg_name in self.__keys:
            s += f"{reg_name} | 0 -> 1: {self.get_01_flips(reg_name)} | 1 -> 0: {self.get_10_flips(reg_name)} \n"
        return s

    def evaluate(self, m: Metric) -> float:
        """
        Returns how well this datapoint performed given some metric. Either a positive value or -1 if unsuccessful
        """
        # if sanity check was not successful no metric performs well..
        if STATUS.RESET_UNSUCCESSFUL in self.response_before_fault.status :
            return -1

        match m:
            case Metric.AnyFlipAnywhere:
                if STATUS.FAULT_WINDOW_TIMEOUT in self.response_after_fault.status\
                        or STATUS.END_SEQUENCE_NOT_FOUND in self.response_after_fault.status:
                    return -1 # a timeout or undefined behavior after fault is not what we expect here
                return sum([self.get_01_flips(reg_name) + self.get_10_flips(reg_name) for reg_name in self.__keys])
            case _:
                raise LookupError(f"Metric {str(m)} not covered")


class Probing(Attack):
    cs: ChipSHOUTER

    def __init__(self):
        super().__init__(start_pos=(95, 68, 86),
                         end_pos=(105, 78, 86),
                         step_size=0.1,
                         max_target_temp=40,
                         cooling=1,
                         repetitions=1)
        self.metric = Metric.AnyFlipAnywhere
        self.response_before_fault = None
        self.response_after_fault = None
        self.aw = None

        # Parameters based on behavior of Device firmware, using BCM pins
        self.reg_size = 4  # in bytes
        self.regs = 8
        self.miso_pin = 9
        self.clk_pin = 11
        self.reset_pin = 0
        self.dut_prep_time = .001
        self.bus_id = "001"
        self.device_id = "020"

        # The end sequence acts like a "checksum", if it is not transferred correctly, we cant be sure about the results
        self.end_seq = BitArray(bytes=b"\x42\x42\x42\x42")
        self.fault_window_start_seq = BitArray(bytes=b"\x2a\x2a\x2a\x2a")
        self.fault_window_end_seq = BitArray(bytes=b"\x13\x37\x13\x37")
        # each register sends a number that distributes 0/1 evenly. r7 is used as a round counter
        self.expected_data = [0xaaaaaaaa,
                              0xaaaaaaaa,
                              0xaaaaaaaa,
                              0xaaaaaaaa,
                              0xaaaaaaaa,
                              0xaaaaaaaa,
                              0xaaaaaaaa,
                              0]
        self.device = Comm(miso_pin=self.miso_pin,
                           clk_pin=self.clk_pin,
                           reset_pin=self.reset_pin,
                           regs=self.regs,
                           reg_size=self.reg_size,
                           fault_window_start_seq=self.fault_window_start_seq,
                           fault_window_end_seq=self.fault_window_end_seq,
                           reg_data_expected=self.expected_data)

        # Other parameters, watch out that dz is 0 if only one layer is attacked!
        self.dx, self.dy, self.dz = ((np.array(self.end_pos) - self.start_pos) / self.step_size)
        self.max_reset_tries = 3

        # The datapoints of a given x,y,z location
        self.dps: List[List[List[List[Datapoint]]]] = self.__init_dp_matrix()

    @staticmethod
    def name() -> str:
        return "Probing Attack"

    def __init_dp_matrix(self):
        # this order is required to access the matrix using [x][y][z]
        import math
        return [[[list() for _ in range(math.ceil(self.dz + 1))] \
                 for _ in range(math.ceil(self.dy + 1))] \
                for _ in range(math.ceil(self.dx + 1))]

    def init(self, aw) -> None:
        self.aw = aw
        self.cs = ChipSHOUTER("/dev/ttyUSB0")
        self.cs.voltage = 500
        self.cs.pulse.repeat = 10

    def shout(self) -> None:
        self.device.wait_fault_window_start()
        while True:
            try:
                if not self.cs.armed:
                    self.cs.armed = True
                    time.sleep(.5)
                self.cs.pulse = True
            except Exception as e:
                self.log.error(e)
                continue
            return

    def was_successful(self) -> bool:
        time_taken = self.device.wait_fault_window_end()
        print(f"Waiting for end sequence took {time_taken} seconds")
        if time_taken < 0: # a timeout
            self.response_after_fault = Response({STATUS.FAULT_WINDOW_TIMEOUT}, None, None)
        else: # we received the end_sequence
            # wait and read regs
            time.sleep(self.dut_prep_time)
            self.response_after_fault = self.device.read_regs()
            # wait and read end sequence
            time.sleep(self.dut_prep_time)
            _data = self.device.read(self.end_seq.len // 8)
            if _data != self.end_seq:
                self.response_after_fault.status.add(STATUS.END_SEQUENCE_NOT_FOUND)

        x, y, z = np.ceil((np.array(self.aw.position) - self.start_pos) / self.step_size).astype(dtype=int)
        d = Datapoint(self.response_before_fault, self.response_after_fault, (x, y, z))
        performance = d.evaluate(self.metric)
        self.dps[x][y][z].append(d)

        print(f"Performance: {performance}, Status[before]: {self.response_before_fault.status}, Status[after]: {self.response_after_fault.status}")

        if performance > 0:
            print(f"The following registers are faulted: {[reg_name for reg_name in self.response_after_fault.reg_data if self.response_after_fault.reg_data[reg_name].corrupted]}")

        match self.metric:
            case Metric.AnyFlipAnywhere:
                success = performance > 0
            case _:
                raise LookupError(f"Metric {self.metric} not covered")

        print("-" * 100)
        return success

    def reset_target(self) -> None:
        self.device.reset()
        self.response_before_fault = self.device.read_regs()
        if STATUS.EXPECTED_DATA_MISMATCH in self.response_before_fault.status:
            print("Reset unsuccessful...")


    def critical_check(self) -> bool:
        return True

    def shutdown(self) -> None:
        from datetime import datetime
        timestamp = datetime.now().strftime("%Y-%m-%d-%H:%M:%S")
        storage_fp = open(f"data_{timestamp}.pickle", "wb")  # where we store the data later
        # evaluate the data:
        pickle.dump(self.dps, storage_fp)
        storage_fp.close()
        self.cs.armed = 0
        self.device.reset()
        print("End...")

if __name__ == '__main__':
    Probing()