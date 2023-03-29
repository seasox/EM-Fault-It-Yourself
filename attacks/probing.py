import time
from dataclasses import dataclass, field
from enum import Enum
from pprint import pprint
import pickle
from bitstring import BitArray

from Comm import Comm, Register, Response, STATUS

import numpy as np
from chipshouter import ChipSHOUTER
from emfi_station import Attack
from typing import Tuple, Dict, List, Any


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
    reg_diff: Dict[str, List[BitFlip]] = field(init=False)
    __regs_before_fault: Dict[str, Register] = field(init=False)
    __regs_after_fault: Dict[str, Register] = field(init=False)

    def __post_init__(self):
        self.__regs_before_fault = self.response_before_fault.reg_data
        self.__regs_after_fault = self.response_after_fault.reg_data
        self.__keys = set(self.__regs_before_fault.keys()).intersection(self.__regs_after_fault.keys())
        self.reg_diff: Dict[str, List[BitFlip]] = self.__calc_reg_diff()

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
        if STATUS.RESET_UNSUCCESSFUL in self.response_before_fault.status:
            return -1

        match m:
            case Metric.AnyFlipAnywhere:
                return sum([self.get_01_flips(reg_name) + self.get_10_flips(reg_name) for reg_name in self.__keys])
            case _:
                raise LookupError(f"Metric {str(m)} not covered")


class Probing(Attack):
    cs: ChipSHOUTER

    def __init__(self):
        super().__init__(start_pos=(5, 63, 114),
                         end_pos=(14, 73, 114),
                         step_size=1,
                         max_target_temp=40,
                         cooling=1,
                         repetitions=3)
        self.metric = Metric.AnyFlipAnywhere
        self.response_before_fault = None
        self.response_after_fault = None
        self.aw = None

        # Parameters based on behavior of Device firmware, using BCM pins
        self.reg_size = 4  # in bytes
        self.regs = 8 # TODO change back to 8
        self.miso_pin = 9
        self.clk_pin = 11
        self.reset_pin = 0

        # The end sequence acts like a "checksum", if it is not transferred correctly, we cant be sure about the results
        self.end_sequence = BitArray(bytes=b"\x42\x42\x42\x42")
        self.expected_data = [i for i in range(self.regs)]  # each register sends its number

        # FIXME it appears only 16 bytes are transferred correctly (3 regs and end sequence)
        self.device = Comm(miso_pin=self.miso_pin,
                           clk_pin=self.clk_pin,
                           reset_pin=self.reset_pin,
                           regs=self.regs,
                           reg_size=self.reg_size,
                           end_sequence=self.end_sequence,
                           expected_data=self.expected_data)
        # Other parameters, watch out that dz is 0 if only one layer is attacked!
        self.dx, self.dy, self.dz = ((np.array(self.end_pos) - self.start_pos) // self.step_size)
        self.max_reset_tries = 3

        # The datapoints of a given x,y,z location
        self.dps: List[List[List[List[Datapoint]]]] = self.__init_dp_matrix()
        self.storage_fp = open("data.pickle", "wb")  # where we store the data later



    @staticmethod
    def name() -> str:
        return "Probing Attack"

    def __init_dp_matrix(self):
        # this order is required to access the matrix using [x][y][z]
        return [[[list() for _ in range(self.dz + 1)] for _ in range(self.dy + 1)] for _ in range(self.dx + 1)]

    def init(self, aw) -> None:
        self.aw = aw
        self.cs = ChipSHOUTER("/dev/ttyUSB0")
        self.cs.voltage = 500
        self.cs.pulse.repeat = 2

    def shout(self) -> None:
        while True:
            try:
                if not self.cs.armed:
                    self.cs.armed = True
                    time.sleep(1)
                self.cs.pulse = True
            except Exception as e:
                self.log(e)
                continue
            return

    def was_successful(self) -> bool:
        self.response_after_fault = self.device.read_regs()
        x, y, z = (np.array(self.aw.position) - self.start_pos) // self.step_size

        d = Datapoint(self.response_before_fault, self.response_after_fault, (x, y, z))
        performance = d.evaluate(self.metric)
        self.dps[x][y][z].append(d)

        print(
            f"Performance: {performance}, Status[before]: {self.response_before_fault.status}, Status[after]: {self.response_after_fault.status}")

        if STATUS.END_SEQUENCE_FOUND not in self.response_after_fault.status:
            print(f"The following registers are EITHER faulted or incorrectly transmitted: {[reg_name for reg_name in self.response_after_fault.reg_data if self.response_after_fault.reg_data[reg_name].corrupted]}")
            print(f"Received buffer after fault: {self.response_after_fault.raw}")


        match self.metric:
            case Metric.AnyFlipAnywhere:
                success = performance > 0
            case _:
                raise LookupError(f"Metric {self.metric} not covered")

        print("-" * 100)
        return success

    def reset_target(self) -> None:
        reset_cnt = 1
        while True:  # Python way of do ... while
            self.device.reset(reset_cnt)
            self.response_before_fault = self.device.read_regs()
            reset_cnt += 1
            success = STATUS.END_SEQUENCE_FOUND in self.response_before_fault.status \
                      and not STATUS.EXPECTED_DATA_MISMATCH in self.response_before_fault.status
            if success:
                break
            print("Resetting again...")
            if reset_cnt > self.max_reset_tries:
                self.response_before_fault.status.add(STATUS.RESET_UNSUCCESSFUL)
                break

    def critical_check(self) -> bool:
        return True

    def shutdown(self) -> None:
        # evaluate the data: TODO verify that this actually overrides this data
        pickle.dump(self.dps, self.storage_fp)
        self.storage_fp.flush()
        self.storage_fp.close()
        self.cs.armed = 0
        self.device.reset()
        print("End...")
