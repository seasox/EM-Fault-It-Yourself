import time
from dataclasses import dataclass, field
from enum import Enum
from pprint import pprint

from bitstring import BitArray

from Comm import Comm, Register, STATUS, Response
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
        keys = set(self.__regs_before_fault.keys()).intersection(self.__regs_after_fault.keys())
        for reg_name in keys:
            pre = self.__regs_before_fault[reg_name]
            post = self.__regs_after_fault[reg_name]
            if pre.data is not None and post.data is not None:
                data[reg_name] = self._diff(pre, post)
        return data

    def evaluate(self, m: Metric) -> float:
        """
        Returns how well this datapoint performed given some metric. Either a positive value or -1 if unsuccessful
        """

        if STATUS.RESET_UNSUCCESSFUL in self.response_before_fault.status:
            return -1

        match m:
            case Metric.AnyFlipAnywhere:
                return sum([self.reg_diff[reg_name].count(BitFlip.ZERO_TO_ONE) +
                            self.reg_diff[reg_name].count(BitFlip.ONE_TO_ZERO)
                            for reg_name in self.reg_diff])
            case _:
                raise LookupError(f"Metric {str(m)} not covered")

class Probing(Attack):
    cs: ChipSHOUTER

    def __init__(self):
        super().__init__(start_pos=(5, 63, 115),
                         end_pos=(14, 73, 115),
                         step_size=1,
                         max_target_temp=40,
                         cooling=1,
                         repetitions=3)
        self.metric = Metric.AnyFlipAnywhere
        self.response_before_fault = None
        self.response_after_fault = None
        self.aw = None

        # Parameters based on behavior of Device firmware, using BCM pins
        self.reg_size = 4 # in bytes
        self.regs = 8
        self.miso_pin = 9
        self.clk_pin = 11
        self.reset_pin = 0

        self.end_sequence = BitArray(bin=f"1{'0' * 62}1")
        self.expected_data = [0,1,2,3,4,5,6,7]

        self.device = Comm(miso_pin=self.miso_pin,
                           clk_pin=self.clk_pin,
                           reset_pin=self.reset_pin,
                           regs=self.regs,
                           reg_size=self.reg_size,
                           end_sequence=self.end_sequence,
                           expected_data=self.expected_data)

        self.dp_matrix_loc = self.__init_dp_matrix()

    @staticmethod
    def name() -> str:
        return "Probing Attack"

    def __init_dp_matrix(self):
        dx, dy, dz = (np.array(self.end_pos) - self.start_pos) // self.step_size  # how many dps in each dim
        return np.zeros((dx + 1, dy + 1, dz + 1))

    def init(self, aw) -> None:
        self.aw = aw
        self.cs = ChipSHOUTER("/dev/ttyUSB0")
        self.cs.voltage = 500
        self.cs.pulse.repeat = 9

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

    def visualize(self):
        import matplotlib.pyplot as plt
        plt.imshow(self.dp_matrix_loc[:, :, 0], cmap='viridis', interpolation='nearest')
        plt.colorbar()
        plt.show()

    def was_successful(self) -> bool:
        self.response_after_fault = self.device.read_regs()
        x, y, z = (np.array(self.aw.position) - self.start_pos) // self.step_size

        d = Datapoint(self.response_before_fault, self.response_after_fault, self.aw.position)
        performance = d.evaluate(self.metric)
        self.dp_matrix_loc[x][y][z] = performance

        match self.metric:
            case Metric.AnyFlipAnywhere:
                success = performance > 0
            case _:
                raise LookupError(f"Metric {str(self.metric)} not covered")

        if success:
            self.aw.a_log.log(str(d.reg_diff))
            print(d)
        return success

    def reset_target(self) -> None:
        self.device.reset()
        self.response_before_fault = self.device.read_regs()

        # this is actually not good, as we just reset:
        if STATUS.END_SEQUENCE_FOUND not in self.response_before_fault.status\
                or STATUS.EXPECTED_DATA_MISSMATCH in self.response_before_fault.status:
            print(f"Reset failed! Status: {self.response_before_fault.status} | Buffer: {str(self.response_before_fault.raw)}")
            self.response_before_fault.status.add(STATUS.RESET_UNSUCCESSFUL)

    def critical_check(self) -> bool:
        return True

    def shutdown(self) -> None:
        self.cs.armed = 0
        self.device.reset()