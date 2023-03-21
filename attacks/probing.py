import time
from dataclasses import dataclass, field
from enum import Enum
from pprint import pprint
import numpy as np
from chipshouter import ChipSHOUTER
from STLinkComm import STLinkComm
from emfi_station import Attack
from typing import Tuple, Optional, Dict

class BitFlip(Enum):
    ZERO_TO_ZERO = 0
    ONE_TO_ONE = 1
    ZERO_TO_ONE = 2
    ONE_TO_ZERO = 3


class Metric(Enum):
    AnyFlipAnywhere = 1


@dataclass
class Datapoint():
    regs_pre_fault: dict
    regs_post_fault: dict
    attack_location: Tuple[int, int, int]
    config: dict
    mem_pre_fault: Optional[np.array]
    mem_post_fault: Optional[np.array]
    mem_diff: dict = field(init=False)
    reg_diff: dict = field(init=False)
    regs_flipped: dict = field(init=False)

    def __post_init__(self):
        self.mem_diff = self._calc_mem_diff()
        self.reg_diff = self._calc_reg_diff()
        self.regs_flipped = self._calc_regs_flipped()
        if not self.config.get("store_memory"):  # does not store memory dump by default
            self.mem_pre_fault = None
            self.mem_post_fault = None

    @staticmethod
    def _diff(pre, post) -> list[BitFlip]:
        flips = []
        for a, b in zip(pre, post):
            if a == b == 0:
                flips.append(BitFlip.ZERO_TO_ZERO)
            elif a == b == 1:
                flips.append(BitFlip.ONE_TO_ONE)
            elif a == 0 and b == 1:
                flips.append(BitFlip.ZERO_TO_ONE)
            else:
                flips.append(BitFlip.ONE_TO_ZERO)
        return flips

    def _calc_mem_diff(self):
        return {}

    def _calc_reg_diff(self) -> Dict:
        regs = set(self.regs_pre_fault.keys()).intersection(self.regs_post_fault.keys())
        data = {}
        for reg in regs:
            distr = self._diff(self.regs_pre_fault[reg].content,
                               self.regs_post_fault[reg].content)
            data[reg] = {
                "Distribution": distr,
                BitFlip.ZERO_TO_ZERO: distr.count(BitFlip.ZERO_TO_ZERO),
                BitFlip.ONE_TO_ONE: distr.count(BitFlip.ONE_TO_ONE),
                BitFlip.ZERO_TO_ONE: distr.count(BitFlip.ZERO_TO_ONE),
                BitFlip.ONE_TO_ZERO: distr.count(BitFlip.ONE_TO_ZERO),
            }

        return data

    def _calc_regs_flipped(self) -> Dict:
        data = {"total": 0, "0 -> 1": {}, "1 -> 0": {}}
        for reg, value in self.reg_diff.items():
            if value == "Corrupted":
                print(f"{reg} is corrupted!")
                continue
            zto = value[BitFlip.ZERO_TO_ONE]
            otz = value[BitFlip.ONE_TO_ZERO]
            data["total"] += zto + otz
            if zto:
                data["0 -> 1"][reg] = {"Count": zto,
                                       "Indices:": [i for i, x in enumerate(value["Distribution"]) if
                                                    x == BitFlip.ZERO_TO_ONE]}
            if otz:
                data["1 -> 0"][reg] = {"Count": otz,
                                       "Indices:": [i for i, x in enumerate(value["Distribution"]) if
                                                    x == BitFlip.ONE_TO_ZERO]}
        return data

    def evaluate(self, m: Metric):
        """
        Returns how well this datapoint performed given some metric
        """
        if m == Metric.AnyFlipAnywhere:
            return self.regs_flipped["total"]


class Probing(Attack):
    to_bits = lambda x, bit_width: np.unpackbits(
        np.frombuffer(x.to_bytes(bit_width, byteorder="big"), dtype=np.uint8), bitorder="big")[-bit_width:]
    cs: ChipSHOUTER

    def __init__(self):
        super().__init__(start_pos=(0, 48, 115),
                         end_pos=(14, 63, 115),
                         step_size=1,
                         max_target_temp=40,
                         cooling=1,
                         repetitions=3)
        self.device = STLinkComm(serial="0671FF3837334D4E43054345", value_cast=Probing.to_bits)
        self.reg_names = self.device.get_reg_names()
        print(self.reg_names)
        self.device.reset("halt")
        # TODO make dynamic
        self.metric = Metric.AnyFlipAnywhere
        self.prev_reg = None
        self.aw = None
        self.dp_matrix_loc = None

    @staticmethod
    def name() -> str:
        return "Probing Attack"

    def __init_dp_matrix(self):
        dx, dy, dz = (np.array(self.end_pos) - self.start_pos) // self.step_size  # how many dps in each dim
        return np.zeros((dx + 1, dy + 1, dz + 1))

    def init(self, aw) -> None:
        self.aw = aw
        self.dp_matrix_loc = self.__init_dp_matrix()
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
        plt.imshow(self.dp_matrix_loc, cmap='viridis', interpolation='nearest')
        plt.colorbar()
        plt.show()

    def was_successful(self) -> bool:
        regs = self.device.regs()
        d = Datapoint(self.prev_reg, regs, self.aw.position, {}, None, None)
        x, y, z = (np.array(self.aw.position) - self.start_pos) // self.step_size
        print(f"{(x, y, z)}; LR: {self.prev_reg['LR'].content} -> {regs['LR'].content}")
        performance = d.evaluate(self.metric)
        self.dp_matrix_loc[x][y][z] = performance
        success = False

        if self.metric == Metric.AnyFlipAnywhere:
            success = performance > 0

        if success:
            self.aw.a_log.log(str(d.regs_flipped))
            pprint(d.regs_flipped)

        return success

    def reset_target(self) -> None:
        self.device.reset('halt')
        time.sleep(1)
        self.prev_reg = self.device.regs()

    def critical_check(self) -> bool:
        return True

    def shutdown(self) -> None:
        self.cs.armed = 0
        self.device.close()


if __name__ == '__main__':
    Probing().init(None)