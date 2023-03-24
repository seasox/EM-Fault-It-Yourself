import time
from dataclasses import dataclass, field
from enum import Enum

from bitstring import BitArray

from Comm import Comm, Register
import numpy as np
from chipshouter import ChipSHOUTER
from emfi_station import Attack
from typing import Tuple, Dict, List, Any


class BitFlip(Enum):
    ZERO_TO_ZERO = 0
    ONE_TO_ONE = 1
    ZERO_TO_ONE = 2
    ONE_TO_ZERO = 3


class Metric(Enum):
    AnyFlipAnywhere = 1


@dataclass
class Datapoint:
    regs_pre: Dict[str, Register]
    regs_post: Dict[str, Register]

    attack_location: Tuple[int, int, int]
    reg_diff: Dict = field(init=False)

    def __post_init__(self):
        self.__keys = set(self.regs_pre.keys()).intersection(self.regs_post.keys())
        self.reg_diff: Dict[str, List[BitFlip]] = self.__calc_reg_diff()

    @staticmethod
    def _diff(pre: Register, post: Register) -> list[BitFlip]:
        flips = []
        for a, b in zip(pre.data.bin, post.data.bin):
            if a == b == 0:
                flips.append(BitFlip.ZERO_TO_ZERO)
            elif a == b == 1:
                flips.append(BitFlip.ONE_TO_ONE)
            elif a == 0 and b == 1:
                flips.append(BitFlip.ZERO_TO_ONE)
            else:
                flips.append(BitFlip.ONE_TO_ZERO)
        return flips

    def __calc_reg_diff(self) -> Dict[str, List[BitFlip]]:
        data = {}
        for reg_name in self.__keys:
            pre = self.regs_pre[reg_name]
            post = self.regs_post[reg_name]
            if pre.data is not None and post.data is not None:
                data[reg_name] = self._diff(pre, post)
        return data

    def evaluate(self, m: Metric) -> Any:
        """
        Returns how well this datapoint performed given some metric
        """
        match m:
            case Metric.AnyFlipAnywhere:
                return sum([self.reg_diff[reg_name].count(BitFlip.ZERO_TO_ONE) +
                            self.reg_diff[reg_name].count(BitFlip.ONE_TO_ZERO)
                            for reg_name in self.reg_diff])
            case _:
                return 0

class Probing(Attack):
    cs: ChipSHOUTER

    def __init__(self):
        super().__init__(start_pos=(0, 48, 115),
                         end_pos=(14, 63, 115),
                         step_size=1,
                         max_target_temp=40,
                         cooling=1,
                         repetitions=3)
        self.metric = Metric.AnyFlipAnywhere
        self.prev_reg = None
        self.aw = None
        self.dp_matrix_loc = None
        self.device = Comm(miso_pin=21,
                           clk_pin=23,
                           regs=8,
                           reg_size=4,
                           end_sequence=BitArray(bin=f"1{'0' * 62}1"))
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
        regs = self.device.read_regs()
        d = Datapoint(self.prev_reg, regs, self.aw.position)
        x, y, z = (np.array(self.aw.position) - self.start_pos) // self.step_size
        performance = d.evaluate(self.metric)
        self.dp_matrix_loc[x][y][z] = performance

        match self.metric:
            case Metric.AnyFlipAnywhere:
                success = performance > 0
            case _:
                success = False

        if success:
            self.aw.a_log.log(str(d.reg_diff))

        return success

    def reset_target(self) -> None:
        self.device.open_ocd.reset("run")
        time.sleep(1)
        # make sure device is running!
        self.prev_reg = self.device.read_regs()

    def critical_check(self) -> bool:
        return True

    def shutdown(self) -> None:
        self.cs.armed = 0
        self.device.open_ocd.halt()
        self.device.open_ocd.shutdown()