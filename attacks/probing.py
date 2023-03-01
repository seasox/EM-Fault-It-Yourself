# EMFI Station - Orchestrate electromagnetic fault injection attacks
# Copyright (C) 2022 Niclas Kühnapfel
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import time
from dataclasses import dataclass, field
from enum import Enum
from pprint import pprint
from typing import Tuple

import numpy as np
from chipshouter import ChipSHOUTER
from emfi_station import Attack
from emfi_station.attack_worker import AttackWorker

import openocd


class BitFlip(Enum):
    ZERO_TO_ZERO = 0
    ONE_TO_ONE = 1
    ZERO_TO_ONE = 2
    ONE_TO_ZERO = 3

def diff(pre, post) -> list[BitFlip]:
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


@dataclass
class Datapoint:
    regs_pre_fault: dict[dict]
    regs_post_fault: dict[dict]
    attack_location: Tuple[int, int, int]
    config: dict
    mem_pre_fault: np.array
    mem_post_fault: np.array
    mem_diff: dict = field(init=False)
    reg_diff: dict = field(init=False)

    def __post_init__(self):
        self.mem_diff = self._calc_mem_diff()
        self.reg_diff = self._calc_reg_diff()
        if not self.config.get("store_memory"): # does not store memory dump by default
            del self.mem_pre_fault
            del self.mem_post_fault

    def _calc_mem_diff(self):
        return {}

    def _calc_reg_diff(self) -> dict:
        assert self.regs_pre_fault.keys() == self.regs_post_fault.keys()
        data = {}
        for key in self.regs_pre_fault.keys():
            distr = diff(self.regs_pre_fault[key]["Content"],
                         self.regs_post_fault[key]["Content"])
            data[key] = {
                "Distribution": distr,
                BitFlip.ZERO_TO_ZERO: distr.count(BitFlip.ZERO_TO_ZERO),
                BitFlip.ONE_TO_ONE: distr.count(BitFlip.ONE_TO_ONE),
                BitFlip.ZERO_TO_ONE: distr.count(BitFlip.ZERO_TO_ONE),
                BitFlip.ONE_TO_ZERO: distr.count(BitFlip.ONE_TO_ZERO),
            }

        return data

    def get_regs_flipped(self):
        data = {"0 -> 1": {}, "1 -> 0": {}}
        for key, value in self.reg_diff.items():
            zto = value[BitFlip.ZERO_TO_ONE]
            otz = value[BitFlip.ONE_TO_ZERO]
            if zto:
                data["0 -> 1"][key] = {"Count": zto, "Indices:": [i for i, x in enumerate(value["Distribution"]) if
                                                                  x == BitFlip.ZERO_TO_ONE]}
            if otz:
                data["1 -> 0"][key] = {"Count": otz, "Indices:": [i for i, x in enumerate(value["Distribution"]) if
                                                                  x == BitFlip.ONE_TO_ZERO]}
        return data

class Probing(Attack):
    to_bits = lambda x, bit_width: np.unpackbits(np.frombuffer(int(x, 16).to_bytes(bit_width, byteorder="big"), dtype=np.uint8),
                                                 bitorder="big")[-bit_width:]
    cs: ChipSHOUTER


    def __init__(self):
        super().__init__(start_pos=(2, 50, 110),
                         end_pos=(12, 60, 110),
                         step_size=1,
                         max_target_temp=40,
                         cooling=1,
                         repetitions=3)

        self.device = openocd.OpenOCD(value_cast=Probing.to_bits)
        self.device.connect()
        self.device.reset("halt")
        self.reg_pre_fault = None
        self.reg_post_fault = None

    @staticmethod
    def name() -> str:
        return "Probing Attack"

    def init(self) -> None:
        self.cs = ChipSHOUTER("/dev/ttyUSB0")
        self.cs.voltage = 500
        self.cs.pulse.repeat = 10

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

    def was_successful(self, aw: AttackWorker) -> bool:
        self.reg_post_fault = self.device.reg()
        d = Datapoint(self.reg_pre_fault, self.reg_post_fault, aw.position, {}, None, None)
        aw.a_log(d.get_regs_flipped())
        pprint(d.get_regs_flipped())
        return False

    def reset_target(self) -> None:
        self.device.reset("halt")
        self.reg_pre_fault = self.device.reg()

    def critical_check(self) -> bool:
        return True

    def shutdown(self) -> None:
        self.cs.armed = 0


if __name__ == '__main__':
    Probing()
