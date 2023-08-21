import os
import pickle
import time
from dataclasses import dataclass, field
from enum import Enum, auto, IntEnum
from pathlib import Path
from typing import Tuple, Dict, List, Set

import numpy as np
from bitstring import BitArray
from chipshouter import ChipSHOUTER

from Comm import Comm, Register, Response, STATUS
from emfi_station import Attack
from emfi_station.utils import add_tuples


class BitFlip(IntEnum):
    ZERO_TO_ZERO = auto()
    ONE_TO_ONE = auto()
    ZERO_TO_ONE = auto()
    ONE_TO_ZERO = auto()

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
    reg_names: Set[str] = field(init=False)

    def __post_init__(self):
        self.__regs_before_fault = self.response_before_fault.reg_data
        self.__regs_after_fault = self.response_after_fault.reg_data

        if self.__regs_before_fault is not None and self.__regs_after_fault is not None:
            self.reg_names = set(self.__regs_before_fault.keys()).intersection(self.__regs_after_fault.keys())
            self.reg_diff: Dict[str, List[BitFlip]] = self.__calc_reg_diff()
        else:
            self.reg_names = set()
            self.reg_diff = dict()

    def __calc_reg_diff(self) -> Dict[str, List[BitFlip]]:
        data = {}

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

        for reg_name in self.reg_names:
            pre = self.__regs_before_fault[reg_name]
            post = self.__regs_after_fault[reg_name]
            if pre.data is not None and post.data is not None:
                data[reg_name] = _diff(pre, post)
        return data

    def get_01_flips(self, reg_name) -> int:
        return self.reg_diff[reg_name].count(BitFlip.ZERO_TO_ONE)

    def get_10_flips(self, reg_name) -> int:
        return self.reg_diff[reg_name].count(BitFlip.ONE_TO_ZERO)

    def __repr__(self):
        s = ""
        for reg_name in self.reg_names:
            s += f"{reg_name} | 0 -> 1: {self.get_01_flips(reg_name)} | 1 -> 0: {self.get_10_flips(reg_name)} \n"
        return s


class Metric(Enum):
    AnyFlipAnywhere = auto()
    Crash = auto()


def evaluate(dp: Datapoint, metric: Metric) -> float:
    """
    Returns how well this datapoint performed given some metric. Either a positive value or -1 if unsuccessful
    """
    # if sanity check was not successful no metric performs well
    if STATUS.RESET_UNSUCCESSFUL in dp.response_before_fault.status:
        return -1

    match metric:
        case Metric.AnyFlipAnywhere:
            # a timeout or undefined behavior after fault is not what we expect here
            if STATUS.FAULT_WINDOW_TIMEOUT in dp.response_after_fault.status or STATUS.END_SEQUENCE_NOT_FOUND in dp.response_after_fault.status:
                return -1
            return sum([dp.get_01_flips(reg_name) + dp.get_10_flips(reg_name) for reg_name in dp.reg_names])
        case Metric.Crash:
            if STATUS.FAULT_WINDOW_TIMEOUT in dp.response_after_fault.status or STATUS.END_SEQUENCE_NOT_FOUND in dp.response_after_fault.status:
                return 1
            return 0


# chip dimensions: 9x9 mm
stm32l0_delta = (2, 5, 0)
stm32l0_start = (110, 62, 83)
stm32l0_end = add_tuples(stm32l0_start, stm32l0_delta)

# chip dimensions: 12x12 mm
stm32f4_delta = (12, 12, 0)
stm32f4_start = (99, 60, 80)
stm32f4_start_offset = (8, 0, 0)
stm32f4_end_offset = (0, -2, 0)

stm32f4_end = add_tuples(stm32f4_start, stm32f4_delta)

stm32f4_start = add_tuples(stm32f4_start, stm32f4_start_offset)
stm32f4_end = add_tuples(stm32f4_end, stm32f4_end_offset)

repetitions = 1000


class Probing(Attack):
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
        self.metric = Metric.AnyFlipAnywhere
        self.aw = None

        # Parameters based on behavior of Device firmware, using BCM pins
        self.reg_size = 4  # in bytes
        self.regs = 8
        self.miso_pin = 9
        self.clk_pin = 11
        self.reset_pin = 0
        self.dut_prep_time = .01  # in seconds

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

        # The datapoints of a given x,y,z location
        self.dps: List[List[List[List[Datapoint]]]] = self.__init_dp_matrix()

    @staticmethod
    def name() -> str:
        return f"Probing Attack ({stm32f4_start} => {stm32f4_end}, {repetitions})"

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
        self.cs.pulse.repeat = 5

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
            self.response_after_fault = Response({STATUS.FAULT_WINDOW_TIMEOUT}, None, None)
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
                self.response_after_fault.status.add(STATUS.END_SEQUENCE_NOT_FOUND)

        x, y, z = (np.array(self.aw.position) - self.start_pos) / self.step_size
        x, y, z = np.round(np.array([x, y, z])).astype(int)
        d = Datapoint(self.response_before_fault, self.response_after_fault, (x, y, z))
        performance = evaluate(d, self.metric)
        self.dps[x][y][z].append(d)

        self.log.info(f"Performance: {performance},"
                      f" Status[before]: {self.response_before_fault.status},"
                      f" Status[after]: {self.response_after_fault.status}")

        if performance > 0:
            self.log.info(
                f"The following registers are faulted: {[reg_name for reg_name in self.response_after_fault.reg_data if self.response_after_fault.reg_data[reg_name].is_faulted]}")

        if self.metric == Metric.AnyFlipAnywhere:
            return performance > 0

        raise LookupError(f"Metric {self.metric} not covered")

    def reset_target(self) -> None:
        self.device.reset()
        self.response_before_fault = self.device.read_regs()

    def critical_check(self) -> bool:
        return True

    def shutdown(self) -> None:
        from datetime import datetime
        timestamp = datetime.now().strftime("%Y-%m-%d-%H:%M:%S")
        _dir = Path("pickles")
        if not _dir.exists():
            os.makedirs(_dir)
        filename = f"data_{timestamp}.pickle"
        filepath = _dir.joinpath(filename)
        fp = open(filepath, "wb")
        # evaluate the data:
        self.log.debug(f'writing progress to pickle due to shutdown')
        pickle.dump(self.dps, fp)
        # storage_fp.close()
        self.cs.armed = 0
        self.device.reset()
        print("End...")


if __name__ == '__main__':
    Probing()
