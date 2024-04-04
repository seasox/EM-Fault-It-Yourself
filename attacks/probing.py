import json
import time
import typing
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum, auto, IntEnum
from pathlib import Path
from typing import Tuple, Dict, List, Set

import numpy as np
from bitstring import BitArray
from chipshouter import ChipSHOUTER

from Comm import Comm, Register, Response, STATUS, ResetRelay, DatapointEncoder
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


@dataclass(eq=False)
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
        return self.reg_diff[reg_name].count(BitFlip.ZERO_TO_ONE) if reg_name in self.reg_diff else 0

    def get_10_flips(self, reg_name) -> int:
        return self.reg_diff[reg_name].count(BitFlip.ONE_TO_ZERO) if reg_name in self.reg_diff else 0

    def __repr__(self):
        s = ""
        for reg_name in self.reg_names:
            s += f"{reg_name} | 0 -> 1: {self.get_01_flips(reg_name)} | 1 -> 0: {self.get_10_flips(reg_name)} \n"
        return s

    def __eq__(self, other):
        return self.response_before_fault == other.response_before_fault \
            and self.response_after_fault == other.response_after_fault \
            and self.attack_location == other.attack_location \
            and self.reg_diff == other.reg_diff \
            and self.reg_names == other.reg_names

    def to_json(self) -> Dict[str, str | Dict | Tuple]:
        return {
            'type': 'Datapoint',
            'response_before_fault': self.response_before_fault,
            'response_after_fault': self.response_after_fault,
            'attack_location': [int(i) for i in self.attack_location],  # np int64 is not serializable
        }

    @staticmethod
    def from_json(dic: Dict[str, typing.Any]) -> 'Datapoint':
        assert dic['type'] == 'Datapoint'
        return Datapoint(response_before_fault=dic['response_before_fault'],
                         response_after_fault=dic['response_after_fault'],
                         attack_location=tuple(dic['attack_location']))


class Metric(Enum):
    AnyFlipAnywhere = auto()
    Crash = auto()
    ZeroOneFlipOnR4OrR5 = auto()
    ResetUnsuccessful = auto()


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
        case Metric.ZeroOneFlipOnR4OrR5:
            # a timeout or undefined behavior after fault is not what we expect here
            if STATUS.FAULT_WINDOW_TIMEOUT in dp.response_after_fault.status or STATUS.END_SEQUENCE_NOT_FOUND in dp.response_after_fault.status:
                return -1
            if sum([dp.get_01_flips(reg_name) for reg_name in ['r0', 'r1', 'r2', 'r3', 'r6', 'r7']]) > 0:
                return -1
            return sum([dp.get_01_flips(reg_name) for reg_name in ['r4', 'r5']])
        case Metric.ResetUnsuccessful:
            return STATUS.RESET_UNSUCCESSFUL in dp.response_before_fault.status


# chip dimensions: 9x9 mm
stm32l0_delta = (2, 5, 0)
stm32l0_start = (110, 62, 83)
stm32l0_end = add_tuples(stm32l0_start, stm32l0_delta)

# chip dimensions: 11x11 mm
device_name = "stm32f4discovery"
stm32f4_delta = (11, 11, 0)
stm32f4_start = (105, 60, 80.5)
stm32f4_start_offset = (7, 0, 0)
stm32f4_end_offset = (0, -8, 0)

stm32f4_end = add_tuples(stm32f4_start, stm32f4_delta)
stm32f4_start = add_tuples(stm32f4_start, stm32f4_start_offset)
stm32f4_end = add_tuples(stm32f4_end, stm32f4_end_offset)

repetitions = 1000


class Probing(Attack):
    cs: ChipSHOUTER
    response_before_fault: Response
    response_after_fault: Response

    def __init__(self):
        # noinspection PyTypeChecker
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
        self.mosi_pin = 10
        self.clk_pin = 11
        self.reset_pin = 0
        self.dut_prep_time = .01  # in seconds

        # The end sequence acts like a "checksum", if it is not transferred correctly, we cant be sure about the results
        self.end_seq = BitArray(bytes=b"\x42\x42\x42\x42")
        self.fault_window_start_seq = BitArray(bytes=b"\x2a\x2a\x2a\x2a")
        self.fault_window_end_seq = BitArray(bytes=b"\x13\x37\x13\x37")
        # each register sends a number that distributes 0/1 evenly. r7 is used as a round counter
        self.expected_data = [0x00000000,
                              0x00000000,
                              0x00000000,
                              0x00000000,
                              0x00000000,
                              0x00000000,
                              0x00000000,
                              0]
        self.reset = ResetRelay(reset_pin=self.reset_pin)
        self.device = Comm(reset=self.reset,
                           miso_pin=self.miso_pin,
                           mosi_pin=self.mosi_pin,
                           clk_pin=self.clk_pin,
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
        self.cs.voltage = 150
        self.cs.pulse.repeat = 3

    def shout(self) -> bool:
        self.log.info("Waiting for start sequence...")
        time_taken = self.device.wait_fault_window_start()
        if time_taken < 0:
            self.log.info("Did not find start sequence")
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
            return True

    def was_successful(self) -> bool:
        time_taken = self.device.wait_fault_window_end()
        self.log.info(f"Waiting for fault window end took {time_taken} seconds")
        if time_taken < 0:  # a timeout, we cannot say anything about the device's state!
            self.log.info('Did not find fault window end sequence')
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
        self.reset.reset()
        self.response_before_fault = self.device.read_regs()

    def critical_check(self) -> bool:
        return True

    def shutdown(self) -> None:
        timestamp = datetime.now().strftime("%Y-%m-%d-%H:%M:%S")
        dp_dir = Path("dp_json")
        dp_dir.mkdir(parents=True, exist_ok=True)
        filename = f"data_{timestamp}.json"
        filepath = dp_dir.joinpath(filename)
        self.log.info(f'writing progress to JSON due to shutdown')
        with open(filepath, "w+") as fp:
            try:
                d = {
                    "datapoints": self.dps,
                    "settings": {
                        "cs": {
                            "voltage": self.cs.voltage.set,
                            "pulse_repeat": self.cs.pulse.repeat
                        },
                        "experiment": {
                            "expected_data": self.expected_data,
                            "start": self.start_pos,
                            "end": self.end_pos,
                            "device_name": device_name,
                            "start_offset": stm32f4_start_offset,
                            "end_offset": stm32f4_end_offset,
                            "repetitions": repetitions,
                        }
                    }
                }
                json.dump(d, fp, cls=DatapointEncoder)
            except Exception as e:
                self.log.error('Error writing progress to JSON: ' + str(e))
        self.cs.armed = 0
        self.reset.reset()
