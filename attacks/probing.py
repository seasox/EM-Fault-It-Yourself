import socket
import uuid
import time
from dataclasses import dataclass, field
from enum import Enum
from pprint import pprint
import numpy as np
from chipshouter import ChipSHOUTER
from emfi_station import Attack
from typing import Tuple, Optional, Callable, Any, List, Dict
from typing_extensions import Literal


class OpenOCD:
    encoding = "utf-8"
    EOF = bytes('\x1a', encoding=encoding)

    def __init__(self, host='localhost', port=6666, _socket=None,
                 value_cast: Callable[[int, int], Any] = lambda x, bit_width: x):
        self._host = host
        self._port = port
        self._buffer_size = 4096
        self._socket = _socket or socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._device_layout = None
        self._value_cast = value_cast

    def _recv(self):
        data, tmp = bytes(), bytes()
        while OpenOCD.EOF not in tmp:
            tmp = self._socket.recv(self._buffer_size)
            data += tmp
        data = data.decode(OpenOCD.encoding).strip()
        # Strip trailing EOF.
        data = data[:-1]
        return data

    def execute(self, command: str):
        data = command.encode(OpenOCD.encoding) + OpenOCD.EOF
        self._socket.sendall(data)
        try:
            return self._recv()
        except socket.timeout:
            raise TimeoutError

    def reset(self, param: Optional[Literal["run", "halt", "init"]] = None):
        res = self.execute(f"reset {param if param else ''}")
        print(res)

    def halt(self, ms: Optional[int] = None):
        """Halt the target execution."""
        self.execute(f"halt {ms if ms else ''}")

    def shutdown(self):
        """Shutdown the OpenOCD server."""
        self.execute("shutdown")

    def close(self):
        """Close the connection."""
        self._socket.close()

    @staticmethod
    def __get_safe(reg: Any, start: int, end: int = None, step: int = 1, optional=None):
        if reg is None:
            return optional
        if end is None:
            if start < len(reg):
                return reg[start]
            else:
                return optional
        else:
            return reg[start:end:step]

    def __to_data_entry(self, res_fields: List):
        reg_name = OpenOCD.__get_safe(res_fields, 0)
        bit_width_raw = OpenOCD.__get_safe(res_fields, 1)
        bit_width = OpenOCD.__get_safe(bit_width_raw, 2, -2)
        content = OpenOCD.__get_safe(res_fields, 2)
        dirty = OpenOCD.__get_safe(res_fields, 3, optional=False)

        if bit_width is not None:
            try:
                bit_width = int(bit_width, base=10)
            except ValueError:
                content = None  # cannot reliably determine register content
                bit_width = None
        if content is not None:
            try:
                content = self._value_cast(int(content, 16), bit_width)
            except ValueError:
                content = None

        corrupted = any((reg_name is None,
                         bit_width is None,
                         content is None))
        reg_name = reg_name or f"Corrupted_{str(uuid.uuid4())[:5]}"

        return reg_name, {"Width": bit_width, "Content": content, "Dirty": dirty, "Corrupted": corrupted}

    def reg(self, name: Optional[str] = None, value: Optional[int] = None, force: Optional[bool] = None):
        if force and value:
            return None
        force = "-force" if force else ""
        if name is None:
            res = [reg.split()[1:] for reg in self.execute("reg").split("\n")[1:-2]]
        elif value is None:  # get the value of the register
            res = [self.execute(f"reg {name} {force}").split()]
        else:  # we set the value
            res = [self.execute(f"reg {name} {hex(value)}").split()]
        data = {}

        for reg in res:
            key, value = self.__to_data_entry(reg)
            data[key] = value
        return data

    def connect(self):
        """Establish a connection to the OpenOCD server."""
        self._socket.connect((self._host, self._port))

    @property
    def host(self):
        """Hostname of the OpenOCD server."""
        return self._host

    @property
    def port(self):
        """Port number of the OpenOCD Tcl interface."""
        return self._port


class BitFlip(Enum):
    ZERO_TO_ZERO = 0
    ONE_TO_ONE = 1
    ZERO_TO_ONE = 2
    ONE_TO_ZERO = 3


class Metric(Enum):
    AnyFlipAnywhere = 1


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
    regs_flipped: dict = field(init=False)

    def __post_init__(self):
        self.mem_diff = self._calc_mem_diff()
        self.reg_diff = self._calc_reg_diff()
        self.regs_flipped = self._calc_regs_flipped()
        if not self.config.get("store_memory"):  # does not store memory dump by default
            del self.mem_pre_fault
            del self.mem_post_fault

    def _calc_mem_diff(self):
        return {}

    def _calc_reg_diff(self) -> Dict:
        if self.regs_pre_fault.keys() != self.regs_post_fault.keys():
            # some register values are not available
            regs = set(self.regs_pre_fault.keys()).intersection(self.regs_post_fault.keys())
        else:
            regs = self.regs_pre_fault.keys()
        data = {}
        for reg in regs:
            if self.regs_pre_fault[reg]["Corrupted"] or self.regs_post_fault[reg]["Corrupted"]:
                data[reg] = "Corrupted"
                continue
            distr = diff(self.regs_pre_fault[reg]["Content"], self.regs_post_fault[reg]["Content"])
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
            return self.regs_flipped["total"] > 0


class Probing(Attack):
    to_bits = lambda x, bit_width: np.unpackbits(
        np.frombuffer(x.to_bytes(bit_width, byteorder="big"), dtype=np.uint8), bitorder="big")[-bit_width:]
    cs: ChipSHOUTER

    def __init__(self):
        super().__init__(start_pos=(0, 48, 125),
                         end_pos=(14, 63, 125),
                         step_size=1,
                         max_target_temp=40,
                         cooling=1,
                         repetitions=3)
        self.device = OpenOCD(value_cast=Probing.to_bits)
        self.device.connect()
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
        return np.zeros((dx, dy, max(dz, 1)))

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
        d = Datapoint(self.prev_reg, self.device.reg(), self.aw.position, {}, None, None)
        x, y, z = (np.array(self.aw.position) - self.start_pos) // self.step_size
        print(x,y,z)
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
        self.device.reset("halt")
        self.prev_reg = self.device.reg()

    def critical_check(self) -> bool:
        return True

    def shutdown(self) -> None:
        self.cs.armed = 0


if __name__ == '__main__':
    Probing().init(None)
