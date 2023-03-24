import time
from typing import Optional, List, Dict
import socket
from dataclasses import dataclass, field
from typing_extensions import Literal
import RPi.GPIO as GPIO
from bitstring import BitArray


@dataclass
class Register:
    name: str
    width: int  # in bits
    data: Optional[BitArray]
    data_uint: Optional[int] = field(init=False, default=None)
    data_int: Optional[int] = field(init=False, default=None)
    data_float: Optional[float] = field(init=False, default=None)
    dirty: Optional[bool] = field(default=False)
    corrupted: bool = field(default=False)
    bitorder: Literal["little", "big"] = field(default="little")

    def __post_init__(self):
        # the integer cast only requires full bytes:
        if self.width % 8 != 0: return

        if self.bitorder == "little":
            self.data_uint = None if self.data is None else self.data.uintle
            self.data_int = None if self.data is None else self.data.intle
        else:
            self.data_uint = None if self.data is None else self.data.uintbe
            self.data_int = None if self.data is None else self.data.intbe

        # we can add more fields if its is 16/32 or 64 bit value
        if self.width not in [16, 32, 64]: return

        if self.bitorder == "little":
            self.data_float = None if self.data is None else self.data.floatle
        else:
            self.data_float = None if self.data is None else self.data.floatbe


class OpenOCD:
    encoding = "utf-8"
    EOF = bytes('\x1a', encoding=encoding)

    def __init__(self, host='localhost', port=6666, _socket=None):
        self._host = host
        self._port = port
        self._buffer_size = 4096
        self._socket = _socket or socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._device_layout = None

    def _recv(self) -> str:
        data, tmp = bytes(), bytes()
        while OpenOCD.EOF not in tmp:
            tmp = self._socket.recv(self._buffer_size)
            data += tmp
        data = data.decode(OpenOCD.encoding).strip()
        # Strip trailing EOF.
        data = data[:-1]
        return data

    def execute(self, command: str) -> str:
        """Executes a command"""
        data = command.encode(OpenOCD.encoding) + OpenOCD.EOF
        self._socket.sendall(data)
        try:
            return self._recv()
        except socket.timeout:
            raise TimeoutError

    def reset(self, param: Optional[Literal["run", "halt", "init"]] = None):
        """Resets the device"""
        self.execute(f"reset {param if param else ''}")

    def halt(self, ms: Optional[int] = None):
        """Halt the target execution."""
        self.execute(f"halt {ms if ms else ''}")

    def shutdown(self):
        """Shutdown the OpenOCD server."""
        self.execute("shutdown")

    def close(self):
        """Close the connection."""
        self._socket.close()

    def get_reg_names(self) -> List[str]:
        """Returns all register names of the connected device"""
        return list(self.reg().keys())

    @staticmethod
    def __to_register(res_fields: List) -> Register:
        # they should always exist:
        reg_name = res_fields[0]
        bit_width = int(res_fields[1][2:-2], base=10)
        # can be non-existent
        if len(res_fields) >= 3:
            content = res_fields[2]
            dirty = len(res_fields) >= 4  # if there is a dirty flag
            return Register(reg_name, bit_width, BitArray(hex=content), dirty=dirty, corrupted=False)
        else:  # is corrupted
            return Register(reg_name, bit_width, None, dirty=None, corrupted=True)

    def resume(self, address: Optional[int] = None):
        """
        Resume the target at its current code position, or the optional address if it is provided. OpenOCD will wait 5 seconds for the target to resume.
        """
        address = hex(address) if address is not None else ""
        self.execute(f"resume {address}")
        time.sleep(5)

    def reg(self, name: Optional[str] = None, value: Optional[int] = None, force: bool = False) -> Dict[str, Register]:
        """The reg instruction as described in the OpenOCD doc"""
        assert not (force and value is not None), "Force and setting a register is not allowed"
        force = "-force" if force else ""
        if name is None:
            res = [reg.split()[1:] for reg in self.execute("reg").split("\n")[1:-2]]
        elif value is None:  # get the value of the register
            res = [self.execute(f"reg {name} {force}").split()]
        else:  # we set the value
            res = [self.execute(f"reg {name} {hex(value)}").split()]
        data = {}

        for reg in res:
            register = self.__to_register(reg)
            data[register.name] = register
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


class Comm:
    def __init__(self,
                 miso_pin: int,
                 clk_pin: int,
                 regs: int | List[str],
                 reg_size: int | List[int],
                 end_sequence: BitArray,
                 init_open_ocd=False):

        self.MISO = miso_pin
        self.CLK = clk_pin

        self.regs: List[str] = [f"r{i}" for i in range(regs)] if type(regs) is int else regs
        self.reg_size: List[int] = [reg_size] * len(self.regs) if type(reg_size) is int else reg_size
        self.open_ocd: Optional[OpenOCD] = self.init_open_ocd() if init_open_ocd else None
        assert len(regs) == len(reg_size), "All registers must have a corresponding size entry!"
        self.end_sequence = end_sequence
        assert self.end_sequence.len % 8 == 0, "End sequence length must be a multiple of 8"
        self.min_buffer_size = self.end_sequence.len // 8 + sum(self.reg_size)  # in bytes

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.MISO, GPIO.IN)
        GPIO.setup(self.CLK, GPIO.OUT)

        GPIO.output(self.CLK, 1)  # init clock to high

    def init_open_ocd(self, open_ocd: Optional[OpenOCD] = None):
        self.open_ocd = open_ocd or OpenOCD()
        self.open_ocd.connect()

    def find_end_sequence(self, data: BitArray) -> int:
        import re
        matches = list(re.finditer(self.end_sequence.bin, data.bin))

        if len(matches) == 1:
            return matches[0].start()
        if len(matches) > 1:
            print(
                "The end sequence was found more than once in the data. Returning the last index, hoping for the best")
            return matches[-1].start()
        if len(matches) == 0:
            raise IndexError

    def _high(self):
        GPIO.output(self.CLK, 1)
        time.sleep(.0001)

    def _low(self):
        GPIO.output(self.CLK, 0)
        time.sleep(.0001)

    def read(self, num_bytes: int) -> BitArray:
        num_bits = num_bytes * 8
        buffer = ""
        for _ in range(num_bits):
            assert GPIO.input(self.CLK)  # assumes a high output
            self._low()
            buffer += str(GPIO.input(self.MISO))
            self._high()
        return BitArray(bin=buffer)

    def read_regs(self) -> Dict[str, Register]:
        buffer = self.read(self.min_buffer_size)
        idx = self.find_end_sequence(buffer)

        if idx == len(buffer) - len(self.end_sequence):
            print("Correctly transferred package!")

        data = {}
        for name, width in zip(self.regs, self.reg_size):
            value = buffer.bytes[:width]  # get the size bytes
            del buffer[:width * 8]  # remove the bits from the buffer
            data[name] = Register(name, width*8, BitArray(bytes=value), dirty=False)
        return data
