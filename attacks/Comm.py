import time
from enum import Enum
from typing import Optional, List, Dict
import socket
from dataclasses import dataclass, field
from typing_extensions import Literal
import RPi.GPIO as GPIO
from bitstring import BitArray
import re


@dataclass
class Register:
    name: str
    width: int  # in bits
    data: Optional[BitArray]
    data_uint: Optional[int] = field(init=False, default=None, repr=False)
    data_int: Optional[int] = field(init=False, default=None, repr=False)
    data_float: Optional[float] = field(init=False, default=None, repr=False)
    dirty: Optional[bool] = field(default=False, repr=False)
    corrupted: bool = field(default=False)
    bitorder: Literal["little", "big"] = field(default="little", repr=False)

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


class STATUS(Enum):
    EXPECTED_DATA_MISSMATCH = 0
    RESET_UNSUCCESSFUL = 1
    END_SEQUENCE_FOUND = 2

    def __str__(self):
        return self.name

    def __repr__(self):
        return self.name


@dataclass
class Response:
    status: set[STATUS]
    raw: BitArray = field(repr = False)
    reg_data: Dict[str, Register]


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
                 reset_pin: int,
                 regs: int | List[str],
                 reg_size: int | List[int],
                 end_sequence: BitArray,
                 expected_data: List[int] = None,
                 init_open_ocd=False):

        # Config pins
        self.miso_pin = miso_pin
        self.clk_pin = clk_pin
        self.reset_pin = reset_pin

        GPIO.setmode(GPIO.BCM)  # This is needed as adafruit uses BCM Mode
        GPIO.setup(self.miso_pin, GPIO.IN)
        GPIO.setup(self.clk_pin, GPIO.OUT)
        GPIO.setup(self.reset_pin, GPIO.OUT)

        GPIO.output(self.clk_pin, 1)  # init clock to high
        GPIO.output(self.reset_pin, 1)  # Reset is pulldown

        # init device specific config
        self.regs: List[str] = [f"r{i}" for i in range(regs)] if type(regs) is int else regs
        self.reg_size: List[int] = [reg_size] * len(self.regs) if type(reg_size) is int else reg_size
        assert end_sequence.len % 8 == 0, "End sequence length must be a multiple of 8"
        self.end_sequence = end_sequence
        self.expected_data = expected_data
        self.buffer_size = self.end_sequence.len // 8 + sum(self.reg_size)  # in bytes

        # make sure parameters are allowed
        assert len(self.regs) == len(self.reg_size), "All registers must have a corresponding size entry!"
        assert self.expected_data is None or len(self.expected_data) == len(self.reg_size), "The expected data must have the same number of bytes as the registers that are read"

        # init open ocd
        self.open_ocd: Optional[OpenOCD] = None
        if init_open_ocd:
            self.init_open_ocd()

        # Timing constants
        self.__low_time = .0001
        self.__high_time = .0001
        self.__reset_time = 1

        self.reset()

    def init_open_ocd(self, open_ocd: Optional[OpenOCD] = None) -> None:
        self.open_ocd = open_ocd or OpenOCD()
        self.open_ocd.connect()

    def find_end_sequence(self, data: BitArray) -> int:
        matches = list(re.finditer(self.end_sequence.bin, data.bin))
        if len(matches) == 1:
            return matches[0].start()
        if len(matches) > 1:
            return matches[-1].start()
        if len(matches) == 0:
            raise IndexError

    def _high(self):
        GPIO.output(self.clk_pin, 1)
        time.sleep(self.__high_time)

    def _low(self):
        GPIO.output(self.clk_pin, 0)
        time.sleep(self.__low_time)

    def read(self, num_bytes: int) -> BitArray:
        num_bits = num_bytes * 8
        buffer = ""
        for _ in range(num_bits):
            assert GPIO.input(self.clk_pin)  # assumes a high output
            self._low()
            buffer += str(GPIO.input(self.miso_pin))
            self._high()
        return BitArray(bin=buffer)

    def reset(self):
        GPIO.output(self.reset_pin, 0)
        time.sleep(self.__low_time)
        GPIO.output(self.reset_pin, 1)
        time.sleep(self.__reset_time)

    def read_regs(self, comp_expected=True) -> Response:
        buffer = self.read(self.buffer_size)
        original_buffer = buffer.copy()
        status: set[STATUS] = set()
        try:
            idx = self.find_end_sequence(buffer) # try to find the end sequence in the buffer
            # End sequence was found at the end
            if idx == len(buffer) - len(self.end_sequence):
                status.add(STATUS.END_SEQUENCE_FOUND)
        except IndexError:
            pass # we don't add the flag

        # Helper methods
        def _parse_with_no_expected_data(corrupted: bool):
            _data: Dict[str, Register] = {}
            for name, width in zip(self.regs, self.reg_size):
                bit_width = width * 8
                value = buffer.bytes[:width]  # get the size bytes
                del buffer[:bit_width]  # remove the bits from the buffer
                _data[name] = Register(name, bit_width, BitArray(bytes=value), dirty=False, corrupted=corrupted)
            return _data

        def _parse_with_expected_data():
            _data: Dict[str, Register] = {}

            for i, (name, width) in enumerate(zip(self.regs, self.reg_size)):
                bit_width = width * 8
                actual = BitArray(bytes=buffer.bytes[:width])
                expected = self.expected_data[i]  # the value (int) we expect
                del buffer[:bit_width]  # remove the bits from the buffer
                missmatch = expected != actual.uintle
                if missmatch: status.add(STATUS.EXPECTED_DATA_MISSMATCH) # we can check for this flag later
                _data[name] = Register(name, bit_width, actual, corrupted=missmatch)
            return _data

        # Parse the status register
        comp_expected = self.expected_data is not None and comp_expected

        if comp_expected:
            # we check each register against its expected value
            return Response(status, original_buffer, _parse_with_expected_data())

        if STATUS.END_SEQUENCE_FOUND not in status:
            # we cannot know if register corrupted, thus we assume corrupted
            return Response(status, original_buffer, _parse_with_no_expected_data(corrupted=True))

        if STATUS.END_SEQUENCE_FOUND in status:
            # END_SEQUENCE_FOUND indicates a correct transfer of data, thus corrupted = False
            return Response(status, original_buffer, _parse_with_no_expected_data(corrupted=False))

        raise LookupError(f"Status combination {status} and comp_expected = {comp_expected} not handled...")