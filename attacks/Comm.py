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
    EXPECTED_DATA_MISMATCH = 0
    RESET_UNSUCCESSFUL = 1
    END_SEQUENCE_NOT_FOUND = 2
    # if we didn't receive the fault window end sequence
    FAULT_WINDOW_TIMEOUT = 3

    def __str__(self):
        return self.name

    def __repr__(self):
        return self.name


@dataclass
class Response:
    status: set[STATUS]
    raw: BitArray | None = field(repr = False)
    reg_data: Dict[str, Register] | None


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
                 fault_window_start_seq:BitArray,
                 fault_window_end_seq: BitArray,
                 reg_data_expected: List[int],
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
        assert fault_window_start_seq.len % 8 == 0, "Fault window start sequence length must be a multiple of 8"
        assert fault_window_end_seq.len % 8 == 0, "Fault window end sequence length must be a multiple of 8"

        self.reg_data_expected = reg_data_expected
        self.fault_window_start_seq = fault_window_start_seq
        self.fault_window_end_seq = fault_window_end_seq

        self.reg_data_buffer_size = sum(self.reg_size)  # in bytes
        self.fault_window_start_seq_buffer_size = self.fault_window_start_seq.len // 8 # in bytes
        self.fault_window_end_seq_buffer_size = self.fault_window_end_seq.len // 8 # in bytes

        # make sure parameters are allowed
        assert len(self.regs) == len(self.reg_size), "All registers must have a corresponding size entry!"
        assert self.reg_data_expected is None or len(self.reg_data_expected) == len(self.reg_size), "The data expected in the registers must have the same number of bytes as the registers that are read"

        # init open ocd
        self.open_ocd: Optional[OpenOCD] = None
        if init_open_ocd:
            self.init_open_ocd()

        # Timing constants
        self.__low_time = .0001
        self.__high_time = .0001
        self.__wait_seq_time = 5

        self.__boot_up_time = 1.5
        self.__relay_time = .8

        self.reset()

    def init_open_ocd(self, open_ocd: Optional[OpenOCD] = None) -> None:
        self.open_ocd = open_ocd or OpenOCD()
        self.open_ocd.connect()

    def _high(self):
        GPIO.output(self.clk_pin, 1)
        time.sleep(self.__high_time)

    def _low(self):
        GPIO.output(self.clk_pin, 0)
        time.sleep(self.__low_time)

    def read(self, num_words: int, bits_per_word=8) -> BitArray:
        assert GPIO.input(self.clk_pin)  # assumes a high output
        num_bits = num_words * bits_per_word
        _buffer = ""
        for _ in range(num_bits):
            assert GPIO.input(self.clk_pin)  # assumes a high output
            self._low()
            _buffer += str(GPIO.input(self.miso_pin))
            self._high()
        return BitArray(bin=_buffer)

    def reset(self):
        # turn off device
        GPIO.output(self.reset_pin, 1)
        # wait until relay changes state
        time.sleep(self.__relay_time)
        GPIO.output(self.reset_pin, 0)
        # wait until relay changes state and device reboots
        time.sleep(self.__relay_time)
        time.sleep(self.__boot_up_time)



    def wait_fault_window_start(self) -> float:
        print("Waiting for start sequence...")
        start = time.time()
        _buffer = self.read(self.fault_window_start_seq_buffer_size)
        while True:
            assert _buffer.len == self.fault_window_start_seq.len
            if time.time() - start > self.__wait_seq_time:
                return -1
            if _buffer == self.fault_window_start_seq:
                return time.time() - start
            next_bit = self.read(num_words=1, bits_per_word=1)
            _buffer = BitArray(_buffer[1:]) + next_bit  # queues next bit keeps length

    def wait_fault_window_end(self) -> float:
        print("Waiting for end sequence...")
        start = time.time()
        _buffer = self.read(self.fault_window_end_seq_buffer_size)
        while True:
            assert _buffer.len == self.fault_window_end_seq.len
            if time.time() - start > self.__wait_seq_time:
                return -1
            if _buffer == self.fault_window_end_seq:
                return time.time() - start
            next_bit = self.read(num_words=1, bits_per_word=1)
            _buffer = BitArray(_buffer[1:]) + next_bit # queues next bit keeps length

    @staticmethod
    def hard_reset(bus_id: str, device_id: str) -> bool:
        import subprocess
        return subprocess.run(["/home/pi/usbreset", f"/dev/bus/usb/{bus_id}/{device_id}"]).returncode == 0
    @staticmethod
    def flash_firmware() -> bool:
        import subprocess
        ret_code = subprocess.run(["/home/pi/.platformio/penv/bin/pio", "run", "-t", "upload"], cwd="/home/pi/pycharm_mnt/EM-Fault-It-Yourself/attacks/stm32_probing/").returncode
        return ret_code == 0

    def read_regs(self) -> Response:
        _buffer = self.read(self.reg_data_buffer_size)
        _buffer_cp = _buffer.copy()
        status: set[STATUS] = set()
        def _parse_with_expected_data():
            _data: Dict[str, Register] = {}
            for i, (name, width) in enumerate(zip(self.regs, self.reg_size)):
                bit_width = width * 8
                actual = BitArray(bytes=_buffer.bytes[:width])
                expected = self.reg_data_expected[i]  # the value (int) we expect
                del _buffer[:bit_width]  # remove the bits from the buffer
                missmatch = expected != actual.uintle
                if missmatch: status.add(STATUS.EXPECTED_DATA_MISMATCH) # we can check for this flag later
                _data[name] = Register(name, bit_width, actual, corrupted=missmatch)
            return _data

        return Response(status, _buffer_cp, _parse_with_expected_data())