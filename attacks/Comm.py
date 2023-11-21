import logging
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, List, Dict

from bitstring import BitArray
from typing_extensions import Literal


@dataclass
class Register:
    name: str
    width: int  # in bits
    data: Optional[BitArray]
    data_uint: Optional[int] = field(init=False, default=None, repr=False)
    data_int: Optional[int] = field(init=False, default=None, repr=False)
    data_float: Optional[float] = field(init=False, default=None, repr=False)
    dirty: Optional[bool] = field(default=False, repr=False)
    is_faulted: bool = field(default=False)
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


class STATUS(int, Enum):
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
    raw: BitArray | None = field(repr=False)
    reg_data: Dict[str, Register] | None


class ResetRelay:
    def __init__(self, reset_pin: int):
        import RPi.GPIO as GPIO
        self.__relay_time = .8
        self.__boot_up_time = 1.5
        self.reset_pin = reset_pin
        GPIO.setmode(GPIO.BCM)  # This is needed as adafruit uses BCM Mode
        GPIO.setup(self.reset_pin, GPIO.OUT)
        GPIO.output(self.reset_pin, 1)  # Reset is pulldown

    def reset(self):
        import RPi.GPIO as GPIO
        # turn off device
        GPIO.output(self.reset_pin, 1)
        # wait until relay changes state
        time.sleep(self.__relay_time)
        GPIO.output(self.reset_pin, 0)
        # wait until relay changes state and device reboots
        time.sleep(self.__relay_time)
        time.sleep(self.__boot_up_time)


class Comm:
    def __init__(self,
                 reset: ResetRelay,
                 miso_pin: int,
                 clk_pin: int,
                 regs: int | List[str],
                 reg_size: int | List[int],
                 fault_window_start_seq: BitArray,
                 fault_window_end_seq: BitArray,
                 reg_data_expected: List[int]):
        import RPi.GPIO as GPIO

        self.reset = reset

        self.log = logging.getLogger(__name__)

        # Config pins
        self.miso_pin = miso_pin
        self.clk_pin = clk_pin

        GPIO.setmode(GPIO.BCM)  # This is needed as adafruit uses BCM Mode
        GPIO.setup(self.miso_pin, GPIO.IN)
        GPIO.setup(self.clk_pin, GPIO.OUT)

        GPIO.output(self.clk_pin, 1)  # init clock to high

        # init device specific config
        self.regs: List[str] = [f"r{i}" for i in range(regs)] if type(regs) is int else regs
        self.reg_size: List[int] = [reg_size] * len(self.regs) if type(reg_size) is int else reg_size
        assert fault_window_start_seq.len % 8 == 0, "Fault window start sequence length must be a multiple of 8"
        assert fault_window_end_seq.len % 8 == 0, "Fault window end sequence length must be a multiple of 8"

        self.reg_data_expected = reg_data_expected
        self.fault_window_start_seq = fault_window_start_seq
        self.fault_window_end_seq = fault_window_end_seq

        self.reg_data_buffer_size = sum(self.reg_size)  # in bytes
        self.fault_window_start_seq_buffer_size = self.fault_window_start_seq.len // 8  # in bytes
        self.fault_window_end_seq_buffer_size = self.fault_window_end_seq.len // 8  # in bytes

        # make sure parameters are allowed
        assert len(self.regs) == len(self.reg_size), "All registers must have a corresponding size entry!"
        assert self.reg_data_expected is None or len(self.reg_data_expected) == len(
            self.reg_size), "The data expected in the registers must have the same number of bytes as the registers that are read"

        # Timing constants
        self.__low_time = .001
        self.__high_time = .001
        # the maximum time we wait for the device. Make sure to sync with the fault window from the DUT!
        self.__wait_end_seq_time = 3
        # we should not wait here anyway
        self.__wait_start_seq_time = 5

        self.reset.reset()

    def _high(self):
        import RPi.GPIO as GPIO
        GPIO.output(self.clk_pin, 1)
        time.sleep(self.__high_time)

    def _low(self):
        import RPi.GPIO as GPIO
        GPIO.output(self.clk_pin, 0)
        time.sleep(self.__low_time)

    def read(self, num_words: int, bits_per_word=8) -> BitArray:
        import RPi.GPIO as GPIO
        if not GPIO.input(self.clk_pin):  # assumes a high output
            self.log.error("Expected high GPIO state, but was low")
            self._high()
        num_bits = num_words * bits_per_word
        _buffer = ""
        for _ in range(num_bits):
            if not GPIO.input(self.clk_pin):  # assumes a high output
                self.log.warning("Expected high GPIO state, but was low")
                self._high()
            if not GPIO.input(self.clk_pin):  # assumes a high output
                self.log.error("Failed to fix clock state")
                assert False
            self._low()
            _buffer += str(GPIO.input(self.miso_pin))
            self._high()
        return BitArray(bin=_buffer)

    def wait_fault_window_start(self) -> float:
        start = time.time()
        _buffer = self.read(self.fault_window_start_seq_buffer_size)
        while True:
            assert _buffer.len == self.fault_window_start_seq.len
            if time.time() - start > self.__wait_start_seq_time:
                return -1
            if _buffer == self.fault_window_start_seq:
                return time.time() - start
            next_bit = self.read(num_words=1, bits_per_word=1)
            _buffer = BitArray(_buffer[1:]) + next_bit  # queues next bit keeps length

    def wait_fault_window_end(self) -> float:
        start = time.time()
        _buffer = self.read(self.fault_window_end_seq_buffer_size)
        while True:
            assert _buffer.len == self.fault_window_end_seq.len
            if time.time() - start > self.__wait_end_seq_time:
                return -1
            if _buffer == self.fault_window_end_seq:
                return time.time() - start
            next_bit = self.read(num_words=1, bits_per_word=1)
            _buffer = BitArray(_buffer[1:]) + next_bit  # queues next bit keeps length

    def read_regs(self) -> Response:
        _buffer = self.read(self.reg_data_buffer_size)
        _buffer_cp = _buffer.copy()
        status: set[STATUS] = set()

        def _fetch_registers():
            _data: Dict[str, Register] = {}
            for i, (reg_name, reg_width) in enumerate(zip(self.regs, self.reg_size)):
                bit_width = reg_width * 8
                actual = BitArray(bytes=_buffer.bytes[:reg_width])
                if self.reg_data_expected is not None:
                    expected = self.reg_data_expected[i]  # the value (int) we expect
                    del _buffer[:bit_width]  # remove the bits from the buffer
                    is_faulted = expected != actual.uintle
                    if is_faulted:
                        status.add(STATUS.EXPECTED_DATA_MISMATCH)  # we can check for this flag later
                else:
                    is_faulted = False
                _data[reg_name] = Register(reg_name, bit_width, actual, is_faulted=is_faulted)
            return _data

        return Response(status, _buffer_cp, _fetch_registers())
