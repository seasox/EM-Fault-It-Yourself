from dataclasses import dataclass
from typing import Generic, TypeVar, Callable, List

from typing_extensions import Literal

_ReadingType = TypeVar('_ReadingType')  # this is a generic representing the value_cast return type (e.g., np.array)


@dataclass
class RegisterReading(Generic[_ReadingType]):
    width: int
    # noinspection PyUnresolvedReferences
    content: '_ReadingType'  # TODO weird warning, maybe https://github.com/python/mypy/issues/7520
    dirty: bool
    corrupted: bool


class STLinkComm(Generic[_ReadingType]):
    """
    This class encapsulates the pystlink library. It allows to easily reset and get all registers from an attached
    STLink capable device from your attack. It is generified over _ReadingType, which is the return type of your
    value_cast callback (see initializer doc).
    """

    _driver: pystlink.lib.stm32.Stm32

    """
    Initialize the STLinkComm. The driver to use depends on the specific MCU attached. Take a look at the output of 
    pystlink/pystlink.py to figure out which driver to use for your MCU. In doubt, a 
    `grep -R 'STM32F{YOURMCU}' pystlink/'
    might help. Additionally, you may pass a `value_cast' to transform readings into a data format appropriate for your
    use case. value_cast takes an integer reading `x' as well as a bit_width (e.g. 16 or 32) and transforms those into
    a format suitable for your custom analysis (e.g., np.array).
    """

    def __init__(self, serial: str, value_cast: Callable[[int, int], _ReadingType] = lambda x, bit_width: x):
        DBG_QUIET = 0  # use 0..3 to control debug output verbosity (0: quiet, 1: normal, 2: verbose, 3: debug)
        dbg = pystlink.lib.dbg.Dbg(DBG_QUIET)
        self._connector = pystlink.lib.stlinkusb.StlinkUsbConnector(dbg=dbg, serial=serial, index=0)
        self._driver = pystlink.lib.stm32fs.Stm32FS(pystlink.lib.stlinkv2.Stlink(self._connector, dbg=dbg), dbg)
        self._value_cast = value_cast

    def reset(self, cmd: Literal[None, 'halt'] = None):
        """
        reset the chip, optionally halting after reset
        """
        if not cmd:
            self._driver.core_reset()
            return
        if cmd == 'halt':
            self._driver.core_reset_halt()
            return
        raise Exception(f'unknown cmd "{cmd}"')

    """
    Get a list of all register names
    """

    def get_reg_names(self) -> List[str]:
        return pystlink.lib.stm32.Stm32.REGISTERS

    def regs(self) -> dict[str, RegisterReading]:
        """
        Get register values.
        :return a name, value dictionary containing RegisterReadings, where values are cast by `value_cast'.
        """
        readings = dict(self._driver.get_reg_all())
        ret = {}
        for k, v in readings.items():
            ret[k] = RegisterReading(
                width=32,  # TODO: bit_width depends on the MCU. We should query the driver for the register bit widths
                content=self._value_cast(v, 32),
                dirty=False,
                corrupted=False,
            )
        return ret

    def close(self):
        self._connector.close()
