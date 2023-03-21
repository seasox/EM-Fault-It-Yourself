import socket
import uuid
from typing import Callable, Any, Optional, List

import numpy as np
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

    def get_reg_names(self):
        names = self.reg().keys()
        return names

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

    def force_reg(self):
        regs = self.reg()
        res = self.execute(f"get_reg -force {{ {' '.join(regs.keys())} }}").split(" ")
        data = {}
        res = np.reshape(res, newshape=(-1, 2))
        for reg, content in res:
            bit_width = regs[reg]["Width"]
            data[reg] = {"Width": bit_width,
                         "Content": self._value_cast(int(content, 16), bit_width),
                         "Dirty": False,
                         "Corrupted": False}
        return data

    def reg(self, name: Optional[str] = None, value: Optional[int] = None, force: bool = False):
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