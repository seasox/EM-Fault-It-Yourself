import socket, logging

from typing import Tuple, Optional, Callable, Any
from typing_extensions import Literal


class OpenOCD:
    encoding = "utf-8"
    EOF = bytes('\x1a', encoding=encoding)

    def __init__(self, host='localhost', port=6666, _socket=None,
                 value_cast: Callable[[str, int], Any] = lambda x, bit_width: int(x, 16)):
        self._host = host
        self._port = port
        self._buffer_size = 4096
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) or _socket
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

    def __to_data_entry(self, bit_width: int, content: str, dirty: bool):
        return {"Width": bit_width,
                "Content": self._value_cast(content, bit_width),
                "Dirty": dirty}

    def reg(self, name: Optional[str] = None, value: Optional[int] = None, force: Optional[bool] = None):
        # TODO are there more optional fields other than dirty?
        if force and value:
            return None
        force = "-force" if force else ""
        if name is None:
            res = self.execute("reg").split("\n")
            return {
                l[1]: self.__to_data_entry(int(l[2][2:-2]), l[3], len(l) == 5)
                for l in map(lambda row: row.split(), res[1:-2])}
        if value is None:  # get the value of the register
            res = self.execute(f"reg {name} {force}").split()
        else:  # we set the value
            res = self.execute(f"reg {name} {hex(value)}").split()
        return {res[0]: self.__to_data_entry(int(res[1][2:-2]), res[2], len(res) == 4)}

    def get_reg(self, registers: Tuple[str], force: Optional[bool] = None):
        force = "-force" if force else ""
        regs = " ".join(registers)
        self.execute(f"get_reg {force} {{ {regs} }}")

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
