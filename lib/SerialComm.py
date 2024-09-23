from threading import Thread
from time import time

from loguru import logger
from serial import STOPBITS_ONE, Serial


class SerialComm:
    def __init__(
        self, port: str = "dev/ttyACM0", baudrate: int = 115200, terminator: str = "$"
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.terminator = terminator

        self.dead_tmr = time()
        self.received = False

        self.input_pkg: str = ""
        self.output_pkg: str = ""

        self.serial_thread = Thread(target=self.tick)
        self.serial_thread.daemon = True

    def connect(self) -> None:
        if self.port is not None:
            self.serial = Serial(
                self.port, baudrate=self.baudrate, stopbits=STOPBITS_ONE
            )

            if self.serial is not None:
                logger.success("Serial Port Connected!")

                self.serial_thread.start()
                logger.success("Serial Thread Started!")
            else:
                logger.error("Serial Port NOT Connected!")

    def set_output_pkg(self, pkg: list) -> None:
        self.output_pkg = "".join(map(str, pkg)) + self.terminator

    def __parse_input_pkg(self) -> dict:
        _input_pkg_dict = dict()

        _input_pkg_dict["state"] = int(self.input_pkg[0])
        _input_pkg_dict["azimuth"] = int(self.input_pkg[1:4])

        return _input_pkg_dict

    def get_input_pkg(self) -> dict:
        return self.__parse_input_pkg()

    def tick(self) -> None:
        if not self.received:  # receiving
            _input_package: str = ""
            _dead_tmr = time()

            while True:
                try:
                    _char = str(self.serial.read(), "utf-8")
                except:
                    logger.error("Can not Read Char from Serial")
                    break

                if _char != self.terminator:
                    _input_package += _char
                else:
                    self.input_pkg = _input_package

                if _dead_tmr + 0.02 < time():
                    break
            self.serial.reset_input_buffer()
            self.received = True
            return

        if self.dead_tmr + 0.01 < time():
            self.serial.write(self.output_pkg.encode("utf-8"))
            self.dead_tmr = time()
        return
