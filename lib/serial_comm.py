from threading import Thread
from time import time

from loguru import logger
from serial import STOPBITS_ONE, Serial


class SerialComm:
    def __init__(
        self,
        port: str = "/dev/ttyACM0",
        baudrate: int = 115200,
        terminator: str = "$",
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.terminator = terminator

        self.__serial = Serial(
            port=port,
            baudrate=baudrate,
            stopbits=STOPBITS_ONE,
        )

        self.RX_TICK = 1 / 30
        self.TX_TICK = 1 / 15

        self.rx_pkg: str = "0200"
        self.__rx_pkg: str = ""
        self.new_rx_pkg_fl: bool = False

        self.tx_pkg: str = "54545454000"
        self.__tx_tmr: float = time()
        self.received = False

        self.__thread = Thread(target=self.__tick)
        self.__thread.daemon = True
        self.__thread.start()

        # self.__rx_thread = Thread(target=self.__rx_tick)
        # self.__rx_thread.daemon = True
        # self.__rx_thread.start()
        #
        # self.__tx_thread = Thread(target=self.__tx_tick)
        # self.__tx_thread.daemon = True
        # self.__tx_thread.start()

    def __tick(self) -> None:
        while True:
            if time() - self.__tx_tmr > 0.1 or self.received:
                self.__serial.write((self.tx_pkg + self.terminator).encode("utf-8"))
                self.tx_tmr = time()

            rx_str = ""
            timeout_tmr = time()

            while True:
                if self.__serial.in_waiting:
                    ret = self.__serial.read().decode("utf-8")

                    if time() - timeout_tmr > 0.05:
                        logger.warning("Reading timeout")
                        break

                    if ret != "$":
                        rx_str += ret

                    else:
                        if len(rx_str) == 4:
                            self.received = True
                            logger.info(f"{rx_str[0]}, {rx_str[1:4]}")
                            self.rx_pkg = rx_str
                        break

    def __rx_tick(self) -> None:
        while True:
            if self.__serial.in_waiting:
                ret = str(self.__serial.read(), "utf-8")

                if ret.isdigit():
                    if len(self.__rx_pkg) < 4:
                        self.__rx_pkg += ret

                elif ret == "$" and len(self.__rx_pkg) == 4:
                    self.rx_pkg = self.__rx_pkg
                    self.__rx_pkg = ""
                    self.new_rx_pkg_fl = True

                    logger.info(f"New Valid Package -> {self.rx_pkg}")

    def __tx_tick(self) -> None:
        while True:
            if time() - self.__tx_tmr > self.TX_TICK:
                self.__tx_tmr = time()

                _tx_pkg = str(self.tx_pkg) + self.terminator
                self.__serial.write(_tx_pkg.encode("utf-8"))

                # logger.info("TX")
                # logger.info(_tx_pkg)


if __name__ == "__main__":
    ser = SerialComm()

    while True:
        pass
