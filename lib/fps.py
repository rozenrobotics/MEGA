from time import time


class FramesPerSecond:
    def __init__(self) -> None:
        self.value = 0

        self.__value: int = 0
        self.__tmr: float = time()

    @property
    def fps(self) -> int:
        self.tick()
        return self.value

    def tick(self) -> None:
        self.__value += 1
        if time() - self.__tmr > 1:
            self.value = self.__value

            self.__tmr = time()
            self.__value = 0

