from time import time


class FPS:
    def __init__(self) -> None:
        self.value = 1
        self.__fps = 0
        self.__fps_tmr = time()

    def tick(self) -> None:
        self.__fps += 1

        if self.__fps_tmr + 1 < time():
            self.value = self.__fps
            self.__fps = 0
            self.__fps_tmr = time()

    def __str__(self) -> str:
        return str(self.value)
