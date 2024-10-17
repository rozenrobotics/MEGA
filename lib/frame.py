from cv2.typing import MatLike


class Frame:
    def __init__(self, src: MatLike) -> None:
        self._src = src
        self._processed = False

    def __call__(self) -> MatLike:
        self._processed = True
        return self._src
