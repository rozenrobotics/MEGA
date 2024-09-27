from typing import Any

import cv2
import numpy as np
from cv2.typing import MatLike, Moments


class Cont:
    def __init__(self, cont: MatLike) -> None:
        self._cont = cont

    @property
    def s(self) -> int:
        if self._s is None:
            self.__get_s()

        return self._s

    @property
    def c(self) -> tuple[int, int]:
        if self._c is None:
            self.__get_c()

        return self._c

    @property
    def r(self) -> tuple[int, int, int, int]:
        if self._r is None:
            self.__get_r()

        return self._r

    def __get_m(self) -> "Cont":
        self._m: Moments = cv2.moments(self._cont)
        return self

    def __get_c(self) -> "Cont":
        if self._m is None:
            self.__get_m()

        self._c: tuple[int, int] = (
            int(self._m["m10"] / self._m["m00"]),  # Cx
            int(self._m["m01"] / self._m["m00"]),  # Cy
        )
        return self

    def __get_s(self) -> "Cont":
        self._s = int(
            cv2.contourArea(self._cont) if self._m is None else self._m["m00"]
        )
        return self

    def __get_r(self) -> "Cont":
        self._x, self._y, self._w, self._h = cv2.boundingRect(self._cont)
        self._r = (self._x, self._y, self._w, self._h)
        return self


Conts = list[Cont]


class Frame:
    def __init__(self, src: MatLike | Any, cs: str = "bgr") -> None:
        self.src = src
        self.cs = cs

        self.h = self.src.shape[0]
        self.w = self.src.shape[1]

    def find_biggest_conts(self, conts_num: int) -> Conts:
        if self.conts is None:
            self.get_conts()

        return sorted(
            self.conts,
            key=lambda cont: cont.s,
        )[:conts_num]

    def get_conts(self) -> "Frame":
        self.conts: list[Cont] = [
            Cont(c)
            for c in cv2.findContours(
                self.src if self.cs == "mon" else self.mon,
                cv2.RETR_TREE,
                cv2.CHAIN_APPROX_SIMPLE,
            )[0]
        ]
        return self

    def in_range(self, bound: tuple[MatLike, MatLike] | Any) -> "Frame":
        self.rng = cv2.inRange(
            self.src,
            bound[0],
            bound[1],
        )
        return Frame(self.rng, "mon")

    def thresh(self, thr_val: int, inv: bool = False) -> "Frame":
        self.thr: MatLike = cv2.threshold(
            self.src if self.cs == "mon" else self.mon,
            thr_val,
            255,
            cv2.THRESH_BINARY if not inv else cv2.THRESH_BINARY_INV,
        )[1]
        return Frame(self.thr, "mon")

    def mk_bgr(self) -> "Frame":
        self.bgr: MatLike = cv2.cvtColor(self.src, self.__get_cvt_code("bgr"))
        return Frame(self.bgr, "bgr")

    def mk_hsv(self) -> "Frame":
        self.hsv: MatLike = cv2.cvtColor(self.src, self.__get_cvt_code("hsv"))
        return Frame(self.hsv, "hsv")

    def mk_mon(self) -> "Frame":
        self.mon: MatLike = cv2.cvtColor(self.src, self.__get_cvt_code("mon"))
        return Frame(self.mon, "mon")

    def __get_cvt_code(self, dst_cs: str) -> int:
        assert self.cs != dst_cs, "Can not convert: Colorspaces Equal"

        match self.cs:
            case "bgr":
                match dst_cs:
                    case "rgb":
                        return cv2.COLOR_BGR2RGB
                    case "hsv":
                        return cv2.COLOR_BGR2HSV
                    case "mon":
                        return cv2.COLOR_BGR2GRAY
            case "mon":
                match dst_cs:
                    case "bgr":
                        return cv2.COLOR_GRAY2BGR
                    case "rgb":
                        return cv2.COLOR_GRAY2RGB
        return 0
