from collections.abc import Sequence
from time import sleep, time

import cv2 as cv
import numpy as np
from cv2.typing import MatLike
from lib.fps import FramesPerSecond
from lib.MEGA import MEGA
from lib.pid import PDCompass
from loguru import logger
from serial import Serial
from static.bounds import Bounds

GATES_COURSE_0 = 120
GATES_COURSE_1 = 80
GATES_COURSE_2 = 240
GATES_TIME_0 = 13
GATES_TIME_1 = 13
GATES_TIME_2 = 13

SPONGE_HEAD = 120

PROBO_HEAD = 310
PROBO_BUTT = 300
SPEED = 70
START_STATE = "Probo"


class StateMachine:
    def __gates(self) -> None:
        if not self.mega.sc.new_frame:
            return

        match self.local_state:
            case "Start":
                self.gates_fps = FramesPerSecond()
                self.local_state = "by azi0"
                self.pdc.set_setpoint(GATES_COURSE_0)

                self.gates_tmr = time()

            case "by azi0":
                self.pdc.tick(self.mega.azimuth, bypass=2)

                self.mega.set_diff_vel(
                    l_vel=SPEED + self.pdc.U,
                    r_vel=SPEED - self.pdc.U,
                )

                if time() - self.gates_tmr > GATES_TIME_0:
                    self.local_state = "by azi1"

                    self.pdc.set_setpoint(GATES_COURSE_1)
                    self.gates_tmr = time()

            case "by azi1":
                self.pdc.tick(self.mega.azimuth, bypass=2)

                self.mega.set_diff_vel(
                    l_vel=SPEED + self.pdc.U,
                    r_vel=SPEED - self.pdc.U,
                )

                if time() - self.gates_tmr > GATES_TIME_1:
                    self.local_state = "by azi2"
                    self.pdc.set_setpoint(GATES_COURSE_2)

                    self.gates_tmr = time()

            case "by azi2":
                self.pdc.tick(self.mega.azimuth, bypass=2)

                self.mega.set_diff_vel(
                    l_vel=SPEED + self.pdc.U,
                    r_vel=SPEED - self.pdc.U,
                )

                if time() - self.gates_tmr > GATES_TIME_2:
                    self.local_state = "by azi2"
                    self.__update_state("Cube")

        self.gates_fps.tick()

    def __probo(self) -> None:
        if not self.mega.bc.new_frame:
            return

        match self.local_state:
            case "Start":
                self.pdc.set_setpoint(PROBO_HEAD)
                self.probo_tmr = time()
                self.local_state = "FindC"

            case "FindC":
                self.pdc.tick(self.mega.azimuth, bypass=2)

                self.mega.set_diff_vel(
                    l_vel=80 + self.pdc.U,
                    r_vel=80 - self.pdc.U,
                )

                self.mask = cv.inRange(
                    cv.cvtColor(
                        self.mega.bc.src,
                        cv.COLOR_BGR2HSV,
                    ),
                    *Bounds.SPONGE,
                )
                cv.blur(self.mask, (9, 9), self.mask)

                self.bc_panno = cv.cvtColor(self.mask, cv.COLOR_GRAY2BGR)

                conts = cv.findContours(
                    self.mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE
                )[0]

                max = 0
                ccc = None
                if len(conts) > 0:
                    for c in conts:
                        s = cv.contourArea(c)
                        if s > max and s > 100:
                            ccc = c
                            max = s

                            x, y, w, h = cv.boundingRect(c)

                            self.center = (x + w // 2, y + h // 2)

                if ccc is not None:
                    ex, ey = 160 - self.center[0], 120 - self.center[1]

                    self.pdc.set_setpoint(PROBO_BUTT)
                    self.probo_tmr = time()
                    self.local_state = "Align"
                    self.mega.set_winch_state(1)

            case "Align":
                self.pdc.tick(self.mega.azimuth, bypass=2)

                self.mega.set_omni_vel(
                    flv=54 + self.pdc.U,
                    blv=54 + self.pdc.U,
                    frv=54 - self.pdc.U,
                    brv=54 - self.pdc.U,
                )

                self.mask = cv.inRange(
                    cv.cvtColor(
                        self.mega.bc.src,
                        cv.COLOR_BGR2HSV,
                    ),
                    *Bounds.SPONGE,
                )
                cv.blur(self.mask, (9, 9), self.mask)

                self.bc_panno = cv.cvtColor(self.mask, cv.COLOR_GRAY2BGR)

                conts = cv.findContours(
                    self.mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE
                )[0]

                max = 0
                ccc = None
                if len(conts) > 0:
                    for c in conts:
                        s = cv.contourArea(c)
                        if s > max and s > 100:
                            ccc = c
                            max = s

                            x, y, w, h = cv.boundingRect(c)

                            self.center = (x + w // 2, y + h // 2)

                if ccc is not None:
                    ex, ey = 160 - self.center[0], 120 - self.center[1]

                    ex *= 0.1
                    ey *= 0.1

                    logger.info(f"{ex}    {ey}")

                    self.mega.add_omni_vel(
                        flv=ex - ey,
                        frv=-ex - ey,
                        blv=-ex - ey,
                        brv=ex - ey,
                    )

                if time() - self.probo_tmr > 20:
                    if self.mega.winch == 2:
                        self.__update_state("Cube")
                        self.mega.set_winch_state(0)

                    self.mega.set_winch_state(2)
                    self.probo_tmr = time()

    @staticmethod
    def get_cont_c_coords_by_m(cont: MatLike) -> tuple[int, int]:
        moments = cv.moments(cont)

        return (
            int(moments["m10"] // moments["m00"]),
            int(moments["m01"] // moments["m00"]),
        )

    @staticmethod
    def draw_cont(frame: MatLike, cont: MatLike) -> None:
        cx, cy = StateMachine.get_cont_c_coords_by_m(cont)
        x, y, w, h = cv.boundingRect(cont)

        cv.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 1)
        cv.circle(frame, (cx, cy), 3, (0, 255, 0), 2)

    @staticmethod
    def __find_biggest_cont(
        mask: MatLike, lower_area: float = 1000, upper_area: float = 1000
    ) -> MatLike | None:
        conts = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)[0]

        if conts is None:
            return None

        if not (len(conts) > 0):
            return None

        filtered_conts = [
            cont for cont in conts if lower_area <= cv.contourArea(cont) <= upper_area
        ]

        if filtered_conts is None:
            return None

        if not (len(filtered_conts) > 0):
            return None

        max_cont, max_area = None, 0
        for cont in filtered_conts:
            if area := cv.contourArea(cont) > max_area:
                max_area = area
                max_cont = cont

        return max_cont

    def __sponge_bomb(self) -> None:
        if not self.mega.bc.new_frame:
            return

        match self.local_state:
            case "Start":
                self.sponge_tmr = time()
                self.local_state = "Align"
                self.pdc.set_setpoint(SPONGE_HEAD)
                self.sponge_mask = None

            case "Align":
                self.sponge_mask = cv.inRange(
                    cv.cvtColor(
                        self.mega.bc.src,
                        cv.COLOR_BGR2HSV,
                    ),
                    *Bounds.SPONGE,
                )

                self.pdc.tick(self.mega.azimuth, bypass=2)

    def __cube(self) -> None:
        if not self.mega.sc.new_frame:
            return

        match self.local_state:
            case "Start":
                self.cube_fps = FramesPerSecond()
                self.cube_tmr = time()
                self.local_state = "2circle"

            case "2circle":
                self.pdc.tick(self.mega.azimuth, bypass=2)

                self.mega.set_diff_vel(
                    l_vel=40 + self.pdc.U,
                    r_vel=40 - self.pdc.U,
                )

                if time() - self.cube_tmr > 10:
                    self.local_state = "Bomb"
                    self.cube_tmr = time()

            case "Bomb":
                self.mega.idle()
                self.mega.set_cube_state(1)

                if time() - self.cube_tmr > 5:
                    self.local_state = "Idle"

            case "Idle":
                self.mega.idle()

    def __init__(self, mega: MEGA) -> None:
        self.mega = mega

        while True:
            try:
                if self.mega.sc._new_frame and self.mega.bc._new_frame:
                    break
            except:
                pass

        logger.info("Cams Initted")

        self.sc_panno: MatLike = self.mega.sc.l
        self.bc_panno: MatLike = self.mega.bc.src
        self.telemetry = [
            ("Azi/", self.mega.azimuth),
            ("Mode/", self.mega.mode),
            ("Stt/", f"{self.state}-{self.local_state}"),
            # ("FPS/", self.state_machine_fps.value),
            ("Pkg/", self.mega.tx_pkg),
        ]

        self.state = "Starting"
        self.state_tmr = time()
        self.local_state = "Start"

        self.state_machine_fps = FramesPerSecond()

        self.pdc = PDCompass(
            k_p=1,
            k_d=1,
            setpoint=0,
        )

        self.last_mode = "Manual"
        self.is_starting = False

    def __start(self) -> None:
        self.mega.idle()
        if self.mega.mode == "Autonomous":
            self.__update_state(START_STATE)
            self.is_starting = False
        else:
            if not self.is_starting:
                self.__update_state("Starting")
                logger.info(self.mega.mode)
                self.is_starting = True

    def tick(self) -> None:
        self.state_machine_fps.tick()

        self.sc_panno: MatLike = self.mega.sc.l
        self.bc_panno: MatLike = self.mega.bc.src

        if self.mega.mode == "Manual":
            self.__update_state("Starting")
            self.is_starting = True

        match self.state:
            case "Idle":
                self.__idle()
            case "Starting":
                self.__start()
            case "Gates":
                self.__gates()
            case "Cube":
                self.__cube()
            case "Probo":
                self.__probo()
            case "Sponge":
                self.__sponge_bomb()

        self.mega.sn.set_frame(
            cv.hconcat(
                [
                    self.sc_panno,
                    self.bc_panno,
                ]
            ),
            self.telemetry,
        )

    def __update_state(self, new_state: str) -> None:
        logger.info(f"New State: {new_state}")

        self.state = new_state
        self.local_state = "Start"
        self.mega.idle()

    def __idle(self) -> None:
        self.mega.idle()


if __name__ == "__main__":
    mega = MEGA()
    state_machine = StateMachine(mega)

    ser = Serial("/dev/ttyACM0", 115200)
    sleep(7.5)

    tx_tmr = time()
    received = False
    while True:
        state_machine.tick()

        if time() - tx_tmr > 0.05 or received:
            ser.write(state_machine.mega.tx_pkg.encode("utf-8"))

            tx_tmr = time()
            received = True

        rx_str = ""
        timeout_tmr = time()
        while True:
            try:
                if ser.in_waiting:
                    ret = ser.read().decode("utf-8")

                    if time() - timeout_tmr > 0.05 and 0:
                        break

                    if ret != "$":
                        rx_str += ret

                    else:
                        state_machine.last_mode = state_machine.mega.mode

                        state_machine.mega.mode = (
                            "Autonomous" if int(rx_str[0]) else "Manual"
                        )
                        state_machine.mega.azimuth = int(rx_str[1:4]) - 200

                        received = True
                        break
            except:
                pass
