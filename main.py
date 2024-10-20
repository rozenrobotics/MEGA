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

SPEED = 85 
START_STATE = "Gates"

GATES_COURSE_RED = 110
GATES_COURSE_GREEN = 240 
GATES_COURSE_BLUE = 240
GATES_TMR_RED = 17
GATES_TMR_GREEN = 6
GATES_TMR_BLUE = 2

GATES2_COURSE_RED = 15
GATES2_COURSE_GREEN = 315 
GATES2_COURSE_BLUE = 240
GATES2_TMR_RED = 6
GATES2_TMR_GREEN = 9
GATES2_TMR_BLUE = 12 

SPONGE_COURSE_HEAD = 285
SPONGE_TMR_SEARCH = 6
SPONGE_TMR_WINCH = 20
SPONGE_BOUND_CONT_AREA = 1000

CUBE_COURSE_BUTT = 25 
CUBE_TMR_BUTT = 5 
CUBE_TMR_CUBING = 2

LITTER_FLAG = True
LITTER_COURSE_ALIGN = 295 
LITTER_COURSE_GRAB_0 = 275 
LITTER_COURSE_GRAB_1 = 180
LITTER_COURSE_GRAB_2 = 75
LITTER_COURSE_EXIT = 3
LITTER_TMR_ALIGN = 8
LITTER_TMR_KISS = 7
LITTER_TMR_DRIFT = 10
LITTER_TMR_PUSH = 10
LITTER_TMR_EXIT = 10
LITTER_BOUND_CONT_AREA = 50
LITTER_BOUND_EXIT = 500

FINISH_COURSE_HEAD = 290 
FINISH_TMR = 18


class StateMachine:
    def __gates(self) -> None:
        if not self.mega.sc.new_frame:
            return

        match self.local_state:
            case "Start":
                self.pdc.set_setpoint(GATES_COURSE_RED)
                self.__update_local_state("Red")

            case "Red":
                self.__move_by_setpoint(SPEED + 13)

                if time() - self.state_tmr > GATES_TMR_RED: # pass gates
                    self.pdc.set_setpoint(GATES_COURSE_GREEN) # turns little to left 
                    self.__update_local_state("Green")

            case "Green":
                self.__move_by_setpoint(SPEED)

                if time() - self.state_tmr > GATES_TMR_GREEN: # aligned perp between blue gates
                    self.pdc.set_setpoint(GATES_COURSE_BLUE) # aligned perp between blue gates
                    self.__update_local_state("Blue")

            case "Blue": 
                self.__move_by_setpoint(SPEED)

                if 1 or time() - self.state_tmr > GATES_TMR_BLUE: # pass blue gates
                    self.__update_state("Sponge")

    def __sponge(self) -> None:
        if not self.mega.bc.new_frame:
            return

        match self.local_state:
            case "Start":
                self.pdc.set_setpoint(SPONGE_COURSE_HEAD)
                self.__update_local_state("Search")

            case "Search": # searching for a contour at the bottom
                self.__move_by_setpoint(SPEED)

                sponge_mask = self.__mk_hsv_mask(self.mega.bc.src, Bounds.SPONGE)
                sponge_cont = self.__get_biggest_cont(sponge_mask, SPONGE_BOUND_CONT_AREA)

                self.bc_panno = cv.cvtColor(sponge_mask, cv.COLOR_GRAY2BGR)

                if time() - self.state_tmr > SPONGE_TMR_SEARCH:
                    self.__move_by_setpoint(65)

                if sponge_cont is not None and time() - self.state_tmr > SPONGE_TMR_SEARCH:
                    self.pdc.set_setpoint(SPONGE_COURSE_HEAD)
                    self.mega.set_winch_state(1)

                    self.__update_local_state("Align")

            case "Align":
                self.__move_by_setpoint(speed=54)

                sponge_mask = self.__mk_hsv_mask(self.mega.bc.src, Bounds.SPONGE)
                sponge_cont = self.__get_biggest_cont(sponge_mask, SPONGE_BOUND_CONT_AREA)

                self.bc_panno = cv.cvtColor(sponge_mask, cv.COLOR_GRAY2BGR)

                if sponge_cont is None:
                    return 

                x, y, w, h = cv.boundingRect(sponge_cont)

                cx, cy = x + w // 2, y + h // 2 
                ex, ey = 160 - cx, 120 - cy

                ex *= 0.1
                ey *= 0.1

                self.mega.add_omni_vel(
                    flv=-ey + ex,
                    frv=-ey - ex,
                    blv=-ey - ex,
                    brv=-ey + ex,
                )

                if time() - self.state_tmr > SPONGE_TMR_WINCH:
                    if self.mega.winch == 2:
                        self.__update_state("Cube")
                        self.mega.vel_range = (10, 98)
                        return 

                    self.mega.set_winch_state(2)
                    self.mega.vel_range = (45, 60)
                    self.pdc.set_setpoint(CUBE_COURSE_BUTT)

                    self.__update_local_state(self.local_state)

    def __cube(self) -> None:
        match self.local_state:
            case "Start":
                self.pdc.set_setpoint(CUBE_COURSE_BUTT)
                self.__update_local_state("Butt2C")

            case "Butt2C":
                self.__move_by_setpoint(speed=45)

                if time() - self.state_tmr > CUBE_TMR_BUTT:
                    self.__update_local_state("Cubing")

            case "Cubing":
                self.__move_by_setpoint(speed=54)
                self.mega.set_cube_state(1)

                if time() - self.state_tmr > CUBE_TMR_CUBING:
                    self.__update_state("Gates2")

    def __2gates(self) -> None:
        match self.local_state:
            case "Start":
                self.pdc.set_setpoint(GATES2_COURSE_RED)
                self.__update_local_state("Red")

            case"Red":
                self.__move_by_setpoint(SPEED)

                if time() - self.state_tmr > GATES2_TMR_RED: # pass gates
                    self.pdc.set_setpoint(GATES2_COURSE_GREEN) # turns little to left 
                    self.__update_local_state("Green")

            case "Green":
                self.__move_by_setpoint(SPEED)

                if time() - self.state_tmr > GATES2_TMR_GREEN: # aligned perp between blue gates
                    self.pdc.set_setpoint(GATES2_COURSE_BLUE) # aligned perp between blue gates
                    self.__update_local_state("Blue")

            case "Blue": 
                self.__move_by_setpoint(SPEED)

                if time() - self.state_tmr > GATES2_TMR_BLUE - 3:
                    self.mega.vel_range = (45, 60)

                if time() - self.state_tmr > GATES2_TMR_BLUE: # pass blue gates
                    self.mega.idle()
                    self.__update_state("Litter" if LITTER_FLAG else "Finish")

    def __litter(self) -> None:
        if not self.mega.sc.new_frame:
            return

        match self.local_state:
            case "Start":
                self.pdc.set_setpoint(LITTER_COURSE_ALIGN)
                self.__update_local_state("Align")

            case "Align":
                self.mega.vel_range = (45, 60)
                self.__move_by_setpoint(speed=54)

                if time() - self.state_tmr > LITTER_TMR_ALIGN:
                    self.__update_local_state("Kiss")

            case "Kiss":
                self.__move_by_setpoint(60)

                scl = self.mega.sc.l.copy()

                litter_mask = self.__mk_hsv_mask(scl, (np.array([0, 100, 0]), np.array([20, 255, 255])))
                litter_cont = self.__get_biggest_cont_1(litter_mask, LITTER_BOUND_CONT_AREA)

                if litter_cont is None:
                    return 

                x, y, w, h = cv.boundingRect(litter_cont)

                cx = x + w // 2
                ex = 160 - cx

                ex *= 0.05

                self.mega.add_omni_vel(
                    flv=-ex,
                    frv=ex,
                    blv=ex,
                    brv=-ex,
                )

                self.sc_panno = cv.cvtColor(litter_mask, cv.COLOR_GRAY2BGR)
                logger.info(int(w * h))

                if time() - self.state_tmr > 9:
                    self.pdc.set_setpoint(LITTER_COURSE_GRAB_0)
                    self.__update_local_state("Push0")

                if w * h > LITTER_BOUND_EXIT and time() - self.state_tmr > LITTER_TMR_KISS:
                    self.pdc.set_setpoint(LITTER_COURSE_GRAB_0)
                    self.__update_local_state("Push0")

            case "Push0":
                self.__move_by_setpoint(speed=54)

                self.mega.add_omni_vel(
                    flv=20,
                    frv=-10,
                    blv=-10,
                    brv=20,
                )

                if time() - self.state_tmr > LITTER_TMR_PUSH:
                    self.__update_local_state("Grab0")

            case "Grab0":
                self.__move_by_setpoint(speed=62)

                self.mega.add_omni_vel(
                    flv=15,
                    frv=0,
                    blv=0,
                    brv=15,
                )

                if time() - self.state_tmr > LITTER_TMR_PUSH + 5:
                    self.pdc.set_setpoint(LITTER_COURSE_GRAB_1)
                    self.__update_local_state("Rot0")

            case "Rot0":
                self.mega.vel_range = (45, 60)
                self.__move_by_setpoint(54)

                if time() - self.state_tmr > 5:
                    self.__update_local_state("Push1")

            case "Push1":
                self.__move_by_setpoint(speed=54)

                self.mega.add_omni_vel(
                    flv=20,
                    frv=-10,
                    blv=-10,
                    brv=20,
                )

                if time() - self.state_tmr > LITTER_TMR_PUSH:
                    self.__update_local_state("Grab1")

            case "Grab1":
                self.__move_by_setpoint(speed=62)

                self.mega.add_omni_vel(
                    flv=15,
                    frv=0,
                    blv=0,
                    brv=15,
                )

                if time() - self.state_tmr > LITTER_TMR_PUSH + 5:
                    self.pdc.set_setpoint(LITTER_COURSE_GRAB_2)
                    self.__update_local_state("Rot1")

            case "Rot1":
                self.mega.vel_range = (45, 60)
                self.__move_by_setpoint(54)

                if time() - self.state_tmr > 5:
                    self.__update_local_state("Grab2")
                    self.mega.vel_range = (10, 98)

            case "Grab2":
                self.__move_by_setpoint(speed=70)

                if time() - self.state_tmr > 5:
                    self.__update_local_state("Exit")
                    self.pdc.set_setpoint(LITTER_COURSE_EXIT)

            case "Exit":
                self.__move_by_setpoint(SPEED)

                if time() - self.state_tmr > 3:
                    self.__update_state("Finish")

    def __finish(self) -> None:
        match self.local_state:
            case "Start":
                self.pdc.set_setpoint(FINISH_COURSE_HEAD)
                self.__update_local_state("Finish")

            case "Finish":
                self.__move_by_setpoint(60)

                if time() - self.state_tmr > FINISH_TMR:
                    self.__update_state("Idle")
                    print('\n\n\n')
                    logger.success("Good girl")

    def __idle(self) -> None:
        self.mega.idle()

    def __mk_hsv_mask(self, frame: MatLike, bound: tuple) -> MatLike:
        mask = cv.inRange(
            cv.cvtColor(frame, cv.COLOR_BGR2HSV),
            bound[0],
            bound[1],
        )
        return mask

    def __get_biggest_cont(self, mask: MatLike, area_bound: int | float) -> MatLike | None:
        conts = cv.findContours(
            mask,
            cv.RETR_EXTERNAL,
            cv.CHAIN_APPROX_SIMPLE,
        )[0]

        if conts is None:
            return None 

        max_cont = None, 0.0
        for cont in conts:
            cont_area = cv.contourArea(cont)
            if cont_area > max_cont[1] and cont_area > area_bound:
                max_cont = (cont, cont_area)

        return max_cont[0] 

    def __get_biggest_cont_1(self, mask: MatLike, area_bound: int | float) -> MatLike | None:
        conts = cv.findContours(
            mask,
            cv.RETR_EXTERNAL,
            cv.CHAIN_APPROX_SIMPLE,
        )[0]

        if conts is None:
            return None 

        max_cont = None, 0.0
        for cont in conts:
            cont_area = cv.contourArea(cont)
            if cont_area > max_cont[1] and cont_area > area_bound:
                x, y, w, h = cv.boundingRect(cont)
                if y < 120:
                    max_cont = (cont, cont_area)

        return max_cont[0] 

    def __move_by_setpoint(self, speed: int | float) -> None:
        self.pdc.tick(self.mega.azimuth, bypass=2)

        self.mega.set_diff_vel(
            l_vel=speed + self.pdc.U,
            r_vel=speed - self.pdc.U,
        )

    def __init__(self, mega: MEGA) -> None:
        self.mega = mega

        self.state = "Starting"
        self.state_tmr = time()
        self.local_state = "Start"

        logger.info("Cams Initting...")
        while True:
            try:
                if self.mega.sc._new_frame and self.mega.bc._new_frame:
                    break
            except:
                pass
        logger.info("Cams Initted!")

        self.sc_panno: MatLike = self.mega.sc.l
        self.bc_panno: MatLike = self.mega.bc.src
        self.telemetry = [
            ("Azi/", self.mega.azimuth),
            ("Mode/", self.mega.mode),
            ("Stt/", f"{self.state}-{self.local_state}"),
            ("Tmr/", round(float(time() - self.state_tmr)), 1),
            ("Pkg/", self.mega.tx_pkg),
        ]

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

        self.telemetry = [
            ("azi", self.mega.azimuth),
            ("mode", self.mega.mode),
            ("stt", f"{self.state}-{self.local_state}"),
            ("tmr", round(float(time() - self.state_tmr)), 3),
            ("pkg", self.mega.tx_pkg),
        ]

        match self.state:
            case "Starting":
                self.__start()
            case "Gates":
                self.__gates()
            case "Sponge":
                self.__sponge()
            case "Cube":
                self.__cube()
            case "Gates2":
                self.__2gates()
            case "Litter":
                self.__litter()
            case "Finish":
                self.__finish()
            case "Idle":
                self.__idle()

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
        self.__update_local_state("Start")

    def __update_local_state(self, new_local_state: str) -> None:
        logger.info(f"New Local State: {new_local_state}")

        self.state_tmr = time()
        self.local_state = new_local_state


if __name__ == "__main__":
    mega = MEGA()
    state_machine = StateMachine(mega)

    ser = Serial("/dev/ttyACM0", 115200)
    sleep(7.5)

    tx_tmr = time()
    received = False
    while True:
        state_machine.tick()

        if time() - tx_tmr > 0.1 or received:
            ser.write(state_machine.mega.tx_pkg.encode("utf-8"))

            tx_tmr = time()
            received = True

        rx_str = ""
        timeout_tmr = time()
        while True:
            try:
                if ser.in_waiting:
                    ret = ser.read().decode("utf-8")

                    if time() - timeout_tmr > 0.1 and 0:
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
