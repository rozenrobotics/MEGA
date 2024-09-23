# src/rpi.py

import math
import time

import cv2 as cv
import numpy as np
import RobotAPI
import serial

mega = serial.Serial("/dev/ttyS0", baudrate=115200, stopbits=serial.STOPBITS_ONE)

rapi, downcam, maincam = RobotAPI.RobotAPI(flag_serial=False), cv.VideoCapture(1), 1

# Set camera parameters
rapi.set_camera(100, 640, 480, 0)

# Initialize variables
fps, cntfps, timfps = 0, 0, 0
mode, zcompass, azimuth, globalcompass = 0, 0, 0, 0


cropbox, cropboxnew = ((240, 375), (0, 640)), ((120, 375), (0, 640))
xm, ym = (
    cropbox[1][0] + (cropbox[1][1] - cropbox[1][0]) // 2,
    cropbox[0][0] + (cropbox[0][1] - cropbox[0][0]) // 2,
)
gp, dgp = (xm, ym, (0, 0, 0)), (320, 240)
conrect = (0, 0)

h, s, v = 0, 0, 0
H, S, V = 180, 255, 255

hsvmarker = ((), ())
hsvyellow = ((30, 135, 0), (75, 255, 255))
hsvyellow1 = ((25, 75, 0), (80, 200, 255))

hsvorange = ((0, 30, 0), (80, 255, 255))
hsvred = ((160, 100, 0), (180, 255, 255))
hsvred1 = ((0, 0, 0), (10, 255, 255))  #
hsvred2 = ((0, 100, 0), (30, 255, 255))
hsvgreen = ((60, 180, 30), (80, 255, 255))
hsvblue = ((97, 150, 0), (120, 255, 255))
hsvpanton = ((97, 150, 0), (120, 255, 255))

hsvblack = ((0, 0, 0), (255, 255, 70))
hsvwhite = ((0, 0, 50), (255, 255, 255))

white = (255, 255, 255)
black = (0, 0, 0)
red = (0, 0, 255)
green = (0, 255, 0)
blue = (255, 0, 0)
pink = (255, 200, 255)
text = black
font = cv.FONT_HERSHEY_COMPLEX_SMALL
cropbox, cropboxnew = ((225, 375), (0, 640)), ((120, 375), (0, 640))
xm, ym = (
    cropbox[1][0] + (cropbox[1][1] - cropbox[1][0]) // 2,
    cropbox[0][0] + (cropbox[0][1] - cropbox[0][0]) // 2,
)
gp, dgp = (xm, ym, (0, 0, 0)), (320, 240)

errorold, eroldc = 0, 0
goal, dist, length, square, point = 0, 0, 0, 0, 320
modego, goal, speed, boolbomb, boolgun, megacompass = 0, 0, 0, 0, 0, zcompass

state, tim, fllocal, cnt = 0, 0, 0, 0
flx, fly = 0, 0
errorx, errory = 0, 0
flnocnt = 0
tim0, tim1, tim2, tim3 = 0, 0, 0, 0
flagstate2, flagstate3 = False, False
cntpcg = 0


def pd(error, coef=0.5, sp=50, d=5):
    """
    Proportional-Derivative (PD) controller for error correction.

    Args:
        error (float): The error value.

    Returns:
        int: The control signal value.
    """
    global errorold
    u = error * coef + (error - errorold) * coef * d

    if u < -sp:
        u = -sp
    elif u > sp:
        u = sp

    errorold = error
    return int(u)


def pd_compass(course, mode=1):
    """
    Proportional-Derivative (PD) controller for compass error correction.

    Args:
        course (float): The desired course.
        mode (int, optional): The mode of operation. Defaults to 1.

    Returns:
        int: The control signal value.
    """
    global eroldc
    if course < 0:
        course = 360 + course
    if course > 360:
        course -= 360

    error = course - azimuth
    if course < azimuth:
        error += 360
    elif course == azimuth:
        error = 0
    if error > 180:
        error = -(360 - error)

    eroldc = error
    if mode:
        return pd(error, 0.2, 50, 2)
    else:
        return int(error)


def gates(sensor, low, high, lim, graphics=0):
    """
    Detects gates in the image using color ranges.

    Args:
        sensor (numpy.ndarray): The input image from the camera.
        low (tuple): The lower color range for gate detection.
        high (tuple): The upper color range for gate detection.
        graphics (int, optional): Flag to enable graphics visualization. Defaults to 0.

    Returns:
        numpy.ndarray: The mask image with detected gates.
    """
    global gp, goal, dist, square, length
    mask = cv.inRange(cv.cvtColor(sensor, cv.COLOR_BGR2HSV), low, high)
    cv.blur(mask, (5, 5), mask)

    cntcnts, cm0, sm0, cm1, sm1, lim = 0, 0, 0, 0, 0, 300
    for c in cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[0]:
        sc = cv.contourArea(c)
        if sc > lim:
            cntcnts, cm0, sm0 = cntcnts + 1, c, sc
            if sm0 > sm1:
                cm0, sm0, cm1, sm1 = cm1, sm1, c, sc

    square = sm0 + sm1
    if cntcnts >= 2:
        x0, y0, w0, h0 = cv.boundingRect(cm0)
        x1, y1, w1, h1 = cv.boundingRect(cm1)

        x0, x1 = x0 + w0 // 2, x1 + w1 // 2
        length = abs(x0 - x1)
        gp = (
            cropbox[1][0] + (x0 + x1) // 2,
            cropbox[0][0] + (((y0 + y1) // 2) + ((h0 + h1) // 2) // 2),
        )

    elif cntcnts == 1:
        x1, y1, w1, h1 = cv.boundingRect(cm1)
        x1 = x1 + w1 // 2

        if x1 < 320:
            gp = (cropbox[1][1], ym)
        else:
            gp = (cropbox[1][0], ym)

    else:
        flnocnt = 1

    if graphics:
        global frame
        frame[cropbox[0][0] : cropbox[0][1], cropbox[1][0] : cropbox[1][1]] = (
            cv.cvtColor(mask, cv.COLOR_GRAY2BGR)
        )

        if cntcnts > 1:
            cv.circle(
                frame, (cropbox[1][0] + x0, cropbox[0][0] + y0 + h0 // 2), 3, green, -1
            )
            cv.circle(
                frame, (cropbox[1][0] + x1, cropbox[0][0] + y1 + h1 // 2), 3, green, -1
            )


def ball(sensor, low, high, lim, graphics=0):
    """
    Detects a ball in the image using color ranges.

    Args:
        sensor (numpy.ndarray): The input image from the camera.
        low (tuple): The lower color range for ball detection.
        high (tuple): The upper color range for ball detection.
        graphics (int, optional): Flag to enable graphics visualization. Defaults to 0.

    Returns:
        None
    """
    global gp, goal, dist, square, conrect, flnocnt
    mask = cv.inRange(cv.cvtColor(sensor, cv.COLOR_BGR2HSV), low, high)
    cv.blur(mask, (5, 5), mask)

    cntcnts, cm0, sm0, lim = 0, 0, 0, 175
    for c in cv.findContours(mask, cv.RETR_CCOMP, cv.CHAIN_APPROX_SIMPLE)[0]:
        sc = cv.contourArea(c)
        x0, y0, w0, h0 = cv.boundingRect(c)
        if x0 > 100 and x0 + w0 < 540:
            if sc > sm0 and sc > lim:
                cntcnts, cm0, sm0 = cntcnts + 1, c, sc

    square = sm0
    if cntcnts > 0:
        flnocnt = 0
        x0, y0, w0, h0 = cv.boundingRect(cm0)
        conrect = (w0, h0)
        gp, dist = (
            cropbox[1][0] + x0 + w0 // 2,
            cropbox[0][0] + y0 + h0 // 2,
        ), cropbox[0][1] - (cropbox[0][0] + y0 + h0)

    else:
        flnocnt = 1

    if graphics:
        global frame
        frame[cropbox[0][0] : cropbox[0][1], cropbox[1][0] : cropbox[1][1]] = (
            cv.cvtColor(mask, cv.COLOR_GRAY2BGR)
        )
        cv.circle(frame, (gp[0], gp[1]), 5, green, -1)


def basket(sensot, low, high, lim, graphics=0):
    global gp, goal, dist, square, conrect
    mask = cv.inRange(cv.cvtColor(sensor, cv.COLOR_BGR2HSV), low, high)
    cv.blur(mask, (5, 5), mask)

    cntcnts, cm0, sm0, lim = 0, 0, 0, 175
    for c in cv.findContours(mask, cv.RETR_CCOMP, cv.CHAIN_APPROX_SIMPLE)[0]:
        sc = cv.contourArea(c)
        x0, y0, w0, h0 = cv.boundingRect(c)
        if x0 > 100 and x0 + w0 < 540:
            if sc > sm0 and sc > lim:
                cntcnts, cm0, sm0 = cntcnts + 1, c, sc

    square = sm0
    if cntcnts > 0:
        x0, y0, w0, h0 = cv.boundingRect(cm0)
        conrect = (w0, h0)
        gp, dist = (
            cropbox[1][0] + x0 + w0 // 2,
            cropbox[0][0] + y0 + h0 // 2,
        ), cropbox[0][1] - (cropbox[0][0] + y0 + h0)

    else:
        print("no contours")

    if graphics:
        global frame
        frame[cropbox[0][0] : cropbox[0][1], cropbox[1][0] : cropbox[1][1]] = (
            cv.cvtColor(mask, cv.COLOR_GRAY2BGR)
        )
        cv.circle(frame, (gp[0], gp[1]), 5, green, -1)


def marker(sensor, low, high, graphics=0):
    global gp, dgp, goal, dist, square
    mask = cv.inRange(sensor, low, high)
    cv.blur(mask, (5, 5), mask)

    cntcnts, cm0, sm0, lim = 0, 0, 0, 5000
    for c in cv.findContours(mask, cv.RETR_CCOMP, cv.CHAIN_APPROX_SIMPLE)[0]:
        sc = cv.contourArea(c)
        if sc > sm0 and sc > lim:
            cntcnts, cm0, sm0 = cntcnts + 1, c, sc

    square = sm0
    if cntcnts > 0:
        x0, y0, w0, h0 = cv.boundingRect(cm0)
        gp = (x0 + w0 // 2, y0 + h0 // 2)
        dgp = (x0 + w0 // 2, y0 + h0 // 2)
        dist = int(math.hypot(gp[0] - 320, gp[1] - 240))

    else:
        print("no contours")

    if graphics:
        global frame
        frame = cv.cvtColor(mask, cv.COLOR_GRAY2BGR)
        if cntcnts > 0:
            cv.rectangle(frame, (x0, y0), (x0 + w0, y0 + h0), green, 2)
        # cv.circle(frame, (gp[0], gp[1]), 5, green, -1)


def telemetry(image):
    """
    Displays telemetry information on the image.

    Args:
        image (numpy.ndarray): The input image.

    Returns:
        None
    """
    global mode, fps, cntfps, timfps
    image[0:10, 290:351], image[10:20, 300:340] = white, white  # browbox
    cv.circle(image, (300, 10), 10, white, -1)
    cv.circle(image, (340, 10), 10, white, -1)
    image[460:480, 0:640] = (239, 239, 239)  # downbox

    # goal point
    if maincam:
        cv.arrowedLine(
            image,
            (cropbox[1][1] // 2, cropbox[0][1]),
            (gp[0], gp[1]),
            pink,
            2,
            tipLength=0.025,
        )
    else:
        cv.arrowedLine(
            image,
            (cropbox[1][1] // 2, cropbox[0][1]),
            (dgp[0], dgp[1]),
            pink,
            2,
            tipLength=0.025,
        )

    # compass
    strcompass = ("", 0)
    if azimuth > 99:
        strcompass = (str(azimuth), 300)
    elif azimuth > 9:
        strcompass = (str(azimuth), 308)
    else:
        strcompass = (str(azimuth), 314)
    cv.putText(image, strcompass[0], (strcompass[1], 15), font, 1, text, 1)

    # mode
    strmode = ("", 0)
    if mode == 0:
        strmode = ("neutral", 270)
    elif mode == 1:
        strmode = ("manual", 270)
    elif mode == 2:
        strmode = ("autonomous", 248)
    cv.putText(image, strmode[0], (strmode[1], 475), font, 1, text, 1)

    # fps
    cntfps, tim = cntfps + 1, time.time()  # fps
    if tim > timfps + 1:
        fps, cntfps, timfps = cntfps, 0, tim
    cv.putText(image, str(fps), (610, 475), font, 1, text, 1)

    # variables printing
    cv.putText(image, str(dist), (20, 475), font, 1, text, 1)
    cv.putText(image, str(int(square)), (100, 475), font, 1, text, 1)
    cv.putText(image, str(state), (200, 475), font, 1, text, 1)
    cv.putText(image, str(fllocal), (410, 475), font, 1, text, 1)
    cv.putText(image, str(int(tim)), (430, 475), font, 1, text, 1)
    frame[0:30, 0:220] = white
    cv.putText(
        image, ("X: " + str(gp[0]) + "   Y: " + str(gp[1])), (20, 20), font, 1, black, 1
    )

    rapi.set_frame(image, 40)


def uart2mega():
    """
    Sends commands to the microcontroller and receives responses.

    Args:
        None

    Returns:
        None
    """
    global mode, azimuth, cntpcg

    # package = str(modego) + str(goal + 200) + str(speed + 200) + str(boolgun) + str(boolbomb) + str(megacompass + 200) + "\n"
    # package = "4321\n"

    if cntpcg == 0:
        package = str(modego) + str(boolgun) + str(boolbomb)
        cntpcg = 1
    elif cntpcg == 1:
        package = str(goal + 200)
        cntpcg = 2
    elif cntpcg == 2:
        package = str(speed + 200)
        cntpcg = 3
    elif cntpcg == 3:
        package = str(megacompass + 200) + "$"
        cntpcg = 0

    mega.write(package.encode("utf-8"))
    # print(package)
    try:
        if mega.in_waiting > 0:
            strmega, t = "", time.time()

            while True:
                c = str(mega.read(), "utf-8")
                if c != "$":
                    strmega += c
                else:
                    break
                if t + 0.04 < time.time():
                    break

            if len(strmega) == 4:
                mode, azimuth = int(strmega[0]), int(strmega[1:4]) - 200
            else:
                mode, azimuth = 0, 0
            mega.reset_input_buffer()

    except ValueError:
        print("err")


def hsvcolor():
    """
    Adjusts the HSV color ranges interactively.

    Args:
        None

    Returns:
        None
    """
    global frame, h, s, v, H, S, V

    key = rapi.get_key()

    if key == 24:
        h += 1
    elif key == 38:
        h -= 1
    elif key == 25:
        s += 1
    elif key == 39:
        s -= 1
    elif key == 26:
        v += 1
    elif key == 40:
        v -= 1
    elif key == 27:
        H += 1
    elif key == 41:
        H -= 1
    elif key == 28:
        S += 1
    elif key == 42:
        S -= 1
    elif key == 29:
        V += 1
    elif key == 43:
        V -= 1
    if h < 0:
        h = 0
    elif h > 180:
        h = 180
    if s < 0:
        s = 0
    elif s > 255:
        s = 255
    if v < 0:
        v = 0
    elif v > 255:
        v = 255
    if H < 0:
        H = 0
    elif H > 255:
        H = 255
    if S < 0:
        S = 0
    elif S > 255:
        S = 255
    if V < 0:
        V = 0
    elif V > 255:
        V = 255

    if key == 52:
        h, s, v = 0, 0, 0
        H, S, V = 180, 255, 255
    color = ((h, s, v), (H, S, V))
    print(color)

    # frame = cv.bitwise_and(frame, cv.cvtColor(cv.inRange(frame, color[0], color[1]), cv.COLOR_GRAY2BGR))
    frame = cv.bitwise_or(
        frame,
        frame,
        mask=cv.inRange(cv.cvtColor(frame, cv.COLOR_BGR2HSV), color[0], color[1]),
    )


def xy2angle(x, y):
    angle, x, y = 0, x - 320, y - 240

    if y > 0 <= x:
        angle = (
            np.arctan(float(abs(x) / abs(y))) * 180 / math.pi
            if abs(x) <= abs(y)
            else 90 - np.arctan(float(abs(y) / abs(x))) * 180 / math.pi
        )
    elif y <= 0 < x:
        angle = (
            90 + np.arctan(float(abs(y) / abs(x))) * 180 / math.pi
            if abs(y) <= abs(x)
            else 180 - np.arctan(float(abs(x) / abs(y))) * 180 / math.pi
        )
    elif y < 0 >= x:
        angle = (
            180 + np.arctan(float(abs(x) / abs(y))) * 180 / math.pi
            if abs(x) <= abs(y)
            else 270 - np.arctan(float(abs(y) / abs(x))) * 180 / math.pi
        )
    elif y >= 0 > x:
        angle = (
            270 + np.arctan(float(abs(y) / abs(x))) * 180 / math.pi
            if abs(y) <= abs(x)
            else 360 - np.arctan(float(abs(x) / abs(y))) * 180 / math.pi
        )

    return abs(int(angle))


def arrow(lim0, lim1, graphics=0):
    global gp, goal, dist, square, conrect, flnocnt
    maskwhite = cv.inRange(frame[160:320, 0:640], hsvwhite[0], hsvwhite[1])
    cv.blur(maskwhite, (3, 3), maskwhite)

    cntswhite = cv.findContours(maskwhite, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)[0]
    maskwhite = cv.cvtColor(maskwhite, cv.COLOR_GRAY2BGR)
    # for c in cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[0]:

    x, y, w, h = 0, 0, 0, 0
    sm = 0
    cm = 0
    cntcnts = 0
    flnocnt = 1
    for c in cntswhite:
        x0, y0, w0, h0 = cv.boundingRect(c)
        sc = cv.contourArea(c)

        if sc > lim0 and sc > 0.7 * h0 * w0:
            cv.rectangle(maskwhite, (x0, y0), (x0 + w0, y0 + h0), (255, 100, 255), 2)

            crop = frame[160 + y0 : 160 + y0 + h0, x0 : x0 + w0]
            maskblack = cv.blur(cv.inRange(crop, hsvblack[0], hsvblack[1]), (5, 5))
            cntsblack = cv.findContours(
                maskblack, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE
            )[0]

            maskblack = cv.cvtColor(maskblack, cv.COLOR_GRAY2BGR)
            for cc in cntsblack:
                if cv.contourArea(cc) > lim1:
                    x1, y1, w1, h1 = cv.boundingRect(cc)
                    if x1 > 2 and x1 + w1 < w0 - 2:
                        if h0 * w0 > sm:
                            x, y, w, h = x0, y0, w0, h0
                            sm = h0 * w0
                            cntcnts += 1
                            flnocnt = 0

    # square = sm0
    if cntcnts > 0:
        conrect = (w, h)
        gp, dist = (cropbox[1][0] + x + w // 2, cropbox[0][0] + y + h // 2), cropbox[0][
            1
        ] - (cropbox[0][0] + y + h)

    if graphics:
        cv.rectangle(maskwhite, (x, y), (x + w, y + h), (0, 255, 0), 2)
        frame[160:320, 0:640] = maskwhite


if __name__ == "__main__":
    hsvmarker = hsvblue  # 8 - blue : 9 - green
    flgreen = 0
    ballmarkers = [hsvyellow, hsvgreen, hsvred]
    maincam = 1
    state = 0
    compass = [325, 15, 325, 93]  # gates : orange : backet : parking

    time_back_after_bouy = 12
    dist2marker_after_gates = 120
    square2ball = 2000
    time2bouy = 4
    protectballs = 20
    fllocal = 0

    for i in range(30):
        frame = rapi.get_frame(wait_new_frame=1)
    while True:
        "The main loop that controls the robot's behavior based on the current state."
        frame = rapi.get_frame(wait_new_frame=1) if maincam else downcam.read()[1]
        modego, gp, dgp, goal, speed, sensor = (
            0,
            (xm, ym),
            (320, 240),
            0,
            0,
            frame[cropbox[0][0] : cropbox[0][1], cropbox[1][0] : cropbox[1][1]],
        )
        dist = 1000
        flnocnt = 0

        if state == "hsv":
            hsvcolor()
        elif mode == 2:

            if state == -1:
                goal = 180
                # goal = pd_compass(315)
                speed = 50
                modego = 1
                megacompass = 315

            # go between gates by compass for marker by square
            if state == 0:
                ball(sensor, hsvmarker[0], hsvmarker[1], 250, 1)
                # goal, speed = pd_compass(325), 50

                modego, goal, speed = 1, 0, 30
                megacompass = 325

                # print(goal, azimuth)

                if fllocal == 1:
                    if tim + 20 < time.time():  # 20
                        # cnt += 1
                        # if cnt > 5:
                        #     state, cnt = 1, 0
                        state = 4  # 4
                        fllocal = 0
                        tim = time.time()

                else:
                    fllocal = 1
                    tim = time.time()

            # align to marker by distance first wave
            elif state == 1:
                ball(sensor, hsvgreen[0], hsvgreen[1], 500, 1)

                if fllocal == 0:
                    modego, goal, speed, megacompass = 1, 0, 30, 325

                    if square > 1500 and tim + 5 < time.time():
                        cnt += 1
                        if cnt > 5:
                            fllocal, cnt = 1, 0
                    else:
                        cnt = 0

                elif fllocal == 1:
                    error = 320 - gp[0]
                    modego, goal, speed, megacompass = 1, 0, 30, 15

                    if error < -100:
                        goal = 90
                    elif error > 100:
                        goal = 270
                    else:
                        goal = 0

                    if square > square2ball:
                        speed = 30

                    if gp[1] > 250:
                        tim, fllocal = time.time(), 2
                        # time.sleep(1.5)

                elif fllocal == 2:  # right
                    modego, goal, speed = 1, 0, 30
                    megacompass = 15 if flgreen else 235

                    if tim + 5 < time.time():
                        tim, fllocal = time.time(), 3

                elif fllocal == 3:  # front
                    modego, goal, speed = 1, 0, 30
                    megacompass = 325

                    if tim + 7 < time.time():
                        tim, fllocal = time.time(), 4

                elif fllocal == 4:  # left
                    modego, goal, speed = 1, 0, 30
                    megacompass = 235 if flgreen else 15

                    if tim + 8 < time.time():
                        tim, fllocal = time.time(), 5

                elif fllocal == 5:  # back
                    modego, goal, speed = 1, 0, 30
                    megacompass = 90

                    if tim + 7 < time.time():
                        tim, fllocal = time.time(), 6

                elif fllocal == 6:  # right
                    modego, goal, speed = 1, 0, 30
                    megacompass = 15 if flgreen else 235

                    if tim + 5 < time.time():
                        tim, fllocal = time.time(), 7
                        # state = 4

                elif fllocal == 7:
                    modego, goal, speed = 1, 0, 30
                    megacompass = 90

                    if tim + 20 < time.time():
                        tim, fllocal = time.time(), 0
                        state = 5

            # return gates
            elif state == 4:
                if fllocal == 0:
                    modego, goal, speed = 1, 180, 30
                    megacompass = 330

                    if tim + 15 < time.time():
                        tim, fllocal = time.time(), 0
                        maincam = 0
                        state = 5

                elif fllocal == 1:
                    gates(sensor, hsvyellow[0], hsvyellow[1], 400, 1)
                    modego, goal, speed = 0, pd(gp[0] - 320, 0.25), 50

                    if flnocnt:
                        goal = pd_compass(90)

                    if length > 400 and square > 4000:
                        tim, fllocal = time.time(), 2
                    # fllocal = 1
                    # tim = time.time()
                    # globalcompass = azimuth

                elif fllocal == 2:
                    modego, goal, speed = 0, pd_compass(90), 30
                    maincam = 0
                    if tim + 4 < time.time():
                        state, tim, fllocal = 5, time.time(), 0
                        # state = 5
                        # fllocal = 0
                        # tim = time.time()

            # go forward before orange circle
            elif state == 5:
                if fllocal == 0:
                    cropbox = ((20, 240), (0, 640))
                    marker(
                        cv.cvtColor(frame, cv.COLOR_BGR2HSV),
                        hsvorange[0],
                        hsvorange[1],
                        1,
                    )

                    modego, goal, speed = 1, 0, 30
                    megacompass = compass[1]

                    if square > 10000:
                        cnt += 1
                        if cnt > 10:
                            state, cnt = 6, 0
                    else:
                        cnt = 0

            # align to down marker and bombing
            elif state == 6:
                # cropbox = ((20, 240), (0, 640))
                marker(
                    cv.cvtColor(frame, cv.COLOR_BGR2HSV), hsvorange[0], hsvorange[1], 1
                )
                angle = 360 - xy2angle(dgp[0], dgp[1])

                modego, goal, speed, megacompass = (
                    1,
                    angle,
                    10 if dist < 100 else 30,
                    compass[1],
                )

                if speed > 30:
                    speed = 30
                if speed < -30:
                    speed = -30

                if dist < 50:
                    goal, speed = 0, 5

                if dist < 40:
                    cnt += 1
                    if cnt > 10:
                        cv.circle(frame, (xm, ym), 100, red, -1)
                        state = 11  # state = 7
                        tim, fllocal = time.time(), 0
                        maincam, boolbomb = 1, 1
                        cropbox = ((225, 375), (0, 640))
                        time.sleep(1.5)

                else:
                    cnt = 0

            # first ball
            elif state == 7:
                ball(sensor, ballmarkers[0][0], ballmarkers[0][1], 500, 1)

                if tim + protectballs < time.time():
                    state = 8

                if fllocal == 0:
                    modego, goal, speed, megacompass = 1, 0, 30, 15

                    if square > 1500 and tim + 4 < time.time():
                        cnt += 1
                        if cnt > 5:
                            fllocal, cnt = 1, 0
                            tim = time.time()

                    else:
                        cnt = 0

                elif fllocal == 1:
                    error = 320 - gp[0]
                    modego, goal, speed, megacompass = 1, 0, 30, 15

                    if error < -100:
                        goal = 90
                    elif error > 100:
                        goal = 270
                    else:
                        goal = 0

                    if gp[1] > 285 and tim + time2bouy < time.time():
                        tim, fllocal = time.time(), 2

                elif fllocal == 2:
                    modego, goal, speed = 1, 0, 30
                    megacompass = 15

                    if tim + 2 < time.time():
                        tim, fllocal = time.time(), 3

                elif fllocal == 3:
                    modego, goal, speed = 1, 180, 30
                    megacompass = 15

                    if tim + time_back_after_bouy < time.time():
                        tim, fllocal, state = time.time(), 0, 8

            # second ball
            elif state == 8:
                ball(sensor, ballmarkers[1][0], ballmarkers[1][1], 100, 1)
                error = 320 - gp[0]

                if tim + protectballs < time.time():
                    state = 8

                if fllocal == 0:
                    modego, goal, speed, megacompass = 1, 265, 30, 15

                    if abs(error) < 40 and not flnocnt:
                        cnt += 1
                        if cnt > 5:
                            fllocal, cnt = 1, 0
                            tim = time.time()

                    else:
                        cnt = 0

                elif fllocal == 1:
                    modego, goal, speed, megacompass = 1, 0, 30, 15

                    if error < -40:
                        goal = 90
                    elif error > 40:
                        goal = 270
                    else:
                        goal = 0

                    if gp[1] > 285 and tim + time2bouy < time.time():
                        speed = 100
                        tim, fllocal = time.time(), 2

                elif fllocal == 2:
                    modego, goal, speed = 1, 0, 30
                    megacompass = 15

                    if tim + 2 < time.time():
                        tim, fllocal = time.time(), 3

                elif fllocal == 3:
                    modego, goal, speed = 1, 180, 30
                    megacompass = 15

                    if tim + time_back_after_bouy < time.time():
                        tim, fllocal, state = time.time(), 0, 9

            # third ball
            elif state == 9:
                ball(sensor, ballmarkers[2][0], ballmarkers[2][1], 500, 1)
                error = 320 - gp[0]

                if tim + protectballs < time.time():
                    state = 8

                if fllocal == 0:
                    modego, goal, speed, megacompass = 1, 265, 30, 15

                    if abs(error) < 40 and not flnocnt:
                        cnt += 1
                        if cnt > 5:
                            fllocal, cnt = 1, 0
                            tim = time.time()
                    else:
                        cnt = 0

                elif fllocal == 1:
                    modego, goal, speed, megacompass = 1, 0, 30, 15

                    if error < -40:
                        goal = 90
                    elif error > 40:
                        goal = 270
                    else:
                        goal = 0

                    if gp[1] > 285 and tim + time2bouy < time.time():
                        speed = 100
                        tim, fllocal = time.time(), 2

                elif fllocal == 2:
                    modego, goal, speed = 1, 0, 30
                    megacompass = 15

                    if tim + 2 < time.time():
                        tim, fllocal = time.time(), 3

                elif fllocal == 3:
                    modego, goal, speed = 1, 180, 30
                    megacompass = 15

                    if tim + time_back_after_bouy - 2 < time.time():
                        tim, fllocal, state = time.time(), 0, 20

            # return to first bouy
            elif state == 10:
                ball(sensor, hsvyellow[0], hsvyellow[1], 200, 1)
                error = 320 - gp[0]

                if fllocal == 0:
                    modego, goal, speed, megacompass = 1, 120, 30, compass[1]

                    if abs(error) < 40:
                        cnt += 1
                        if cnt > 5:
                            fllocal, cnt = 1, 0
                    else:
                        cnt = 0

                elif fllocal == 1:
                    modego, goal, speed = 0, pd_compass(compass[1]), -50

                    if tim + 20 < time.time():
                        speed = 10
                        if tim + 25 < time.time():
                            tim, fllocal = time.time(), 2

                elif fllocal == 2:
                    modego, goal, speed = 0, pd_compass(15), 0

                    if tim + 15 < time.time():
                        tim, fllocal, state = time.time(), 0, 11

            # basket
            elif state == 11:
                # basket(sensor, hsvblue[0], hsvblue[1], 200, 1)

                if fllocal == 0:
                    modego, goal, speed, megacompass = 1, 0, 30, 330

                    if tim + 15 < time.time():  # to it
                        speed = 0
                        boolgun = 1
                        time.sleep(1.5)
                        tim, fllocal = time.time(), 2

                elif fllocal == 2:  # back
                    modego, goal, speed = 1, 180, 30
                    megacompass = 330

                    if tim + 10 < time.time():
                        speed = 5
                        if tim + 15 < time.time():
                            tim, fllocal, state = time.time(), 0, 7

            # parking
            elif state == 20:
                # ball(sensor, hsvorange[0], hsvorange[1], 100, 1)
                arrow(200, 100, 1)

                modego, goal, speed, megacompass = 1, 0, 30, compass[3]

                errorx = xm - gp[0]

                if fllocal == 0:
                    if conrect[0] * conrect[1] > 1500:
                        cnt += 1
                        if cnt > 5:
                            fllocal, cnt = 1, 0
                    else:
                        cnt = 0

                elif fllocal == 1:
                    if errorx < -40:
                        goal = 90
                    elif errorx > 40:
                        goal = 270

                    else:
                        goal = 0

                    speed = 50
                    if gp[1] > 265 and not flnocnt and tim + 25 < time.time():
                        state = 21
                        tim = time.time()

            # finish
            elif state == 21:
                modego, goal, speed, megacompass = 1, 0, 30, compass[3]
                if tim + 10 < time.time():
                    speed = 0
                    print("finish")
                # maincam, modego, goal, speed, megacompass = 1, 1, 0, 10, 315

            print(fllocal, *gp)
        else:
            tim = time.time()

        # goal, speed, modego = 0, 0, 0

        uart2mega()
        telemetry(frame)
