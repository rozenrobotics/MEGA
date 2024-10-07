import cv2 as cv

oxoy = (320, 240)
white = (255, 255, 255)

low, high = (20, 90, 60), (35, 255, 235)

frame = cv.imread("/home/vanische/CLionProjects/Boat2023/source/11-09-22 071.jpg")
frame = cv.resize(frame, (640, 480), 1)
hsvframe = cv.cvtColor(frame, cv.COLOR_BGR2HSV)


def gate(hsvframe, low, high):
    global frame
    mask = cv.inRange(hsvframe, low, high)

    sm0, sm1 = 0, 0
    cm0, cm1 = 0, 0
    cnts, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if len(cnts) > 1:
        for c in cnts:
            sc = cv.contourArea(c)
            if sc > 1000:
                if sc > sm0:
                    sm0 = sc
                    cm0 = c
                    if sm0 > sm1:
                        sm0, sm1 = sm1, sc
                        cm0, cm1 = cm1, c

        x0, y0, w0, h0 = cv.boundingRect(cm0)
        x1, y1, w1, h1 = cv.boundingRect(cm1)
        left, right = x0 + w0 // 2, x1 + w1 // 2

        cv.rectangle(frame, (x0, y0), (x0 + w0, y0 + h0), white, 2)
        cv.rectangle(frame, (x1, y1), (x1 + w1, y1 + h1), white, 2)

        if x0 < x1:
            left, right, yl, yr = left, right, y0, y1
        else:
            left, right, yl, yr = right, left, y1, y0

        cv.circle(frame, (left, yl), 5, white, -1)
        cv.circle(frame, (right, yr), 5, white, -1)

        goal = (left + right) // 2
        cv.arrowedLine(frame, (480, oxoy[1]), (goal, (yl + yr) // 2), white, 1)


# gate(hsvframe, low, high)

while 1:
    frame = cv.imread("/home/vanische/CLionProjects/Boat2023/source/11-09-22 071.jpg")
    cv.imshow("frame", frame)
