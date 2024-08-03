import cv2 
import numpy as np


class Color: ...
class ColorRange: ...
class Frame: ...
class Contour: ...
class Rect: ... 
class ROI: ...
class Point: ...
class Moments: ...


class Colors:
    RED    = (0,   0,   255)
    BLUE   = (255, 0,   0  )
    CYAN   = (255, 255, 0  )
    WHITE  = (255, 255, 255)
    BLACK  = (0,   0,   0  )
    GREEN  = (0,   255, 0  )
    ORANGE = (0,   165, 255)
    PURPLE = (255, 0,   255)
    YELLOW = (255,   0, 100)

class ColorRange:
    def __init__(
            self,
            lower: tuple[int, int, int],
            upper: tuple[int, int, int],
        ) -> None:

        self.lower = np.array([
            lower[0],
            lower[1],
            lower[2],
        ])
        self.upper = np.array([
            upper[0],
            upper[1],
            upper[2],
        ])


class Frame:
    def __init__(self, src, colorspace: str):
        self.src = src
        self.colorspace = colorspace

        self.h = self.src.shape[0]
        self.w = self.src.shape[1]

    def show(self, winname: str = 'frame') -> Frame: 
        cv2.imshow(winname, self.src)
        return self

    @staticmethod
    def create_hsv_tb_window(winname: str = 'hsv trackbars') -> None:
        cv2.namedWindow(winname)
        cv2.createTrackbar('h lower', winname, 0, 180, lambda x: None)
        cv2.createTrackbar('s lower', winname, 0, 255, lambda x: None)
        cv2.createTrackbar('v lower', winname, 0, 255, lambda x: None)
        cv2.createTrackbar('h upper', winname, 0, 180, lambda x: None)
        cv2.createTrackbar('s upper', winname, 0, 255, lambda x: None)
        cv2.createTrackbar('v upper', winname, 0, 255, lambda x: None)

        cv2.setTrackbarPos('h lower', winname, 0)
        cv2.setTrackbarPos('s lower', winname, 0)
        cv2.setTrackbarPos('v lower', winname, 0)
        cv2.setTrackbarPos('h upper', winname, 180)
        cv2.setTrackbarPos('s upper', winname, 255)
        cv2.setTrackbarPos('v upper', winname, 255)

    def hsv_tb_mask(
            self,
            src: Frame,
            winname: str = 'hsv trackbars',
            mask_color: Color = None
        ) -> Frame:

        self.hsv_color_range = ColorRange(
            (
                cv2.getTrackbarPos('h lower', winname),
                cv2.getTrackbarPos('s lower', winname),
                cv2.getTrackbarPos('v lower', winname),
            ),
            (
                cv2.getTrackbarPos('h upper', winname),
                cv2.getTrackbarPos('s upper', winname),
                cv2.getTrackbarPos('v upper', winname),
            )
        )
        hsv_mask = src.in_range(self.hsv_color_range)
        bgr_hsv_mask = hsv_mask.cvt2bgr()
        self.hsv_mask = hsv_mask.src

        if mask_color:
            ret = self.bitwise(Frame(255 - bgr_hsv_mask.src, 'bgr'))
            ret.src[self.hsv_mask != 0] = Colors.PURPLE

            return ret
        
        return self.bitwise(bgr_hsv_mask)
            
    def create_thresh_tb_window(
            winname: str = 'thresh trackbar',
            default_value: int = 0
        ) -> None:

        cv2.namedWindow(winname)
        cv2.createTrackbar('thresh', winname, 0, 255, lambda x: None)
        cv2.setTrackbarPos('thresh', winname, default_value)

    def thresh_tb_mask(
            self,
            src: Frame,
            winname: str = 'thresh trackbar',
            mask_color: Color = None,
            invert=False
        ) -> Frame:

        self.thresh_value = cv2.getTrackbarPos('thresh', winname)

        thresh_mask = src.threshold(self.thresh_value, invert=invert)
        bgr_thresh_mask = thresh_mask.cvt2bgr()
        self.thresh_mask = thresh_mask.src

        if mask_color is not None:
            ret = self.bitwise(Frame(255 - bgr_thresh_mask.src, 'bgr'))
            ret.src[self.thresh_mask != 0] = mask_color

            return ret, thresh_mask
        return self.bitwise(bgr_thresh_mask), thresh_mask

    def resize(self, width: int, heigt: int) -> Frame:
        return Frame(cv2.resize(self.src, (width, heigt)), self.colorspace)

    def bitwise(self, mask: Frame) -> Frame:
        return Frame(cv2.bitwise_and(self.src, mask.src), self.colorspace)

    def erode(self, ksize: int, iterations: int = 1) -> Frame:
        self.src = cv2.erode(self.src, (ksize, ksize), iterations=iterations)
        return self

    def dilate(self, ksize: int = 3, iterations: int = 1) -> Frame:
        self.src = cv2.dilate(self.src, (ksize, ksize), iterations=iterations)
        return self

    def cvt2bgr(self) -> Frame:
        self.bgr = cv2.cvtColor(self.src, self.get_cvt_code('bgr'))
        return Frame(self.bgr, 'bgr')

    def cvt2hsv(self) -> Frame:
        self.hsv = cv2.cvtColor(self.src, self.get_cvt_code('hsv'))
        return Frame(self.hsv, 'hsv')

    def cvt2gray(self) -> Frame:
        self.gray = cv2.cvtColor(self.src, self.get_cvt_code('gray'))
        return Frame(self.gray, 'gray')

    def get_cvt_code(self, dst_colorspace: str) -> int:
        assert self.colorspace != dst_colorspace, "Can't convert: types equal"

        if self.colorspace == 'bgr':
            if dst_colorspace == 'hsv':
                return cv2.COLOR_BGR2HSV
            elif dst_colorspace == 'gray':
                return cv2.COLOR_BGR2GRAY
            
        elif self.colorspace == 'hsv':
            if dst_colorspace == 'bgr':
                return cv2.COLOR_HSV2BGR
            
        elif self.colorspace == 'gray':
            if dst_colorspace == 'bgr':
                return cv2.COLOR_GRAY2BGR

    def blur(self, ksize: int) -> Frame:
        self.blur = cv2.blur(self.src, (ksize, ksize))
        return Frame(self.blur, self.colorspace)
    
    def threshold(self, value: int, invert: bool = False) -> Frame:
        self.thresh = cv2.threshold(self.src, value, 255, cv2.THRESH_BINARY)[1]
        return Frame(self.thresh if not invert else 255 - self.thresh, 'gray')

    def in_range(self, color_range: ColorRange) -> Frame:
        self.ranged = cv2.inRange(self.src, 
                                  color_range.lower,
                                  color_range.upper,
        )
        return Frame(self.ranged, 'gray')

    def canny(self, value: int) -> Frame:
        self.canny = cv2.Canny(self.src, value, 200)
        return Frame(self.canny, 'gray')

    def get_conts(self) -> Frame:
        self.conts, self.hierarchy = cv2.findContours(
            self.src,
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        return self

    def roi(self, roi: ROI) -> Frame:
        return Frame(self.src[roi.y1:roi.y2, roi.x1:roi.x2], self.colorspace)

    def put_roi(self, src: Frame, roi: ROI) -> Frame:
        if src.colorspace == 'gray' and self.colorspace != 'gray':
            src = src.cvt2bgr()

        if self.colorspace == 'gray' and src.colorspace != 'gray':
            src = src.cvt2gray()

        self.src[roi.y1:roi.y2, roi.x1:roi.x2] = src.src
        return self

    def invert(self) -> Frame:
        self.src = 255 - self.src
        return self

    def print(
            self,
            text,
            point: Point,
            color: Color = Colors.WHITE,
            thickness: int = 1,
            scale: float = 0.5
        ) -> Frame:

        cv2.putText(
            self.src,
            str(text),
            (point.x, point.y),
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            color,
            thickness,
            cv2.LINE_AA,
        )
        return self

    def draw_point(
            self,
            point: Point = Point(),
            color: Color = Colors.RED
        ) -> Frame:

        cv2.circle(self.src, (point.x, point.y), 4, color, -1)
        return self

    def draw_rect(
            self,
            rect: Rect,
            color: Color = Colors.PURPLE,
            thickness: int = 1
        ) -> Frame:

        cv2.rectangle(self.src, (rect.x, rect.y), (rect.x+rect.w, rect.y+rect.h), color, thickness)
        return self

    def draw_circle(
            self,
            point: Point,
            color: Color = Colors.PURPLE,
            radius: int = 10,
            thickness: int = 1
        ) -> Frame:

        cv2.circle(self.src, (point.x, point.y), radius, color, thickness)

    def draw_cont_rect(
            self,
            cont: Contour,
            color: Color = Colors.PURPLE,
            thickness: int = 1
        ) -> Frame:

        cv2.rectangle(self.src, (cont.x, cont.y), (cont.x+cont.w, cont.y+cont.h), color, thickness)

    def draw_conts(
            self,
            cont: Contour,
            color: Color = Colors.PURPLE,
            thickness: int = 1
        ) -> Frame:

        cv2.drawContours(self.src, cont.cont, -1, color, thickness)
        return self

    def draw_approxed_contour_point(
            self,
            cont: Contour,
            color: Color = Colors.PURPLE,
            thickness: int = 1
        ) -> Frame:

        cv2.drawContours(self.src, cont.approxed_cont, -1, color, thickness)
        return self


    def draw_conts_boxes(self, conts: list = None) -> Frame:
        if conts is None:
            if self.conts is not None:
                for cont in self.conts:
                    x, y, w, h = cv2.boundingRect(cont)
                    cv2.rectangle(self.src, (x, y), (x+w, y+h), Colors.RED, 2)
                return self
            return self

        for cont in conts:
            x, y, w, h = cv2.boundingRect(cont)
            cv2.rectangle(self.src, (x, y), (x+w, y+h), Colors.GREEN, 2)
        return self


class Contour:
    def __init__(self, cont: np.ndarray) -> None:
        self.cont = cont

    def get_moments(self) -> Moments:
        self.moments = Moments(cv2.moments(self.cont))
        return self.moments

    def get_m_center(self) -> Point:
        self.m_center: Point = self.moments.center
        return self.m_center

    def get_area(self) -> int:
        self.area = cv2.contourArea(self.cont)
        return self.area

    def get_m_area(self) -> int:
        ...
    
    def get_bounding_rect(self) -> Rect:
        self.x, self.y, self.w, self.h = cv2.boundingRect(self.cont)
        
        self.rect = Rect(
            self.x,
            self.y,
            self.w,
            self.h
        )

        return self.rect

    def approx(self, value: float = 0.01) -> Contour:
        self.cont = cv2.approxPolyDP(
                self.cont,
                value * cv2.arcLength(
                    self.cont,
                    True
                    ),
                True
                )
        return self
    
    def get_approxed(self, value: float = 0.01) -> Contour:
        self.approxed_cont = cv2.approxPolyDP(
                self.cont,
                value * cv2.arcLength(
                    self.cont,
                    True
                    ),
                True
                )
        return self

class Rect:
    def __init__(
            self,
            x: int = 0,
            y: int = 0,
            w: int = 0,
            h: int = 0,
        ) -> None:

        self.x = int(x)
        self.y = int(y)
        self.w = int(w)
        self.h = int(h)

    def to_roi(self) -> ROI:
        return ROI.from_rect(self)
    
    def add_offset(self, offset: int) -> Rect:
        self.x -= offset // 2 
        self.y -= offset // 2
        self.w += offset
        self.h += offset

        return self

    def with_offset(self, offset: int) -> Rect:
        return Rect (
            self.x - offset // 2,
            self.y - offset // 2,
            self.w + offset,
            self.h + offset
        )
    
    def with_roi_offset(self, roi: ROI) -> Rect:
        return Rect (
            self.x + roi.x1,
            self.y + roi.y1,
            self.w,
            self.h
        )

class ROI:
    def __init__(
            self,
            y1: int,
            y2: int,
            x1: int,
            x2: int,
        ) -> None:

        self.y1 = y1
        self.y2 = y2
        self.x1 = x1
        self.x2 = x2

    @staticmethod
    def from_rect(rect: Rect) -> ROI:
        return ROI(
            y1=rect.y,
            y2=rect.y + rect.h,
            x1=rect.x,
            x2=rect.x + rect.w
        )

class Point:
    def __init__(self, x, y) -> None:
        self.x = x
        self.y = y

class Moments:
    def __init__(self, M) -> None:
        self.M = M 

        self.center: Point = Point(int(M['m10']/M['m00']), int(M['m01']/M['m00']))   
