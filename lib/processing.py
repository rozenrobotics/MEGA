import cv2


class Processing:
    @staticmethod
    def findBiggestContours(conts, num_conts):
        contour_areas = [(cv2.contourArea(contour), contour) for contour in conts]

        contour_areas.sort(reverse=True, key=lambda x: x[0])

        # Extract the top num_conts contours
