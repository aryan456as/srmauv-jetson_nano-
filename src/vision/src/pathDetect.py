import cv2
import numpy as np
import math

class PathDetector:
    def __init__(self):
        self.img = cv2.imread('path3.png', 1)
        self.detect_path()

    def detect_path(self):
        h, w, c = self.img.shape

        hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        blur = cv2.GaussianBlur(hsv, (5, 5), 0)

        lower_red = np.array([0, 0, 0])
        upper_red = np.array([70, 220, 200])

        mask = cv2.inRange(blur, lower_red, upper_red)

        kernel = np.ones((4, 4), np.uint8)
        res = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        im2, contours, hierarchy = cv2.findContours(res, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnt = contours[0]

        M = cv2.moments(cnt)

        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        cv2.circle(self.img, (cx, cy), 4, (0, 0, 255), -1)
        cv2.circle(self.img, (int(w / 2), int(h / 2)), 4, (0, 0, 255), -1)

        cv2.circle(self.img, (w, h), 4, (0, 0, 255), -1)
        cv2.circle(self.img, (0, 0), 4, (0, 0, 255), -1)

        rect = cv2.minAreaRect(cnt)
        (x, y), (width, height), theta = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        cv2.drawContours(self.img, [box], 0, (0, 0, 255), 2)
        pt1 = (box[0][0], box[0][1])
        pt4 = (box[3][0], box[3][1])
        pt2 = (box[1][0], box[1][1])
        pt3 = (box[2][0], box[2][1])

        x1, y1 = (pt4[0] + pt1[0]) / 2.0, (pt4[1] + pt1[1]) / 2.0
        x2, y2 = (pt2[0] + pt3[0]) / 2.0, (pt2[1] + pt3[1]) / 2.0

        cv2.line(self.img, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
        cv2.line(self.img, (int(w / 2), 0), (int(w / 2), h), (255, 0, 0), 2)

        pt1 = (box[0][0], h - box[0][1])
        pt4 = (box[3][0], h - box[3][1])
        pt2 = (box[1][0], h - box[1][1])
        pt3 = (box[2][0], h - box[2][1])

        x1, y1 = (pt4[0] + pt1[0]) / 2.0, (pt4[1] + pt1[1]) / 2.0
        x2, y2 = (pt2[0] + pt3[0]) / 2.0, (pt2[1] + pt3[1]) / 2.0

        angle = math.atan2(y2 - y1, x2 - x1)
        print(math.degrees(angle))

        cv2.imshow('res', self.img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

if __name__ == "__main__":
    detector = PathDetector()
