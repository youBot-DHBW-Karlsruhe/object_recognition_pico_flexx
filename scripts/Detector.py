import numpy as np
import time

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from ObjectManager import ObjectManager


class Detector:

    def __init__(self):
        self.image_sub = rospy.Subscriber("/royale_camera_driver/depth_image", Image, self.callback)
        self.cv_bridge = CvBridge()

        # print("OpenCV version: {0}".format(cv2.__version__)) # 2.4.8
        self.object_manager = ObjectManager("../objects/objects.json")

        self.timestamp_last_call = time.time()
        self.main_method = None

        self.colors = {"blue": (255, 0, 0), "green": (0, 255, 0), "red": (0, 0, 255), "yellow": (0, 255, 255),
                       "pink": (255, 0, 255)}

        self.image_bw = None
        self.image_rgb = None
        self.contours = None

    def callback(self, img_msg):
        # Prevent running multiple callbacks at once
        if time.time() > self.timestamp_last_call + 1:  # time.time() in seconds
            self.timestamp_last_call = time.time()

            self.convert_img_msg(img_msg)
            self.get_contours(True)
            self.main_method()

    def find_matching_contour(self, contour_object):
        smallest_difference = 0.2
        index_best = None

        for index, contour_scene in enumerate(self.contours):
            difference = cv2.matchShapes(contour_scene, contour_object, 1, 0.0)
            if difference < smallest_difference:
                smallest_difference = difference
                index_best = index

        return index_best, smallest_difference

    def get_center_on_image(self, contour_index, drawing_color=None):
        M = cv2.moments(self.contours[contour_index])
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        if drawing_color is not None:
            cv2.circle(self.image_rgb, (cx, cy), 1, drawing_color)

        return cx, cy

    def get_contour_angle_on_image(self, contour_index, drawing_color=None):
        x1, y1, x2, y2 = self.get_contour_line_on_image(contour_index)
        angle_rad = np.arctan((y2 - y1) / (x2 - x1))
        angle_deg = angle_rad * 180 / np.pi + 90

        if drawing_color is not None:
            cv2.line(self.image_rgb, (x1, y1), (x2, y2), drawing_color)

        return angle_deg

    def get_contour_line_on_image(self, contour_index):
        rows, cols = self.image_rgb.shape[:2]
        [vx, vy, x, y] = cv2.fitLine(self.contours[contour_index], cv2.cv.CV_DIST_L2, 0, 0.01, 0.01)
        # First point
        x1 = 0
        y1 = int((-x * vy / vx) + y)
        # Second point
        x2 = cols - 1
        y2 = int(((cols - x) * vy / vx) + y)

        return x1, y1, x2, y2

    def get_contours(self, show=False):
        # Only look at the interesting brightness values
        prepared_image = self.adjust_gamma(self.image_bw, val_min=38, val_max=58)
        # Only pay attention to objects nearer/darker than ... Else --> 0 (ignore, is ground)
        # Delete objects further away than ...
        ret, prepared_image = cv2.threshold(prepared_image, ((55 - 40) * 255) / (60 - 40), 0, cv2.THRESH_TOZERO_INV)
        # Remove noise
        prepared_image = cv2.morphologyEx(prepared_image, cv2.MORPH_OPEN,
                                          cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
        if show:
            self.show_image(prepared_image, "Prepared Image")

        # Prepare image edges
        # edges = cv2.morphologyEx(image, cv2.MORPH_GRADIENT, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
        # if show:
        #     show_image(edges, "Edges")

        # Find Contours
        contours = cv2.findContours(prepared_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]

        # Filter useful contours
        useful_contours = []
        image_y, image_x = self.image_bw.shape
        corners = [(2, 2), (2, image_y - 3), (image_x - 3, 2), (image_x - 3, image_y - 3)]
        for contour in contours:
            # Only select contours with more than 40 points
            if len(contour) > 40:
                in_corner = False
                # Test if Contour is in one of the corners
                for point in corners:
                    if cv2.pointPolygonTest(contour, point, False) == 1:
                        in_corner = True
                if not in_corner:
                    useful_contours.append(contour)

        # Sort useful contours by their number of points
        useful_contours.sort(key=len, reverse=True)

        # Show useful contours
        if show:
            image_show = np.copy(self.image_rgb)
            # drawContours(image, contours, contourIdx, color, thickness)
            cv2.drawContours(image_show, useful_contours, -1, (0, 255, 255), 1)
            for corner in corners:
                cv2.circle(image_show, corner, 1, color=(255, 0, 0), thickness=1)
            self.show_image(image_show, "Useful Contours")

        self.contours = useful_contours

    def adjust_gamma(self, image, val_min=60, val_max=100):
        # build a lookup table mapping the pixel values [0, 255] to
        # their adjusted gamma values

        table = []
        for i in range(256):
            value = ((i - val_min) * 255) / (val_max - val_min)
            if i < 30:
                value = ((50 - val_min) * 255) / (val_max - val_min)
            if value <= 0:
                value = 0
            if value > 255:
                value = 255
            table.append(value)
        table = np.array(table).astype("uint8")

        # apply gamma correction using the lookup table
        return cv2.LUT(image, table)

    def convert_img_msg(self, img_msg):
        # Convert ros image message to numpy array
        cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg, "32FC1")
        # Convert 32fc1 to 8uc1 (Gray scale)
        self.image_bw = (cv_image * 255).astype('u1')
        # helper.show_image(image, "Origin")
        self.image_rgb = cv2.cvtColor(self.image_bw, cv2.COLOR_GRAY2RGB)

    def show_image_wait(self, image, window_name="Stream"):
        while True:
            self.show_image(image, window_name)
            pressed_key = cv2.waitKey(1000)
            print(pressed_key)
            if pressed_key != -1:  # Button pressed
                break

    def show_image(self, image, window_name="Stream"):
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, image.shape[1] * 4, image.shape[0] * 4)
        cv2.imshow(window_name, image)  # Show image
