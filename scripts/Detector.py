import numpy as np
import time

import cv2
import roslib
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from JsonManager import JsonManager
from ObjectManager import ObjectManager

roslib.load_manifest('object_recognition_pico_flexx')


class Detector:

    def __init__(self):
        self.image_sub = rospy.Subscriber("/royale_camera_driver/depth_image", Image, self.callback)
        self.cv_bridge = CvBridge()

        # print("OpenCV version: {0}".format(cv2.__version__)) # 2.4.8
        self.settings = JsonManager("../parameters/settings.json").load_json()
        self.object_manager = ObjectManager(self.settings["objects"])

        self.timestamp_last_call = time.time()
        self.loop_method = None
        self.pressed_key = -1

        self.colors = {"blue": (255, 0, 0), "green": (0, 255, 0), "red": (0, 0, 255), "yellow": (0, 255, 255),
                       "pink": (255, 0, 255)}

        self.image_bw = None
        self.image_rgb = None
        self.contours = None

        rospy.loginfo("To enable/disable debugging images, press D")
        self.debugging = True

    def callback(self, img_msg):
        # Prevent running multiple callbacks at once
        if time.time() > self.timestamp_last_call + 1:  # time.time() in seconds
            self.timestamp_last_call = time.time()

            self.convert_img_msg(img_msg)
            self.get_contours()
            self.loop_method()

            # Detect user shutdown
            if self.pressed_key == 27:  # 27 = Escape key
                rospy.signal_shutdown("User Shutdown")
            # Detect debugging
            if self.pressed_key == ord('d'):
                self.debugging = not self.debugging
                rospy.loginfo("Debugging: " + str(self.debugging))

    def find_matching_contour(self, contour_object):
        smallest_difference = 0.2
        index_best = None

        for index, contour_scene in enumerate(self.contours):
            difference = cv2.matchShapes(contour_scene, contour_object, 1, 0.0)
            if difference < smallest_difference:
                smallest_difference = difference
                index_best = index

        return index_best, smallest_difference

    def get_center_on_image(self, contour_index):
        # noinspection PyPep8Naming
        M = cv2.moments(self.contours[contour_index])
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        cv2.circle(self.image_rgb, (cx, cy), 1, self.colors["red"])

        return cx, cy

    def get_initial_gripper_expanse(self, contour_index):
        m = -1 / self.get_rotation(contour_index)

        center = self.get_center_on_image(contour_index)
        cx, cy = center

        rows, cols = self.image_rgb.shape[:2]

        # First point
        x1 = 0
        y1 = int(cy - cx * m)
        point_1 = (x1, y1)
        # Second point
        x2 = cols - 1
        y2 = int(cy + (x2 - cx) * m)
        point_2 = (x2, y2)

        intersection1 = self.intersect_line_with_contour(center, point_1, contour_index)
        intersection2 = self.intersect_line_with_contour(center, point_2, contour_index)

        cv2.circle(self.image_rgb, intersection1, 3, color=self.colors["yellow"], thickness=1)
        cv2.circle(self.image_rgb, intersection2, 3, color=self.colors["yellow"], thickness=1)

        distance = np.sqrt(np.power(intersection1[0] - intersection2[0], 2)
                           + np.power(intersection1[1] - intersection2[1], 2))

        return distance

    def intersect_line_with_contour(self, line_start, line_end, contour_index):
        contour_pixels = np.zeros(self.image_rgb.shape[0:2])
        line_pixels = np.zeros(self.image_rgb.shape[0:2])
        debug = np.zeros(self.image_rgb.shape[0:2])

        cv2.drawContours(contour_pixels, self.contours, contour_index, 1)
        cv2.drawContours(debug, self.contours, contour_index, 1)
        cv2.line(line_pixels, line_start, line_end, 1, thickness=2)
        cv2.line(debug, line_start, line_end, 1, thickness=2)

        intersection = np.logical_and(contour_pixels, line_pixels)

        intersection_x = int(np.average(np.where(intersection)[1]))
        intersection_y = int(np.average(np.where(intersection)[0]))

        cv2.circle(debug, (intersection_x, intersection_y), 5, color=1, thickness=1)

        return intersection_x, intersection_y

    def get_rotation(self, contour_index):
        [vx, vy, x, y] = cv2.fitLine(self.contours[contour_index], cv2.cv.CV_DIST_L2, 0, 0.01, 0.01)

        return vy / vx

    def get_contours(self):
        # Only pay attention to objects nearer/darker than ... Else --> 0 (ignore, is ground)

        # Delete obstacles further away than ...
        thresh = (self.settings["camera_thresh"] - self.settings["camera_min"]) * 255 / \
                 (self.settings["camera_max"] - self.settings["camera_min"])

        ret, prepared_image = cv2.threshold(self.image_bw, thresh, 0, cv2.THRESH_TOZERO_INV)
        # Remove noise
        prepared_image = cv2.morphologyEx(prepared_image, cv2.MORPH_OPEN,
                                          cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))

        self.debug_image(prepared_image, "Prepared Image")

        # Prepare image edges
        # edges = cv2.morphologyEx(image, cv2.MORPH_GRADIENT, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
        # if show:
        #     show_image(edges, "Edges")

        # Find Contours
        contours = cv2.findContours(prepared_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]

        # Filter useful contours
        self.contours = []
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
                    self.contours.append(contour)

        # Sort useful contours by their number of points
        self.contours.sort(key=len, reverse=True)

        # Show useful contours
        image_show = np.copy(self.image_rgb)
        # drawContours(image, contours, contourIdx, color, thickness)
        cv2.drawContours(image_show, self.contours, -1, (0, 255, 255), 1)
        for corner in corners:
            cv2.circle(image_show, corner, 1, color=self.colors["blue"], thickness=1)
        self.debug_image(image_show, "Useful Contours")

    def convert_img_msg(self, img_msg):
        # Convert ros image message to numpy array
        float_image = self.cv_bridge.imgmsg_to_cv2(img_msg, "32FC1")

        # Adjust Image
        float_image[float_image < 0.01] = self.settings["camera_thresh"]
        float_image = (float_image - self.settings["camera_min"]) / \
                      (self.settings["camera_max"] - self.settings["camera_min"])
        float_image[float_image > 1] = 1
        float_image[float_image < 0] = 0

        # Convert 32fc1 to 8uc1
        self.image_bw = (float_image * 255).astype('u1')
        self.image_rgb = cv2.cvtColor(self.image_bw, cv2.COLOR_GRAY2RGB)

    def show_image_wait(self, window_name):
        show_image(self.image_rgb, window_name)
        self.pressed_key = cv2.waitKey(500) & 255

    def debug_image(self, image, window_name):
        if self.debugging:
            show_image(image, window_name)

    def draw_contour(self, contour_index, color_index=2):
        cv2.drawContours(self.image_rgb, self.contours, contour_index, self.colors.values()[color_index], 1)


def show_image(image, window_name):
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, image.shape[1] * 4, image.shape[0] * 4)
    cv2.imshow(window_name, image)  # Show image
