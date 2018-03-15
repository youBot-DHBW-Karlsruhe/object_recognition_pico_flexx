#!/usr/bin/env python
from __future__ import print_function

import time

import cv2
import roslib
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import helper

roslib.load_manifest('object_recognition_pico_flexx')


class ObjectLearner:

    def __init__(self):
        self.image_sub = rospy.Subscriber("/royale_camera_driver/depth_image", Image, self.callback)
        self.cv_bridge = CvBridge()
        # print("OpenCV version: {0}".format(cv2.__version__)) # 2.4.8

        self.timestamp_last_call = time.time()
        self.state = "start"
        self.pressed_key = -1

        self.colors = {"blue": (255, 0, 0), "green": (0, 255, 0), "red": (0, 0, 255), "yellow": (0, 255, 255)}

        self.tracking_color = None
        self.tracking_contour = None

    def callback(self, img_msg):
        # Prevent running multiple callbacks at once
        if time.time() > self.timestamp_last_call + 1:
            self.timestamp_last_call = time.time()
            self.learn_objects(img_msg)
        # else:
        # print("Ignored")

    def learn_objects(self, img_msg):
        cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg, "32FC1")
        # Convert 32fc1 to 8uc1 (Gray scale)
        image = (cv_image * 255).astype('u1')
        # helper.show_image(image, "Origin")
        contours = helper.get_contours(image, False)
        image_show = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)

        if self.state == "start":
            self.start()

        if self.state == "find_objects":
            self.find_objects(contours, image_show)

        elif self.state == "track_object":
            self.track_object(contours, image_show)

        elif self.state == "save_object":
            self.save_object()

        else:
            rospy.logerr("Unknown learner state! Shutting down...")
            rospy.signal_shutdown("Unknown state")

        if self.pressed_key == 27:  # 27 = Escape key
            rospy.signal_shutdown("User Shutdown")

    def start(self):
        rospy.loginfo("Press any of the following buttons to save the respective object:")
        rospy.loginfo(
            " ".join(["'" + color_name[0] + "'" + " for " + color_name + ";" for color_name in self.colors.keys()]))
        self.state = "find_objects"

    def find_objects(self, contours, image_show):

        for index, contour in enumerate(contours):
            if index < len(self.colors):
                # drawContours(image, contours, contourIdx, color, thickness)
                cv2.drawContours(image_show, contours, index, self.colors.values()[index], 1)
        helper.show_image(image_show, "Learner")

        self.pressed_key = cv2.waitKey(500) & 255
        for index, contour in enumerate(contours):
            if index < len(self.colors):
                if self.pressed_key == ord(self.colors.keys()[index][0]):
                    rospy.loginfo(
                        "Tracking the " + self.colors.keys()[index] + " object. Do you want to save it? (y/n)")
                    self.tracking_color = self.colors.values()[index]
                    self.tracking_contour = contour
                    self.state = "track_object"

    def track_object(self, contours, image_show):
        index, difference, angle = helper.find_best_matching_contour(self.tracking_contour, contours, image_show, True)

        # Show best result
        print("Contour Length:", len(contours[index]))
        print("Matching:", difference)
        print("Angle in scene", angle)

        cv2.drawContours(image_show, contours, index, self.tracking_color, 1)

        helper.show_image(image_show, "Learner")
        self.pressed_key = cv2.waitKey(500) & 255
        if self.pressed_key == ord("n"):
            self.state = "find_objects"
        if self.pressed_key == ord("y"):
            self.state = "save_object"

    def save_object(self):
        rospy.loginfo("Saving object...")
        self.state = "start"


def main():
    ObjectLearner()
    rospy.init_node('object_learner', anonymous=True, disable_signals=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
