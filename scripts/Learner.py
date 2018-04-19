#!/usr/bin/env python
from __future__ import print_function

import cv2
import roslib
import rospy

from Detector import Detector

roslib.load_manifest('object_recognition_pico_flexx')


class Learner(Detector):

    def __init__(self):
        Detector.__init__(self)
        self.main_method = self.learn_objects

        self.state = "start"
        self.pressed_key = -1

        self.object_contour = None
        self.object_angle = None
        self.object_center = None

    def learn_objects(self):
        if self.state == "start":
            self.start()

        elif self.state == "find_objects":
            self.find_objects()

        elif self.state == "track_object":
            self.track_object()

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

    def find_objects(self):
        contours = self.contours
        image_show = self.image_rgb

        for index, contour in enumerate(contours):
            if index < len(self.colors):
                # drawContours(image, contours, contourIdx, color, thickness)
                cv2.drawContours(image_show, contours, index, self.colors.values()[index], 1)
        self.show_image(image_show, "Learner")

        self.pressed_key = cv2.waitKey(500) & 255
        for index, contour in enumerate(contours):
            if index < len(self.colors):
                if self.pressed_key == ord(self.colors.keys()[index][0]):
                    rospy.loginfo(
                        "Tracking the " + self.colors.keys()[index] + " object. Do you want to save it? (y/n)")
                    self.object_contour = contour
                    self.object_angle = self.get_contour_angle_on_image(contours[index], image_show, None)
                    self.object_center = self.get_center_on_image(contours[index], image_show, None)
                    self.state = "track_object"

    def track_object(self):
        contours = self.contours
        image_show = self.image_rgb

        index, difference = self.find_best_matching_contour(self.object_contour, contours)

        if index is not None:
            # Show best result
            # print("Contour Length:", len(contours[index]))
            print("Difference:", difference)

            cv2.drawContours(image_show, contours, index, self.colors["green"], 1)
            angle = self.get_contour_angle_on_image(contours[index], image_show, self.colors["green"])
            self.get_center_on_image(contours[index], image_show, self.colors["red"])

            # print("Angle in scene", angle)

        self.show_image(image_show, "Learner")
        self.pressed_key = cv2.waitKey(500) & 255
        if self.pressed_key == ord("n"):
            self.state = "start"
        if self.pressed_key == ord("y"):
            self.state = "save_object"

    def save_object(self):
        name = raw_input("Please enter a name for the object...")
        rospy.loginfo("Saving object under name '" + name + "'...")
        self.object_manager.save(name, self.object_contour, self.object_angle, self.object_center)

        self.state = "start"


def main():
    Learner()
    rospy.init_node('object_learner', anonymous=True, disable_signals=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
