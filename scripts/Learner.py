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
        # Show colored contours
        for contour_index, contour in enumerate(self.contours):
            if contour_index < len(self.colors):
                # drawContours(image, contours, contourIdx, color, thickness)
                self.draw_contour(contour_index, contour_index)
        self.show_image(self.image_rgb, "Learner")

        # Get desired object
        self.pressed_key = cv2.waitKey(500) & 255
        for contour_index, contour in enumerate(self.contours):
            if contour_index < len(self.colors):
                if self.pressed_key == ord(self.colors.keys()[contour_index][0]):
                    # Saving object details
                    self.object_contour = contour
                    self.object_angle = self.get_contour_angle_on_image(contour_index)
                    self.object_center = self.get_center_on_image(contour_index)

                    # Inform user
                    rospy.loginfo("Tracking the " + self.colors.keys()[contour_index] + " object.")
                    rospy.loginfo("Do you want to save it? (y/n)")
                    self.state = "track_object"

    def track_object(self):
        contour_index, difference = self.find_matching_contour(self.object_contour)

        if contour_index is not None:
            # Show best result
            print("Difference:", difference)

            self.draw_contour(contour_index)
            self.get_contour_angle_on_image(contour_index)
            self.get_center_on_image(contour_index)

        # Get desired action (save/cancel)
        self.show_image(self.image_rgb, "Learner")
        self.pressed_key = cv2.waitKey(500) & 255
        if self.pressed_key == ord("y"):
            self.state = "save_object"
        if self.pressed_key == ord("n"):
            self.state = "start"

    def save_object(self):
        # Getting object name
        name = raw_input("Please enter a name for the object...")
        # Saving object
        rospy.loginfo("Saving object under name '" + name + "'...")
        self.object_manager.save(name, self.object_contour, self.object_angle, self.object_center)
        # Back to start
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
