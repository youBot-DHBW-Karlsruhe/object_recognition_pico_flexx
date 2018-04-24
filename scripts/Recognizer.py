#!/usr/bin/env python
from __future__ import print_function

import cv2
import roslib
import rospy

from Detector import Detector

roslib.load_manifest('object_recognition_pico_flexx')


class Recognizer(Detector):

    def __init__(self):
        Detector.__init__(self)
        self.loop_method = self.recognize_objects

        self.objects = self.object_manager.load_objects()

    def recognize_objects(self):
        # Find best matching contour for each object
        recognized_contours = {}

        for obj in self.objects:
            contour_index, difference = self.find_matching_contour(obj["contour"])

            if contour_index is not None:
                # Only assign object to a contour if it is a better match for that contour
                if contour_index not in recognized_contours or recognized_contours[contour_index] > difference:
                    recognized_contours[contour_index] = (obj, difference)

        # Visualize found objects
        for contour_index, (obj, difference) in recognized_contours.iteritems():
            # Show best result
            print("Difference for", obj["name"], ":", difference)

            self.draw_contour(contour_index)
            angle = self.get_initial_gripper_expanse(contour_index)
            center = self.get_center_on_image(contour_index)

            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(self.image_rgb, obj["name"], center, font, 0.5, self.colors["pink"])

            # print("Angle in scene", angle)

        self.show_image_wait("Recognized Objects")

        # (rows, cols) = cv_image.shape
        # if cols > 60 and rows > 60:
        #     cv2.circle(cv_image, (50, 50), 10, 255)

        # cv2.imshow("Object Recognition", cv_image)
        # cv2.waitKey(3)

        # self.image_pub.publish(self.cv_bridge.cv2_to_imgmsg(cv_image, "32FC1"))

        #
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        # self.pub.publish(hello_str)

        # TODO: publish recognized_objects


def main():
    Recognizer()
    rospy.init_node('object_recognizer', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
