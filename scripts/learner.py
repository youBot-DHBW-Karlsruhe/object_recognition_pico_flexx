#!/usr/bin/env python
from __future__ import print_function

import roslib
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

roslib.load_manifest('object_recognition_pico_flexx')


class ObjectLearner:

    def __init__(self):
        self.image_sub = rospy.Subscriber("/royale_camera_driver/depth_image", Image, self.recognize_objects)
        self.cv_bridge = CvBridge()

    def recognize_objects(self, img_msg):
        recognized_objects = []

        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg, "32FC1")

            # (rows, cols) = cv_image.shape
            # if cols > 60 and rows > 60:
            #     cv2.circle(cv_image, (50, 50), 10, 255)

            cv2.imshow("Object Learning", cv_image)

            pressed_key = cv2.waitKey(1)
            # if pressed_key != -1:
            #     print(pressed_key)
            if pressed_key == ord('s'):
                rospy.loginfo("Saving object...")
            if pressed_key == 27:  # 27 = Escape key
                rospy.signal_shutdown("User Shutdown")

        except CvBridgeError as e:
            print(e)

        # TODO: publish recognized_objects


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
