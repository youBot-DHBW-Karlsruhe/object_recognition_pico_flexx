#!/usr/bin/env python
from __future__ import print_function

import cv2
import roslib
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import time

import helper

roslib.load_manifest('object_recognition_pico_flexx')


class ObjectLearner:

    def __init__(self):
        self.image_sub = rospy.Subscriber("/royale_camera_driver/depth_image", Image, self.callback)
        self.cv_bridge = CvBridge()
        self.timestamp_last_call = time.time()

    def callback(self, img_msg):
        # Prevent running multiple callbacks at once
        if time.time() > self.timestamp_last_call + 1:
            self.timestamp_last_call = time.time()
            self.learn_object(img_msg)
        # else:
            # print("Ignored")

    def learn_object(self, img_msg):

        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg, "32FC1")
            # Convert 32fc1 to 8uc1 (Gray scale)
            image = (cv_image * 255).astype('u1')
            # helper.show_image(image, "Origin")

            contours = helper.get_contours(image, True)

            image_show = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
            colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (0, 255, 255), (255, 0, 255)]
            for index, contour in enumerate(contours):
                if index < 5:
                    # drawContours(image, contours, contourIdx, color, thickness)
                    cv2.drawContours(image_show, contours, index, colors[index], 1)
            helper.show_image(image_show, "Learner")

            pressed_key = cv2.waitKey(500) & 255
            # if pressed_key != -1:
            #     print(pressed_key)
            if pressed_key == ord('s'):
                rospy.loginfo("Saving object...")
            if pressed_key == 27:  # 27 = Escape key
                rospy.signal_shutdown("User Shutdown")

        except CvBridgeError as e:
            print(e)


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
