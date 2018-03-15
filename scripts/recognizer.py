#!/usr/bin/env python
from __future__ import print_function

import time

import roslib
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

roslib.load_manifest('object_recognition_pico_flexx')

# http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
class ObjectRecognizer:

    def __init__(self):
        self.image_sub = rospy.Subscriber("/royale_camera_driver/depth_image", Image, self.callback)
        self.cv_bridge = CvBridge()

        self.timestamp_last_call = time.time()

        # self.pub = rospy.Publisher('/recognized_objects', String, queue_size=10)
        self.image_pub = rospy.Publisher("object_recognizer_visualization", Image, queue_size=10)

    def callback(self, img_msg):
        # Prevent running multiple callbacks at once
        if time.time() > self.timestamp_last_call + 1:
            self.timestamp_last_call = time.time()
            self.recognize_objects(img_msg)

    def recognize_objects(self, img_msg):

        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg, "32FC1")

            # (rows, cols) = cv_image.shape
            # if cols > 60 and rows > 60:
            #     cv2.circle(cv_image, (50, 50), 10, 255)

            # cv2.imshow("Object Recognition", cv_image)
            # cv2.waitKey(3)

            self.image_pub.publish(self.cv_bridge.cv2_to_imgmsg(cv_image, "32FC1"))

        #
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        # self.pub.publish(hello_str)

        # TODO: publish recognized_objects


def main():
    ObjectRecognizer()
    rospy.init_node('object_recognizer', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
