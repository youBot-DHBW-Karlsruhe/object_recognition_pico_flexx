#!/usr/bin/env python
from __future__ import print_function

import time

import cv2
import roslib
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import helper
from ObjectManager import ObjectManager

roslib.load_manifest('object_recognition_pico_flexx')


# http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
class Recognizer:

    def __init__(self):
        self.image_sub = rospy.Subscriber("/royale_camera_driver/depth_image", Image, self.callback)
        self.cv_bridge = CvBridge()

        self.timestamp_last_call = time.time()

        object_manager = ObjectManager("../objects/objects.json")
        self.objects = object_manager.load_objects()

        # self.pub = rospy.Publisher('/recognized_objects', String, queue_size=10)
        self.image_pub = rospy.Publisher("object_recognizer_visualization", Image, queue_size=10)

    def callback(self, img_msg):
        # Prevent running multiple callbacks at once
        if time.time() > self.timestamp_last_call + 1:  # time.time() in seconds
            self.timestamp_last_call = time.time()
            self.recognize_objects(img_msg)

    def recognize_objects(self, img_msg):

        image_bw, image_rgb = helper.convert_img_msg(self.cv_bridge, img_msg)
        contours = helper.get_contours(image_bw, False)

        recognized_contours = {}
        for obj in self.objects:
            index, difference = helper.find_best_matching_contour(obj["contour"], contours)

            if index is not None:
                if index not in recognized_contours or recognized_contours[index] > difference:
                    recognized_contours[index] = (obj, difference)

        for index, (obj, difference) in recognized_contours.iteritems():
            # rospy.loginfo("Found object with name '" + obj["name"] + "'!")
            # Show best result
            # print("Contour Length:", len(contours[index]))
            print("Difference:", difference)

            cv2.drawContours(image_rgb, contours, index, helper.colors["green"], 1)
            angle = helper.get_contour_angle_on_image(contours[index], image_rgb, helper.colors["green"])
            center = helper.get_center_on_image(contours[index], image_rgb, helper.colors["red"])

            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(image_rgb, obj["name"], center, font, 0.5, helper.colors["pink"])

            # print("Angle in scene", angle)

        helper.show_image(image_rgb, "Recognized Objects")
        pressed_key = cv2.waitKey(500) & 255
        if pressed_key == 27:  # 27 = Escape key
            rospy.signal_shutdown("User Shutdown")

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
