#!/usr/bin/env python
from __future__ import print_function

import cv2
import roslib
import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud2

from Detector import Detector

roslib.load_manifest('object_recognition_pico_flexx')


class Recognizer(Detector):

    def __init__(self):
        Detector.__init__(self)
        self.state = self.recognize_objects

        self.objects = self.object_manager.load_objects()

        rospy.Subscriber("/royale_camera_driver/point_cloud", PointCloud2, self.save_point_cloud)
        self.point_cloud = None

        self.pub_point = rospy.Publisher("/object_recognition/point", PointStamped, queue_size=10)

    def recognize_objects(self):
        # Find best matching contour for each object
        recognized_contours = {}

        for obj in self.objects:
            matching_contours = self.get_matching_contours(obj["contour"])

            for contour_index, difference in matching_contours.iteritems():
                # Only assign object to a contour if it is a better match for that contour
                if contour_index not in recognized_contours or difference < recognized_contours[contour_index][1]:
                    recognized_contours[contour_index] = (obj, difference)

        if recognized_contours:
            self.visualize_recognitions(recognized_contours)

            best_match = min(recognized_contours.items(), key=lambda x: x[1][1])
            self.publish_match(best_match)

        self.show_image_wait("Recognized Objects")

    def visualize_recognitions(self, recognized_contours):

        # Visualize found objects
        for contour_index, (obj, difference) in recognized_contours.iteritems():
            self.draw_contour(contour_index)
            center = self.get_center_on_image(contour_index)

            cv2.putText(self.image_rgb, obj["name"], center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.colors["pink"])

            print("Difference for", obj["name"], ":", difference)

    def publish_match(self, match):
        contour_index, (obj, difference) = match
        midpoint, angle, distance = self.get_gripper_parameters(contour_index)

        point_stamped = PointStamped(self.point_cloud.header, self.get_3d_point(midpoint))

        self.pub_point.publish(point_stamped)

    def save_point_cloud(self, point_cloud):
        self.point_cloud = point_cloud

    def get_3d_point(self, point_2d):
        u, v = point_2d
        point_3d = list(pc2.read_points(self.point_cloud, uvs=[(u, v)]))[0]
        x, y, z = point_3d

        return Point(x, y, z)


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
