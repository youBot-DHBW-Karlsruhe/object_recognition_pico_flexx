#!/usr/bin/env python
from __future__ import print_function

import cv2
import roslib
import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from object_recognition_pico_flexx.msg import RecognizedObject
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

        self.pub_position = rospy.Publisher("/object_recognition/point", PointStamped, queue_size=10)
        self.pub_object = rospy.Publisher("/object_recognition/recognized_object", RecognizedObject, queue_size=10)

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

            print("Found object:", obj["name"])

    def publish_match(self, match):
        contour_index, (obj, difference) = match
        midpoint, finger_point1, finger_point2, rotation = self.get_object_parameters(contour_index)

        [midpoint, finger_point1, finger_point2] = self.get_3d_points([midpoint, finger_point1, finger_point2])
        width = self.get_distance((finger_point1.x, finger_point1.y), (finger_point2.x, finger_point2.y))

        self.pub_position.publish(PointStamped(self.point_cloud.header, midpoint))
        self.pub_object.publish(RecognizedObject(
            self.point_cloud.header,
            str(obj["name"]),
            midpoint,
            width,
            int(rotation)
        ))

    def save_point_cloud(self, point_cloud):
        self.point_cloud = point_cloud

    def get_3d_points(self, points_2d):
        points_3d = []
        for point_3d in list(pc2.read_points(self.point_cloud, uvs=points_2d)):
            x, y, z = point_3d
            points_3d.append(Point(x, y, z))
        return points_3d


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
