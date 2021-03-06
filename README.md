# ROS Object Detection and Classification with _pmd CamBoard  pico flexx_

## Introduction

This repository contains a ROS node for Object Detection and grasp planning. After naming objects, the program recognizes the objects and claculates a grasping point for the two-finger gripper of _KUKA youBot_.

See the following video for visualization:

[![YouTube Video](https://img.youtube.com/vi/4J9ihnDPLBA/0.jpg)](https://www.youtube.com/watch?v=4J9ihnDPLBA)

## Environment and subscribed topics

For a full environment setup please refer to [this document](https://docs.google.com/document/d/1mHekbqrnkeZCLNHFKreaERwDxS6Y1B2LWokVlte4wV0/edit?usp=sharing).

The node expects depth image messages under the topic `/royale_camera_driver/depth_image` and point_cloud2 messages under the topic `/royale_camera_driver/point_cloud`. These are provided by the official [pmd Royale ROS Wrapper](https://pmdtec.com/picofamily/2018/04/05/ros-support-for-pico-flexx-and-pico-monstar/).

## Published Topic

The node publishes object messages under the topic `/object_recognition/recognized_object`. The message type is defined in [msg/RecognizedObject.msg](msg/RecognizedObject.msg). An example message is shown below:

    header: 
      seq: 1939
      stamp: 
        secs: 1525095329
        nsecs: 744063000
      frame_id: royale_camera_link
    name: Duplo
    midpoint: 
      x: -0.0169242266566
      y: 0.00806986819953
      z: 0.188075810671
    width: 0.033241365099
    rotation: -51

## Configuration

Configuration is done in [parameters/settings.json](parameters/settings.json). The default values in this file correspond to the youBot object recognition pose descibed in [parameters/ObjectRecognitionPose.md](parameters/ObjectRecognitionPose.md).

- `"debugging"`: Do you want to see debugging images?

- `"objects"`: Path to object .json fil

- `"camera_thresh"`: Camera distance to ground. (Float in meters)

- `"camera_max"`: Camera distance to highest object. (Float in meters)

- `"maximal_contour_difference"`: Maxiumum difference between contours to be recognized accoridng to Hu-Moments.

- `"minimal_contour_length"`: Miniumum number of a contour's points to be estimated as potential object.

## Execution

1. Executing the Learner:
    `roslaunch object_recognition_pico_flexx Learner.launch`
    
2. Executing the Recognizer:
    `roslaunch object_recognition_pico_flexx Recognizer.launch`
