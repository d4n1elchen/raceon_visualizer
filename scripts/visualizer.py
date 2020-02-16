#!/usr/bin/python3

## Get image data

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from raceon.msg import TrackPosition

# Dependencies for visualization
import cv2
import numpy as np

class Visualizer():

    def __init__(self):
        # Topic name
        self.topic_name_camera_image = rospy.get_param("~topic_name_camera_image", "camera/image")
        self.topic_name_pos_err = rospy.get_param("~topic_name_position_error", "position/error")
        self.topic_name_pos_track = rospy.get_param("~topic_name_position_track", "position/track")

        # Parameter name
        self.param_name_scan_line = rospy.get_param("~param_name_scan_line", "scan_line")
        self.param_name_track_width = rospy.get_param("~param_name_track_width", "track_width")
        self.param_name_camera_center = rospy.get_param("~param_name_camera_center", "camera_center")

        # Parameters for vis
        self.scan_line = rospy.get_param(self.param_name_scan_line, 170)
        self.track_width = rospy.get_param(self.param_name_track_width, 600)
        self.camera_center = rospy.get_param(self.param_name_camera_center, 320)

        self.pos = self.camera_center
        self.left = self.camera_center-self.track_width//2
        self.right = self.camera_center+self.track_width//2

        self.line_width_params = 2
        self.line_width_indicator = 2

    def start(self):
        blank_image = np.zeros(shape=[480, 640, 1], dtype=np.uint8)
        cv2.imshow("Visualizer", blank_image)

        self.sub_camera = rospy.Subscriber(self.topic_name_camera_image, Image, self.image_callback)
        self.sub_pos_err = rospy.Subscriber(self.topic_name_pos_err, Pose, self.pos_err_callback)
        self.sub_pos_track = rospy.Subscriber(self.topic_name_pos_track, TrackPosition, self.pos_track_callback)
        rospy.spin()

    def image_callback(self, img_msg):
        width = img_msg.width
        height = img_msg.height

        np_arr = np.frombuffer(img_msg.data, dtype=np.uint8)
        gray = np_arr.reshape((height, width, 1))

        rospy.loginfo("Image with shape {:s} received. (max, min)=({:d}, {:d})".format(str(gray.shape), gray.min(), gray.max()))

        img = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)

        cv2.line(img, (0, self.scan_line), (width, self.scan_line), (255,255,255), self.line_width_params)
        cv2.line(img, (self.camera_center, height-50), (self.camera_center, height), (0,255,255), self.line_width_params)
        cv2.line(img, (self.camera_center-self.track_width//2, height-50), (self.camera_center-self.track_width//2, height), (0,0,255), self.line_width_params)
        cv2.line(img, (self.camera_center+self.track_width//2, height-50), (self.camera_center+self.track_width//2, height), (0,0,255), self.line_width_params)

        cv2.line(img, (self.camera_center-self.pos, 0), (self.camera_center-self.pos, height), (255,0,255), self.line_width_indicator)
        if(self.left):
            cv2.line(img, (self.left, 0), (self.left, height), (0,255,0), self.line_width_indicator)
        if(self.right):
            cv2.line(img, (self.right, 0), (self.right, height), (0,255,0), self.line_width_indicator)

        cv2.imshow("Visualizer", img)
        cv2.waitKey(1)

    def pos_err_callback(self, err_msg):
        self.pos = int(err_msg.position.x)
        rospy.loginfo("Get position info: {:d}".format(self.pos))

    def pos_track_callback(self, track_msg):
        self.left = int(track_msg.left)
        self.right = int(track_msg.right)
        rospy.loginfo("Get track info: {:d}, {:d}".format(self.left, self.right))

if __name__ == "__main__":
    cv2.namedWindow("Visualizer")
    rospy.init_node("visualizer")
    visualizer = Visualizer()
    try:
        visualizer.start()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()
