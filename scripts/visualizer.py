#!/usr/bin/python3

## Get image data

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from raceon.msg import TrackPosition, AckermannDrive

# Dependencies for visualization
import time
import cv2
import numpy as np

# Manual control
from manual import Controller
from pynput.keyboard import Listener

class Visualizer():

    def __init__(self, controller):
        # Topic name
        self.topic_name_camera_image = rospy.get_param("~topic_name_camera_image", "camera/image")
        self.topic_name_pos_pose = rospy.get_param("~topic_name_position_pose", "position/pose")
        self.topic_name_pos_track = rospy.get_param("~topic_name_position_track", "position/track")

        # Parameter name
        self.param_name_scan_line = rospy.get_param("~param_name_scan_line", "scan_line")
        self.param_name_scan_line_d = rospy.get_param("~param_name_scan_line_d", "scan_line_d")
        self.param_name_scan_line_u = rospy.get_param("~param_name_scan_line_u", "scan_line_u")
        self.param_name_track_width = rospy.get_param("~param_name_track_width", "track_width")
        self.param_name_camera_center = rospy.get_param("~param_name_camera_center", "camera_center")

        # Parameters for vis
        self.scan_line = rospy.get_param(self.param_name_scan_line, 170)
        self.scan_line_d = rospy.get_param(self.param_name_scan_line_d, self.scan_line)
        self.scan_line_u = rospy.get_param(self.param_name_scan_line_u, self.scan_line)
        self.track_width = rospy.get_param(self.param_name_track_width, 600)
        self.camera_center = rospy.get_param(self.param_name_camera_center, 320)

        self.speed = 0
        self.img = np.zeros(shape=[480, 640, 3], dtype=np.uint8)
        self.width = 640
        self.height = 480
        self.pos = self.camera_center
        self.line_pos = -self.pos + self.camera_center
        self.left = self.camera_center-self.track_width//2
        self.right = self.camera_center+self.track_width//2

        self.line_width_params = 4
        self.line_width_indicator = 2

        self.frame_cnt = 0
        self.time = 0
        self.fps = 0

        # Mannual mode
        self.controller = controller
        self.topic_name_manual_mode = rospy.get_param("~topic_name_manual_mode", "control/manual_mode")
        self.topic_name_control = rospy.get_param("~topic_name_control", "control")
        self.manual_mode = False

    def start(self):
        self.sub_camera = rospy.Subscriber(self.topic_name_camera_image, Image, self.image_callback, queue_size=10)
        self.sub_pos_pose = rospy.Subscriber(self.topic_name_pos_pose, Pose, self.pos_pose_callback, queue_size=10)
        self.sub_pos_track = rospy.Subscriber(self.topic_name_pos_track, TrackPosition, self.pos_track_callback, queue_size=10)
        self.sub_control = rospy.Subscriber(self.topic_name_control, AckermannDrive, self.control_callback, queue_size=10)
        self.pub_manual_mode = rospy.Publisher(self.topic_name_manual_mode, Bool, queue_size=10)
        self.pub_control = rospy.Publisher(self.topic_name_control, AckermannDrive, queue_size=10)
        #rospy.spin()
        self.terminate = False
        while not self.terminate:
            self.plot()
            if self.manual_mode:
                self.pub_control.publish(controller.get_msg())

    def image_callback(self, img_msg):
        self.width = img_msg.width
        self.height = img_msg.height

        np_arr = np.frombuffer(img_msg.data, dtype=np.uint8)

        if img_msg.encoding == 'bgr8':
            self.img = np_arr.reshape((self.height, self.width, 3))
        elif img_msg.encoding == '8UC1' or img_msg.encoding == 'mono8':
            gray = np_arr.reshape((self.height, self.width, 1))
            self.img = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)

    def control_callback(self, control_msg):
        self.speed = control_msg.speed

    def pos_pose_callback(self, pos_msg):
        self.pos = int(pos_msg.position.x)
        self.line_pos = -self.pos + self.camera_center

    def pos_track_callback(self, track_msg):
        self.left = int(track_msg.left)
        self.right = int(track_msg.right)

    def plot_scan_line(self, img, scan_line):
        cv2.line(img, (0, scan_line), (self.width, scan_line), (255,255,255), self.line_width_params)
        cv2.line(img, (self.camera_center, scan_line-20), (self.camera_center, scan_line+20), (0,255,255), self.line_width_params)
        cv2.line(img, (self.camera_center-self.track_width//2, scan_line-20), (self.camera_center-self.track_width//2, scan_line+20), (0,0,255), self.line_width_params)
        cv2.line(img, (self.camera_center+self.track_width//2, scan_line-20), (self.camera_center+self.track_width//2, scan_line+20), (0,0,255), self.line_width_params)

    def plot(self):
        img = self.img.copy()

        self.plot_scan_line(img, self.scan_line_d)
        self.plot_scan_line(img, self.scan_line_u)

        if self.left != 0 and self.right != 0:
            cv2.putText(img, f"{self.right - self.left:d}", (self.width - 80, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)

        cv2.line(img, (self.line_pos, 0), (self.line_pos, self.height), (255,0,255), self.line_width_indicator)
        if(self.left):
            cv2.line(img, (self.left, 0), (self.left, self.height), (0,255,0), self.line_width_indicator)
        if(self.right):
            cv2.line(img, (self.right, 0), (self.right, self.height), (0,255,0), self.line_width_indicator)

        if self.frame_cnt == 0:
            self.time = time.time()
        self.frame_cnt = (self.frame_cnt + 1) % 10
        if self.frame_cnt == 0:
            self.fps = 10 / (time.time() - self.time)
        cv2.putText(img, f"{self.fps:.3f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,0), 2)

        if self.manual_mode:
            set_speed = controller.get_default_speed()
            set_steering = controller.get_default_steering()
            cv2.putText(img, f"Manual mode, speed = {self.speed:.2f} (set: {set_speed:.2f}/{set_steering:.2f})", (10, self.height-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
        else:
            cv2.putText(img, f"Autodr mode, speed = {self.speed:.2f}", (10, self.height-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)

        cv2.imshow("Visualizer", img)
        key = cv2.waitKey(33)

        if key == ord('q'):
            self.terminate = True
        elif key == ord('m'):
            self.manual_mode = not self.manual_mode
            self.pub_manual_mode.publish(Bool(data=self.manual_mode))

if __name__ == "__main__":
    cv2.namedWindow("Visualizer")
    rospy.init_node("visualizer")

    default_speed = rospy.get_param("~default_speed", 200)

    controller = Controller(default_speed = default_speed)
    listener = Listener(on_press=controller.on_press, on_release=controller.on_release)
    listener.start()

    visualizer = Visualizer(controller)
    try:
        visualizer.start()
    except rospy.ROSInterruptException:
        pass

    cv2.destroyAllWindows()

    rospy.loginfo("Program end.")
    listener.stop()
