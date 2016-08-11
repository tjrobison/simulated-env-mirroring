#!/usr/bin/env python

import sys
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

import cv2
from cv_bridge import CvBridge, CvBridgeError

SIM_TOPIC = '/simbot1/cam_bot/camera1/image_raw'
BOT_TOPIC = '/bot_image_repub'

"""
Not much to say here. A simple script that republishes both the simulated and CameraViewRelay
robot camera streams into a single image stream. The robot video stream is republished via
the image_transport package so we can handle a compressed stream in a raw format. Rospy doesn't
offer python support for compresed image streams.
"""

class CameraViewRelay:

	def __init__(self):
		self.bridge = CvBridge()
		self.overlay_flag = True

		self.sim_image_sub = rospy.Subscriber(SIM_TOPIC, Image, self.sim_cb)
		self.robot_image_sub = rospy.Subscriber(BOT_TOPIC, Image, self.bot_cb)
		self.enable_overlay_sub = rospy.Subscriber('/enable_overlay', Int32, self.enable_overlay_callback)

		self.image_repub = rospy.Publisher('image_repub', Image, queue_size=10)

	def enable_overlay_callback(self, msg):
		if msg.data == 1:
			self.overlay_flag = True
		elif msg.data == 0:
			self.overlay_flag = False

	def sim_cb(self, image):
		if self.overlay_flag:
			self.image_repub.publish(image)

	def bot_cb(self, image):
		if not self.overlay_flag:
			self.image_repub.publish(image)


if __name__ == "__main__":
	rospy.init_node('camera_view')
	camera_view = CameraViewRelay()
	rospy.spin()
