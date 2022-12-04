#!/usr/bin/env python3

import numpy as np
import cv2 as cv
import rospkg
import rospy
from tracking.msg import circle
from tracking.msg import circles
from circle_detector import Circle_Detector
from video_publisher import Video_Publisher

if __name__ == '__main__':
	node_name = 'ball_tracking_node'
	rospy.init_node(node_name)
	# try:
	# 	# publish the video
	# 	VIDEO_TOPIC = 'video'
	# 	pub_obj = Video_Publisher('video')
	# 	# Not including RosPack causes error if this script is rosrun from a different directory
	# 	rospack = rospkg.RosPack()
	# 	filename = rospack.get_path("tracking") + "/video/tennis-ball-video.mp4"
	# 	pub_obj.publishVideo(filename)
	# except rospy.ROSException as e:
	# 	rospy.logerr(e)

	# detect the ball
	detector = Circle_Detector('ball_coord')
	detector.detection('video')

