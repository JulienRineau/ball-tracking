#!/usr/bin/env python3

import rospy
from tracking.msg import circle
from tracking.msg import circles
import random


def talker():
    circleData = circle()
    circleArray = circles()
    circle_pub = rospy.Publisher('circles', circles, queue_size=50)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100) # ROS Rate at 5Hz
    while not rospy.is_shutdown():
        circleData.x = random.randint(0, 350)
        circleData.y = random.randint(0, 250)
        circleData.radius = 30
        circleArray.circles.append(circleData)
        rospy.loginfo('x:' + str(circleData.x) + ', y:' + str(circleData.y) + ', radius:' + str(circleData.radius))
        circle_pub.publish(circleArray)
        circleData = circle()
        circleArray = circles()

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass