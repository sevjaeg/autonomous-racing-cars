#!/usr/bin/env python3
from __future__ import print_function
from curses import raw
from platform import java_ver
from re import T
import sys
import math
import time

#ROS Imports
import rospy
from nav_msgs.msg import Odometry

# TIMER PARAMS
START_X1 = rospy.get_param('/timer/x1', 0)  
START_Y1 = rospy.get_param('/timer/y1', -2)
START_X2 = rospy.get_param('/timer/x2', 0)
START_Y2 = rospy.get_param('/timer/y2', 2)

class Timer:
    def __init__(self):
        self.start = time.time()
        self.below_zero = False
        self.lap_counter = 1
        odom_topic = '/odom'
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=1)

    def odom_callback(self, data):
        if data.pose.pose.position.x < 0 and not self.below_zero:
            self.below_zero = True
        if data.pose.pose.position.x > 0 and self.below_zero:
            if data.pose.pose.position.y > START_Y1 and data.pose.pose.position.y < START_Y2:
                lap = time.time()
                rospy.loginfo("Lap %i: %f", self.lap_counter, lap - self.start)
                self.start = lap
                self.below_zero = False
                self.lap_counter += 1
        

def main(args):
    rospy.init_node("timer_node", anonymous=True)
    timer = Timer()

    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
    main(sys.argv)
