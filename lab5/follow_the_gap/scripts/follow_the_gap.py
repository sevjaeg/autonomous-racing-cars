#!/usr/bin/env python3
from __future__ import print_function
from re import T
import sys
import math
from turtle import left, speed

#ROS Imports
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

from dynamic_reconfigure.server import Server

USE_DYNAMIC_RECONFIG = False

if USE_DYNAMIC_RECONFIG:
    from follow_the_gap.cfg import GainsConfig

# FOLLOW THE GAP PARAMS
BASIC_VELOCITY = False  # simple velocity scheme from the assignment sheet, otherwise more aggressive behaviour
MAX_SPEED = rospy.get_param('/follow_the_gap/max_speed', 6.3)  # m/s  (only without basic velocity)
MIN_SPEED = rospy.get_param('/follow_the_gap/min_speed', 1.7)  # m/s  (only without basic velocity)
LOOKAHEAD_DIST_FAST = 3.0  # m  (if the car drives more than 5 m/s)
LOOKAHEAD_DIST_MID = 2.25  # m  (if the car drives more than 3 m/s)
LOOKAHEAD_DIST_SLOW = rospy.get_param('/follow_the_gap/lookahead_dist_slow', 1.5) # m  (if the car drives slower than 3 m/s)

# Car params
MAX_STEERING_ANGLE = math.radians(24)

# Globar variables
prev_time = 0.0
velocity = 0.0

def reconfig_callback(config, level):
    return config

class FollowTheGap:
    """ Implement reactive algorithm on the car """
    def __init__(self):
        rospy.loginfo("Hello from follow_the_gap node")

        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
        
    def getRange(self, data, angle):
        """ Get ranges """
        pass

    def lidar_callback(self, data):
        """call something"""
        rospy.loginfo("scan received")
        pass
        

def main(args):
    rospy.init_node("follow_the_gap_node", anonymous=True)
    ftg = FollowTheGap()

    if USE_DYNAMIC_RECONFIG:
        srv = Server(GainsConfig, reconfig_callback)
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
