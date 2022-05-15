#!/usr/bin/env python3
from __future__ import print_function
from re import T
import sys
import math
# from turtle import left, speed

#ROS Imports
import rospy
# from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import MarkerArray, Marker

from dynamic_reconfigure.server import Server

USE_DYNAMIC_RECONFIG = False

if USE_DYNAMIC_RECONFIG:
    from follow_the_gap.cfg import GainsConfig

# DISPARITY EXTENDER PARAMS
BASIC_VELOCITY = False  # simple velocity scheme from the assignment sheet, otherwise more aggressive behaviour
MAX_SPEED = rospy.get_param('/follow_the_gap/max_speed', 6.3)  # m/s  (only without basic velocity)
MIN_SPEED = rospy.get_param('/follow_the_gap/min_speed', 1.7)  # m/s  (only without basic velocity)
LOOKAHEAD_DIST_FAST = 3.0  # m  (if the car drives more than 5 m/s)
LOOKAHEAD_DIST_MID = 2.25  # m  (if the car drives more than 3 m/s)
LOOKAHEAD_DIST_SLOW = rospy.get_param('/follow_the_gap/lookahead_dist_slow', 1.5) # m  (if the car drives slower than 3 m/s)
VISUALIZE_ARRAY = False # TODO: Make it reconfigurable

# The minimum distance that is considered a disparity
DISPARITY = 5
# Safety distance to maintain from a disparity
SAFETY_DISTANCE = 2  # 0.4
# When the distance in front is less than this, the car turns
MIN_DISTANCE_TO_TURN = 6

# Car params
MAX_STEERING_ANGLE = math.radians(24)

# Global variables
prev_time = 0.0
velocity = 2.0

def reconfig_callback(config, level):
    return config

class FollowTheGap:
    """ Implement reactive algorithm on the car """
    def __init__(self):
        rospy.loginfo("Hello from disparity_extender node")

        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'
        marker_array_topic = '/visualization_marker_array'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
        self.marker_array_pub = rospy.Publisher(marker_array_topic, MarkerArray, queue_size=10)

    def lidar_callback(self, data):
        filtered_ranges = self.filter_ranges(data)
        processed_ranges = self.process_disparities(filtered_ranges)
        self.navigate_farthest(processed_ranges)

    # Filter the lidar data to obtain only lasers in range (-90, +90)
    def filter_ranges(self, data):
        lasers = len(data.ranges) * 180 / 270
        skipped = (len(data.ranges) - lasers) / 2
        filtered = data.ranges[int(skipped): int(skipped + lasers)]
        return filtered

    def process_disparities(self, ranges):
        processed_ranges = list(ranges)
        for i in range(len(ranges) - 1):
            if abs(ranges[i] - ranges[i + 1]) >= DISPARITY:
                # Disparity
                if ranges[i] > ranges[i + 1]:
                    # Right edge
                    safety_rays = self.calculate_angle(ranges[i + 1])
                    processed_ranges[int(i - safety_rays): (i + 1)] = [ranges[i + 1]] * int(safety_rays)
                else:
                    # Left edge
                    safety_rays = self.calculate_angle(ranges[i])
                    # print("Rays:" + str(safety_rays))
                    processed_ranges[i: int(i + safety_rays + 1)] = [ranges[i]] * int(safety_rays)
        return processed_ranges

    def calculate_angle(self, distance):
        # Calculated as 1080/270
        RAYS_IN_1_ANGLE = 4
        angle = math.degrees(math.atan(SAFETY_DISTANCE / distance))
        return angle * RAYS_IN_1_ANGLE

    def navigate_farthest(self, ranges):
        ranges_list = list(ranges)
        straight = ranges_list[int(len(ranges_list)/2)]
        farthest = ranges_list.index(max(ranges_list))
        if straight < MIN_DISTANCE_TO_TURN:
            angle = self.get_angle(ranges, farthest)
            # print("Angle: " + str(angle))
            # print("Index: " + str(farthest))
            for i in range(6):
                print(ranges_list[farthest + i - 3])
        else:
            angle = 0.0
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = math.radians(angle)
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    def get_angle(self, ranges, i):
        return (i / len(ranges)) * 180 - 90

def main(args):
    rospy.init_node("follow_the_gap_node", anonymous=True)
    ftg = FollowTheGap()

    if USE_DYNAMIC_RECONFIG:
        srv = Server(GainsConfig, reconfig_callback)
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
