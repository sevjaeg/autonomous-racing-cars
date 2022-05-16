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

# DISPARITY EXTENDER PARAMS
# The minimum distance that is considered a disparity
DISPARITY = 1
# Safety distance to maintain from a disparity
SAFETY_DISTANCE = 0.3
# When the distance in front is less than this, the car turns
MIN_DISTANCE_TO_TURN = math.inf

# CONSTANTS
RAYS_PER_DEGREE = 4

def reconfig_callback(config, level):
    return config

class FollowTheGap:
    def __init__(self):
        rospy.loginfo("Hello from disparity_extender node")

        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)

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
        i = 0
        while i < (len(ranges) - 1):
            if abs(ranges[i] - ranges[i + 1]) >= DISPARITY:
                # Disparity
                if ranges[i] > ranges[i + 1]:
                    # Right edge
                    safety_rays = self.calculate_angle(ranges[i + 1])
                    processed_ranges[int(i - safety_rays): (i + 1)] = [ranges[i + 1]] * int(safety_rays)
                else:
                    # Left edge
                    safety_rays = self.calculate_angle(ranges[i])
                    processed_ranges[i: int(i + safety_rays + 1)] = [ranges[i]] * int(safety_rays)
                    # Skip the edited values
                    i += int(safety_rays - 1)
            i += 1
        return processed_ranges

    def calculate_angle(self, distance):
        # Calculated as 1080/270
        angle = math.degrees(math.atan(SAFETY_DISTANCE / distance))
        return angle * RAYS_PER_DEGREE

    def navigate_farthest(self, ranges):
        ranges_list = list(ranges)
        straight = ranges_list[int(len(ranges_list)/2)]
        farthest = ranges_list.index(max(ranges_list))
        if straight < MIN_DISTANCE_TO_TURN:
            angle = self.get_angle(ranges, farthest)
        else:
            angle = 0.0
        velocity = self.calculate_velocity(straight)
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = math.radians(angle)
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    def get_angle(self, ranges, i):
        return (i / len(ranges)) * 180 - 90

    def calculate_velocity(self, straight_distance):
        return straight_distance * 0.5

def main(args):
    rospy.init_node("follow_the_gap_node", anonymous=True)
    ftg = FollowTheGap()

    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
