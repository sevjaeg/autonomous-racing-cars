#!/usr/bin/env python3
from __future__ import print_function
from curses import raw
from platform import java_ver
from re import T
import sys
import math

#ROS Imports
import rospy
# from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
# from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped

# DISPARITY EXTENDER PARAMS
VISUALIZATION = rospy.get_param('/reactive/visualization', False)  # quite costly in performance, this leads to problems in the test bench
BASIC_VELOCITY = False  # simple velocity scheme from the assignment sheet, otherwise more aggressive behaviour
MAX_SPEED = rospy.get_param('/reactive/max_speed', 3)  # m/s  (only without basic velocity)
MIN_SPEED = rospy.get_param('/reactive/min_speed', 1.2)  # m/s  (only without basic velocity)
VELOCITY_GAIN = rospy.get_param('/reactive/velocity_gain', 0.6)
STEERING_GAIN = rospy.get_param('/reactive/steering_gain', 0.8)
# The minimum distance that is considered a disparity
DISPARITY = rospy.get_param('/reactive/disparity', 0.2)  # m
# Safety distance to maintain from a disparity
SAFETY_DISTANCE = rospy.get_param('/reactive/safety_distance', 0.42)  # m
LIDAR_ANGULAR_RANGE = rospy.get_param('/reactive/lidar_angular_range', 160)   # degrees
LIDAR_RANGE = rospy.get_param('/reactive/lidar_range', 20) # m


# When the distance in front is less than this, the car turns (so far unused)
MIN_DISTANCE_TO_TURN = 1000

# CONSTANTS
RAYS_PER_DEGREE = 4
MAX_STEERING_ANGLE = 24  # degrees

viz_arr = None
viz_ranges = None
lidar_data = None

class DisparityExtender:
    def __init__(self):
        rospy.loginfo("Hello from the reactive node")

        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'
        lidar_viz_topic = '/lidar_viz'
        drive_viz_topic = '/drive_viz'

        self.blocked = 0
        self.possible = 11
        self.chosen = 6

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
        self.lidar_viz_pub = rospy.Publisher(lidar_viz_topic, LaserScan, queue_size=10)
        self.lidar_drive_viz_pub = rospy.Publisher(drive_viz_topic, PoseStamped, queue_size=10)
        self.skipped = 0
        self.scan = None

    def lidar_callback(self, data):
        global viz_arr
        if VISUALIZATION:
            viz_arr = list(data.intensities)
            self.scan = data
            data.ranges = self.set_ranges()
        filtered_ranges = self.filter_ranges(data)
        processed_ranges = self.process_disparities(filtered_ranges)
        self.navigate_farthest(processed_ranges)

    # Filter the lidar data to obtain only lasers in range (-90, +90)
    def filter_ranges(self, data):
        global viz_arr, lidar_data
        lidar_data = data
        lasers = int(len(data.ranges) * LIDAR_ANGULAR_RANGE / 270)
        self.skipped = int((len(data.ranges) - lasers) / 2)
        filtered = data.ranges[self.skipped: self.skipped + lasers]
        if VISUALIZATION:
            viz_arr[0: self.skipped] = [0] * self.skipped
            viz_arr[self.skipped + lasers:] = [0] * self.skipped
        return filtered

    def process_disparities(self, ranges):
        global viz_arr
        processed_ranges = list(ranges)
        i = 0
        while i < (len(ranges) - 1):
            if abs(ranges[i] - ranges[i + 1]) >= DISPARITY:
                # Disparity
                if ranges[i] > ranges[i + 1]:
                    # Right edge
                    safety_rays = int(self.calculate_angle(ranges[i + 1])+0.5)
                    for j in range(i - safety_rays, i+1):
                        if j < 0 or j >= len(processed_ranges):
                            continue
                        processed_ranges[j] = min(processed_ranges[i+1], processed_ranges[j])

                        if VISUALIZATION:
                            viz_arr[j + self.skipped] = self.blocked
                            viz_ranges[j + self.skipped] = processed_ranges[j]
                else:
                    # Left edge
                    safety_rays = int(self.calculate_angle(ranges[i])+0.5)
                    for j in range(i, i + safety_rays + 1):
                        if j < 0 or j >= len(processed_ranges):
                            continue
                        processed_ranges[j] = min(processed_ranges[i], processed_ranges[j])
                    
                        if VISUALIZATION:
                            viz_arr[j + self.skipped] = self.blocked
                            viz_ranges[j + self.skipped] = processed_ranges[j]
                    
                    # Skip the edited values
                    # i += safety_rays - 1
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

        # reduces oscillations when going at higher speeds
        angle = STEERING_GAIN * angle
        
        # Angle clipping. Probably handled by VESC anyway, but improves plot readablity
        if(angle > MAX_STEERING_ANGLE):
            angle = MAX_STEERING_ANGLE
        elif (angle < -MAX_STEERING_ANGLE):
            angle = -MAX_STEERING_ANGLE

        if BASIC_VELOCITY:
            velocity = 1
        else:
            velocity = straight * VELOCITY_GAIN
            if velocity > MAX_SPEED:
                velocity = MAX_SPEED
            if velocity < MIN_SPEED:
                velocity = MIN_SPEED

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = math.radians(angle)
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)
        if VISUALIZATION:
            self.set_intensities(farthest)
            self.publish_viz()
            drive_viz_msg = PoseStamped()
            drive_viz_msg.header.stamp = rospy.Time.now()
            drive_viz_msg.header.frame_id = "laser"
            drive_viz_msg.pose.orientation.y = velocity * math.sin(math.radians(angle))
            drive_viz_msg.pose.orientation.x = velocity * math.cos(math.radians(angle))
            self.lidar_drive_viz_pub.publish(drive_viz_msg)

    def get_angle(self, ranges, i):
        return (1.0 * i / len(ranges)) * LIDAR_ANGULAR_RANGE - (LIDAR_ANGULAR_RANGE / 2)
        # Funnily enough it works with the code below
        # return (1.0 * i / len(ranges)) * 180 - 90

    def set_intensities(self, farthest):
        global viz_arr, lidar_data
        i = farthest + self.skipped
        while i < len(viz_arr):
            if viz_arr[i] == 0:
                break
            viz_arr[i] = self.chosen
            i += 1
        i = farthest
        while i > 0:
            if viz_arr[i] == 0:
                break
            viz_arr[i] = self.chosen
            i -= 1
        i = 0
        while i < len(viz_arr):
            if not (viz_arr[i] == 0) and not (viz_arr[i] == self.chosen):
                viz_arr[i] = self.possible
            i += 1
        pub = lidar_data
        pub.intensities = viz_arr
        rospy.logdebug(viz_arr)
        self.lidar_viz_pub.publish(pub)

    def set_ranges(self):
        global viz_ranges
        dist = LIDAR_RANGE
        viz_ranges = list(self.scan.ranges)
        i = 0
        while i < len(self.scan.ranges):
            if viz_ranges[i] > dist:
                viz_ranges[i] = dist
            i += 1
        return viz_ranges

    def publish_viz(self):
        pub = self.scan
        pub.intensities = viz_arr
        pub.ranges = viz_ranges
        self.lidar_viz_pub.publish(pub)


def main(args):
    rospy.init_node("reactive_node", anonymous=True)
    dex = DisparityExtender()

    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
    main(sys.argv)
