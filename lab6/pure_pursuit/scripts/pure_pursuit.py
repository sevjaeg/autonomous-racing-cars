#!/usr/bin/env python3
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
import math
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import tf2_ros
import tf2_geometry_msgs

from skimage import io, morphology, img_as_ubyte

# Parameters of Pure Pursuit
LOOKAHEAD_DISTANCE = rospy.get_param('/pure_pursuit/lookahead_distance', 1)
STEERING_GAIN = rospy.get_param('/pure_pursuit/steering_gain', 0.5)
BASIC_VELOCITY = rospy.get_param('/pure_pursuit/basic_velocity', False)
VELOCITY = 2
MAX_SPEED = rospy.get_param('/pure_pursuit/max_speed', 4)  # m/s  (only without basic velocity)
MIN_SPEED = rospy.get_param('/pure_pursuit/min_speed', 1.5)  # m/s  (only without basic velocity)
VELOCITY_GAIN = rospy.get_param('/pure_pursuit/velocity_gain', 1.0)
# Visualization parameters
VISUALIZATION = rospy.get_param('/pure_pursuit/visualization', False)
LOG_OUTPUT = rospy.get_param('/pure_pursuit/log_output', True)
LOG_OUTPUT_LENGTH = rospy.get_param('/pure_pursuit/log_output_length', 100) # only every 100th message
QUEUE_LENGTH = rospy.get_param('/pure_pursuit/queue_length', 100) # number of message points displayed

class pure_pursuit:


    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/nav'
        map_topic = '/map'
        odom_topic = '/odom'
        path_topic = '/path'
        marker_goal_topic = '/marker_goal'
        trajectory_topic = '/trajectory'

        self.path_poses = None
        self.velocity = VELOCITY
        self.log_counter = 0
        
        if VISUALIZATION:
            self.init_trail()

        self.path_sub = rospy.Subscriber(path_topic, Path, self.path_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=1)
        if not BASIC_VELOCITY:
            self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback, queue_size=1) # optional
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)
        self.marker_goal_pub = rospy.Publisher(marker_goal_topic, Marker, queue_size=1)
        self.trajectory_pub = rospy.Publisher(trajectory_topic, Marker, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tf_buffer)


    def odom_callback(self, data):
        if self.log_counter % int(LOG_OUTPUT_LENGTH) == 0:
            if LOG_OUTPUT:
                rospy.loginfo('x: %s, y: %s', data.pose.pose.position.x, data.pose.pose.position.y)
            if VISUALIZATION:
                self.visualize_trail(data)
        self.log_counter += 1
        
        self.pursuit_algorithm(data)

    def path_callback(self, data):
        self.path_poses = data.poses
        print("Map received")

    # Calculate the velocity based on the distance in front of the car
    def lidar_callback(self, data):
        velocity = data.ranges[540] * VELOCITY_GAIN
        if velocity > MAX_SPEED:
            velocity = MAX_SPEED
        if velocity < MIN_SPEED:
            velocity = MIN_SPEED
        self.velocity = velocity

    def pursuit_algorithm(self, data):
        current = data.pose.pose.position
        if self.path_poses is None:
            return
        goal = self.get_goal(current)
        if VISUALIZATION:
            self.visualize_goal(goal)
        local_position = self.get_local_position(goal)
        angle = self.get_steering_angle(local_position)
        self.publish_drive_msg(self.velocity, angle)

    # Return the index and the distance of the nearest point in the path
    def find_nearest(self, current_position):
        # I didn't use math.inf due to compatibility issues
        min_dist = -1
        min_index = 0
        for index, pose in enumerate(self.path_poses):
            path_position = pose.pose.position
            distance = math.dist([current_position.x, current_position.y], [path_position.x, path_position.y])
            if min_dist == -1:
                min_dist = distance
            if distance < min_dist:
                min_dist = distance
                min_index = index
        return min_index, min_dist

    # Returns the position of the goal point in the path
    def get_goal(self, current_position):
        index, distance = self.find_nearest(current_position)
        pose = self.path_poses[index]
        # Avoid infinite loop
        iterations = 0
        while distance < LOOKAHEAD_DISTANCE and iterations <= len(self.path_poses):
            if index < len(self.path_poses) - 1:
                index += 1
            else:
                index = 0
            pose = self.path_poses[index]
            path_position = pose.pose.position
            distance = math.dist([current_position.x, current_position.y], [path_position.x, path_position.y])
            iterations += 1
        return pose

    # Returns a list of 2 elements: x position, y position
    def get_local_position(self, goal):
        local_pose = self.transform_pose(goal, 'map', 'base_link')
        return local_pose.position

    # Get the steering angle from the curvature calculated with the formula GAMMA = 2y/L^2
    def get_steering_angle(self, position):
        y = position.y
        curvature = (2 * y) / (LOOKAHEAD_DISTANCE * LOOKAHEAD_DISTANCE)
        steering_angle = STEERING_GAIN * curvature
        return steering_angle

    def publish_drive_msg(self, velocity, angle):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    def transform_pose(self, input, from_frame, to_frame):
        input_pose = input.pose

        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.pose = input_pose
        pose_stamped.header.frame_id = from_frame
        pose_stamped.header.stamp = rospy.Time()

        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            output_pose_stamped = self.tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
            return output_pose_stamped.pose

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

    def visualize_goal(self, goal):
        # print(goal)
        marker_goal = Marker()
        marker_goal.header.frame_id = 'map'
        marker_goal.header.stamp = rospy.Time.now()
        marker_goal.type = 2
        marker_goal.pose = goal.pose
        marker_goal.scale.x = .3
        marker_goal.scale.y = .3
        marker_goal.scale.z = .3
        marker_goal.color.a = 1
        marker_goal.color.r = 0
        marker_goal.color.g = 0
        marker_goal.color.b = 1
        # marker_goal.color = [1, 0, 1, 0]
        self.marker_goal_pub.publish(marker_goal)

    def init_trail(self):
        self.points = []
        self.trajectory_msg = Marker()
        self.trajectory_msg.header.frame_id = 'map'
        self.trajectory_msg.type = 7        
        self.trajectory_msg.scale.x = .3
        self.trajectory_msg.scale.y = .3
        self.trajectory_msg.scale.z = .3
        self.trajectory_msg.color.a = 1
        self.trajectory_msg.color.r = 1
        self.trajectory_msg.color.g = 1
        self.trajectory_msg.color.b = 0
        self.trajectory_msg.pose.position.x = 0
        self.trajectory_msg.pose.position.y = 0
        self.trajectory_msg.pose.position.z = 0
        self.trajectory_msg.pose.orientation.w = 0
        self.trajectory_msg.pose.orientation.x = 0
        self.trajectory_msg.pose.orientation.y = 0
        self.trajectory_msg.pose.orientation.z = 0

    def visualize_trail(self, data):
        self.trajectory_msg.header.stamp = rospy.Time.now()
        point = Point(data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)
        self.points.append(point)
        if (len(self.points) > QUEUE_LENGTH):
            self.points.pop(0)
        self.trajectory_msg.points = self.points
        self.trajectory_pub.publish(self.trajectory_msg)

def main(args):
    rospy.init_node("pure_pursuit_node", anonymous=True)
    rfgs = pure_pursuit()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
