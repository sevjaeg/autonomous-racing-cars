#!/usr/bin/env python3
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
import tf
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

from dynamic_reconfigure.server import Server
from wall_follow.cfg import GainsConfig

#PID CONTROL PARAMS
#TODO fine-tuning
kp = math.radians(14)  
kd = math.radians(0.09)
ki = math.radians(0.3)

servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0
prev_time = 0.0
velocity = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270  # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9  # meters
DESIRED_DISTANCE_LEFT = 0.9  # meters
VELOCITY = 2.00  # meters per second
CAR_LENGTH = 0.50  # Traxxas Rally is 20 inches or 0.5 meters

# TODO optimise
THETA = 32  # degrees
LOOKAHEAD_TIME = 1.5  # seconds

def reconfig_callback(config, level):
    global kp
    global ki
    global kd
    kp = config.kp
    ki = config.ki
    kd = config.kd
    rospy.loginfo("Gains set to kp={kp}, ki={ki}, kd={kd}".format(**config))
    return config

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        rospy.loginfo("Hello from wall_follow node")

        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'
        alpha_topic = '/alpha'
        dist_topic = '/dist_left'
        dist_lookahead_topic = '/dist_lookahead'
        error_topic = '/err'
        integral_topic = '/integral'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
        self.alpha_pub = rospy.Publisher(alpha_topic, Float64, queue_size=10)
        self.error_pub = rospy.Publisher(error_topic, Float64, queue_size=10)
        self.dist_left_pub = rospy.Publisher(dist_topic, Float64, queue_size=10)
        self.dist_lookahead_pub = rospy.Publisher(dist_lookahead_topic, Float64, queue_size=10)
        self.integral_pub = rospy.Publisher(integral_topic, Float64, queue_size=10)

        listener = tf.TransformListener()
        

    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        # make sure to take care of nans etc.

        input_angle = angle - 90
        angle_rad = math.radians(input_angle)
        number_of_beams = len(data.ranges)
        array_increment = angle_rad / data.angle_increment

        # TODO check if in FOV
        
        array_index = (number_of_beams / 2 + array_increment if number_of_beams / 2 + array_increment <= number_of_beams else number_of_beams)
        
        result = data.ranges[int(array_index)]
        
        #http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28Python%29
        #The tranformation from the lidar to, idk. possibly base_link, needs to be implemented properly
        # try:
        #     (trans, rot) = listener.lookupTransform('/base_link',data.header.frame_id,rospy.Time(0))
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     # TODO continue
        
        if (result and result > 0.0 and result < 10.0):
            return result
        else:
            rospy.loginfo("Invalid data: " + str(result))
            return 10.0  # TODO find suitable default
        

    def pid_control(self, error):
        global integral
        global prev_error
        global kp
        global ki
        global kd
        global prev_time
        global velocity

        time = rospy.get_time()

        if prev_time > 0.0:
            delta_t = time - prev_time
            delta_err = error - prev_error
            # rospy.loginfo("Invalid data: " + str(result))
            if abs(velocity) > 0.1:  # TODO useless
                integral += delta_t * error
            # TODO anti windup (if I control is used)
        else:  # ignore first time update
            delta_t = 1  # avoid division by 0
            delta_err = 0

        angle = kp * error + kd * delta_err/delta_t + ki * integral

        # Probably handled by VESC anyway, but improves plot readiblity
        if(angle > math.radians(24)):
            angle = math.radians(24)
        elif (angle < -math.radians(24)):
            angle = -math.radians(24)

        if(angle > math.radians(20)):
            velocity = 0.5
        elif (angle > math.radians(10)):
            velocity = 1
        else:
            velocity = 1.5

        # TODO fancier velocity heuristic (e.g. considering both angle and distance to obstacle ahead)

        prev_error = error
        prev_time = time

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

        # debugging messages
        error_msg = Float64()
        error_msg.data = error
        self.error_pub.publish(error_msg)

        integral_msg = Float64()
        integral_msg.data = integral
        self.integral_pub.publish(integral_msg)

    def followLeft(self, data, leftDist):
        global velocity

        #Follow left wall as per the algorithm 
        dist_left = self.getRange(data, 180)
        dist_theta = self.getRange(data, 180-THETA)

        alpha = math.atan2((dist_theta*math.sin(math.radians(THETA))), (dist_theta*math.cos(math.radians(THETA))-dist_left)) - math.pi/2

        dist_wall = dist_left*math.cos(alpha)
        dist_wall_lookahead = dist_wall + LOOKAHEAD_TIME * velocity * math.sin(-alpha)
        error = dist_wall_lookahead - leftDist

        # debugging messages
        alpha_msg = Float64()
        alpha_msg.data = alpha
        self.alpha_pub.publish(alpha_msg)

        dist_msg = Float64()
        dist_msg.data = dist_wall
        self.dist_left_pub.publish(dist_msg)

        dist_lookahead_msg = Float64()
        dist_lookahead_msg.data = dist_wall_lookahead
        self.dist_lookahead_pub.publish(dist_lookahead_msg)

        return error 

    def lidar_callback(self, data):
        """ 
        """
        error = self.followLeft(data, DESIRED_DISTANCE_LEFT)    
        self.pid_control(error)
        

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()

    srv = Server(GainsConfig, reconfig_callback)
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
