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

# WARNING: THIS IS ANNOYING
# Unfortunately the dynamic reconfiguration imports do not work with the given
# file names as the package and the file have the same name. This causes a 
# naming conflict. Thus, there are two configurations
# 
# A. Set the flag to true and name this file follow.py (also change CMake and
#    launch files). This allows dynamic reconfiguration
# B. Name this file wall_follow.py and disable the flag. Then, the code can be
#    uploaded to the server provieded on TUWEL
# 
# Unfortunately you might have to rerun catkin_make and delete the files in 
# ~/catkin_ws/devel/lib/python3/dist-packages/wall_follow and ~/catkin_ws/
# ~/catkin_ws/src/wall_follow/scripts/wall_follow when changing between the
# two configurations.
USE_DYNAMIC_RECONFIG = False

if USE_DYNAMIC_RECONFIG:
    from wall_follow.cfg import GainsConfig

# PID CONTROL PARAMS
kp = 0.32
kd = 0 #0.0002
ki = 0

# WALL FOLLOW PARAMS
THETA = 42 # degrees
DYNAMIC_DISTANCE = False  # drives in the middle of the track
# use 1.2 used for rect map
DESIRED_DISTANCE_LEFT = 1.5 # meters (only active if no dynamic distance)
MAX_WALL_DISTANCE = 1.8  # m (only active with dynamic distance, helps with wide curves)
BASIC_VELOCITY = True  # simple velocity scheme from the assignment sheet, otherwise more aggressive behaviour
MAX_SPEED = 6.25  # m/s  (only without basic velocity)
MIN_SPEED = 1.7  # m/s  (only without basic velocity)
LOOKAHEAD_DIST_FAST = 3.0  # m  (if the car drives more than 5 m/s)
LOOKAHEAD_DIST_MID = 2.25  # m  (if the car drives more than 3 m/s)
LOOKAHEAD_DIST_SLOW = 1.5  # m  (if the car drives slower than 3 m/s)

# Car params
MAX_STEERING_ANGLE = math.radians(24)

# Globar variables
prev_error = 0.0 
error = 0.0
integral = 0.0
prev_time = 0.0
velocity = 0.0

def reconfig_callback(config, level):
    global kp
    global ki
    global kd
    global DESIRED_DISTANCE_LEFT
    global THETA
    global DYNAMIC_DISTANCE
    kp = config.kp
    ki = config.ki
    kd = config.kd
    DESIRED_DISTANCE_LEFT = config.dist
    THETA = config.THETA
    DYNAMIC_DISTANCE = config.dynamic_dist
    rospy.loginfo("Gains set to kp={kp}, ki={ki}, kd={kd}".format(**config))
    rospy.loginfo("Wall distance set to {dist} m".format(**config))
    rospy.loginfo("Theta set to {THETA} degrees".format(**config))
    rospy.loginfo("Lookahead distance set to {d_lookahead} m".format(**config))
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
        
    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        # make sure to take care of nans etc.

        input_angle = angle - 90
        angle_rad = math.radians(input_angle)
        number_of_beams = len(data.ranges)
        array_increment = angle_rad / data.angle_increment
        
        array_index = (number_of_beams / 2 + array_increment if number_of_beams / 2 + array_increment <= number_of_beams else number_of_beams)
        
        result = data.ranges[int(array_index)]
        
        if (result and result > 0.0 and result < 10.0):
            return result
        else:
            # rospy.loginfo("Invalid data: " + str(result))
            return 10.0  # TODO better error handling

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
            integral += delta_t * error
        else:  # ignore first time update
            delta_t = 1  # avoid division by 0
            delta_err = 0

        angle = kp * error + kd * delta_err/delta_t + ki * integral

        # Angle clipping. Probably handled by VESC anyway, but improves plot readablity
        if(angle > MAX_STEERING_ANGLE):
            angle = MAX_STEERING_ANGLE
        elif (angle < -MAX_STEERING_ANGLE):
            angle = -MAX_STEERING_ANGLE

        if BASIC_VELOCITY:
            
            if(abs(math.degrees(angle)) > 20):
                velocity = 0.5
            elif (abs(math.degrees(angle)) > 10):
                velocity = 1
            else:
                velocity = 1.5
        else:
            velocity = MAX_SPEED - (MAX_SPEED-MIN_SPEED)/math.degrees(MAX_STEERING_ANGLE) * abs(math.degrees(kp) * error)
            if velocity < MIN_SPEED:
                velocity = MIN_SPEED

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
        global prev_error

        #Follow left wall as per the algorithm 
        dist_left = self.getRange(data, 180)
        dist_THETA = self.getRange(data, 180-THETA)

        # TODO error handling (invalid distances)
        
        max_dist = self.getRange(data, 0) + self.getRange(data, 180)
        
        alpha = math.atan2((dist_THETA*math.sin(math.radians(THETA))), (dist_THETA*math.cos(math.radians(THETA))-dist_left)) - math.pi/2

        dist_wall = dist_left*math.cos(alpha)

        # dynamic lookahead distance. No direct proportionality to avoid noise feedback
        if velocity < 3:
            ld = LOOKAHEAD_DIST_MID
        if velocity > 5:
            ld = LOOKAHEAD_DIST_FAST
        else:
            ld = LOOKAHEAD_DIST_MID

        dist_wall_lookahead = dist_wall + ld * math.sin(-alpha)

        if DYNAMIC_DISTANCE:
            if max_dist < 2 * MAX_WALL_DISTANCE:
                leftDist = max_dist/2
            else:
                leftDist = MAX_WALL_DISTANCE

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

    if USE_DYNAMIC_RECONFIG:
        srv = Server(GainsConfig, reconfig_callback)
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
