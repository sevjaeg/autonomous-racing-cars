#!/usr/bin/env python
from __future__ import print_function
from platform import java_ver
from re import T
import sys
import math

#ROS Imports
import rospy
# from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import MarkerArray, Marker

# DISPARITY EXTENDER PARAMS
BASIC_VELOCITY = False  # simple velocity scheme from the assignment sheet, otherwise more aggressive behaviour
MAX_SPEED = rospy.get_param('/reactive/max_speed', 1)  # m/s  (only without basic velocity)
MIN_SPEED = rospy.get_param('/reactive/min_speed', 1)  # m/s  (only without basic velocity)
# The minimum distance that is considered a disparity
DISPARITY = rospy.get_param('/reactive/disparity', 0.15)  # m
# Safety distance to maintain from a disparity
SAFETY_DISTANCE = rospy.get_param('/reactive/safety_distance', 0.52)  # m
# When the distance in front is less than this, the car turns
MIN_DISTANCE_TO_TURN = 300

# CONSTANTS
RAYS_PER_DEGREE = 4
MAX_STEERING_ANGLE = 24  # degrees

viz_arr = None
lidar_data = None

class DisparityExtender:
    def __init__(self):
        rospy.loginfo("Hello from the reactive node")

        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'
        lidar_viz_topic0 = '/lidar_viz0'
        lidar_viz_topic1 = '/lidar_viz1'
        lidar_viz_topic2 = '/lidar_viz2'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
        self.lidar_viz0_pub = rospy.Publisher(lidar_viz_topic0, LaserScan, queue_size=10)
        self.lidar_viz1_pub = rospy.Publisher(lidar_viz_topic1, LaserScan, queue_size=10)
        self.lidar_viz2_pub = rospy.Publisher(lidar_viz_topic2, LaserScan, queue_size=10)

    def lidar_callback(self, data):
        global viz_arr
        #viz_arr = list(data.intensities)
        filtered_ranges = self.filter_ranges(data)
        processed_ranges = self.process_disparities(filtered_ranges)
        self.navigate_farthest(processed_ranges)

    # Filter the lidar data to obtain only lasers in range (-90, +90)
    def filter_ranges(self, data):
        global viz_arr, lidar_data
        lidar_data = data
        lasers = len(data.ranges) * 120 / 270
        skipped = (len(data.ranges) - lasers) / 2
        filtered = data.ranges[int(skipped): int(skipped + lasers)]
        #viz_arr[0: int(skipped)] = [0] * int(skipped)
        #viz_arr[int(skipped + lasers):] = [0] * int(skipped)
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

                    # processed_ranges[int(i - safety_rays): (i + 1)] = [ranges[i + 1]] * int(safety_rays)
                    for j in range(i - safety_rays, i+1):
                        if j < 0 or j >= len(processed_ranges):
                            continue
                        processed_ranges[j] = min(ranges[i+1], ranges[j])

                    #viz_arr[i - safety_rays: (i + 1)] = [0] * int(safety_rays)
                else:
                    # Left edge
                    safety_rays = int(self.calculate_angle(ranges[i])+0.5)

                    # processed_ranges[i: int(i + safety_rays + 1)] = [ranges[i]] * int(safety_rays)
                    for j in range(i, i + safety_rays + 1):
                        if j < 0 or j >= len(processed_ranges):
                            continue
                        processed_ranges[j] = min(ranges[i], ranges[j])

                    #viz_arr[i: i + safety_rays + 1] = [0] * int(safety_rays)
                    
                    # Skip the edited values
                    i += safety_rays - 1
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
        
	#print(farthest)	

	if straight < MIN_DISTANCE_TO_TURN:
            angle = self.get_angle(ranges, farthest)
        else:
            angle = 0.0
        # velocity = self.calculate_velocity(straight)
	
	velocity = straight * 1.5
	
	v_max = 10
	v_min = 1.3

	if velocity > v_max:
		velocity = v_max
	if velocity < v_min:
		velocity = v_min

	#print("angle:" + str(angle))
	#print("vel:  " + str(velocity))

	angle = 0.32 * angle
	
        # Angle clipping. Probably handled by VESC anyway, but improves plot readablity
        if(angle > MAX_STEERING_ANGLE):
            angle = MAX_STEERING_ANGLE
        elif (angle < -MAX_STEERING_ANGLE):
            angle = -MAX_STEERING_ANGLE
	
	#print("angle:" + str(angle))

        if BASIC_VELOCITY:
            if (abs(angle) > 10):
                velocity = 0.8
            else:
                velocity = 1.2
        # else:
            # TODO find something smarter
            # velocity = MAX_SPEED - (MAX_SPEED-MIN_SPEED)/MAX_STEERING_ANGLE * angle
            #if velocity < MIN_SPEED:
            #    velocity = MIN_SPEED

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = math.radians(angle)
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)
        # self.set_intensities(farthest)

    def get_angle(self, ranges, i):
        #print("len: " + str(len(ranges)))
	return (1.0 * i / len(ranges)) * 180 - 90

    def calculate_velocity(self, straight_distance):
        return straight_distance * 0.5

    # def set_intensities(self, farthest):
    #     global viz_arr, lidar_data
    #     i = farthest 
    #     while i < len(viz_arr):
    #         if viz_arr[i] == 0:
    #             break
    #         viz_arr[i] = 1
    #         i += 1
    #     i = farthest
    #     while i > 0:
    #         if viz_arr[i] == 0:
    #             break
    #         viz_arr[i] = 1
    #         i -= 1
    #     i = 0
    #     while i < len(viz_arr):
    #         if not (viz_arr[i] == 0.0) and not (viz_arr[i] == 1.0):
    #             viz_arr[i] = 0.5
    #         i += 1
    #     pub = lidar_data
    #     pub.intensities = viz_arr
    #     self.lidar_viz0_pub.publish(pub)
    #     self.lidar_viz1_pub.publish(pub)
    #     self.lidar_viz2_pub.publish(pub)
        

def main(args):
    rospy.init_node("reactive_node", anonymous=True)
    dex = DisparityExtender()

    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)