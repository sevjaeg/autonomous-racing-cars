#!/usr/bin/env python3
from __future__ import print_function
from operator import ne
import sys
import math
from tkinter.tix import MAX
import numpy as np
import os
import tf

#ROS Imports
import rospy
import geometry_msgs

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from skimage import io, morphology, img_as_ubyte
from skimage.filters import gaussian

MAX_FLOAT = 99999.9


class planner:
    def __init__(self):

        # Topics & Subscriptions, Publishers
        map_topic = '/map'
        path_topic = '/path'

        self.map_sub = rospy.Subscriber(map_topic, OccupancyGrid, self.map_callback, queue_size=1)
        self.path_pub = rospy.Publisher(path_topic, Path, queue_size=10, latch=True)

        # Parameters

        # False: basic approach as in assignment
        # True:  gradient descent based path calculation with larger step size
        self.USE_GRADIENT_DESCENT = rospy.get_param('/planner/use_gradient_descent', True)
        
        self.occupied_thresh = 0.65  # map pixels are considered occupied if larger than this value

        self.safety_dist = rospy.get_param('/planner/wall_distance', 0.6)  # safety foam radius (m)
        self.waypoint_distance = rospy.get_param('/planner/waypoint_distance', 0.3)  # distance between waypoints (in meters)
        self.map_name = rospy.get_param('/planner/map_name', "")  # name for the exported map

        # Path for saving maps
        self.savepath = os.path.dirname(os.path.realpath(__file__)) + "/../maps"
        if not os.path.exists(self.savepath):
            os.makedirs(self.savepath)

        self.path_points = []
        self.path_length = 0

    def map_callback(self, data):
        """
        Process the map to pre-compute the path to follow and publish it
        """
        rospy.loginfo("map info from OccupancyGrid message:\n%s", data.info)
        
        self.shape = (data.info.width, data.info.height)
        self.start_position = (data.info.origin.position.x,
                               data.info.origin.position.y)
        self.resolution = data.info.resolution
        self.start_pixel = (int(-self.start_position[0]/self.resolution),
                            int(-self.start_position[1]/self.resolution))

        self.waypoint_distance = rospy.get_param('/planner/waypoint_distance', 0.8)/self.resolution
        
        map = self.preprocess_map(data)
    
        driveable_area = self.get_driveable_area(data, map)
        self.save_map(driveable_area, "1_drivable_area")

        driveable_area = self.add_safety_foam(driveable_area)
        self.save_map(driveable_area, "2_drivable_area_safety")

        shortest_lap = MAX_FLOAT
        for start_x in range(self.start_line_left+1, self.start_line_right, 4):
            distances = self.get_distances(driveable_area, start_x)
            lap_length = distances[start_x, self.start_pixel[1] + 2]
            rospy.loginfo("Start @" + str(start_x) + ": track length " + str(lap_length))
            if lap_length <= shortest_lap:
                shortest_lap = lap_length
                best_start_x = start_x
                shortest_distances = distances

        self.start_x = best_start_x
        rospy.loginfo("Shortest Lap: Start @" + str(self.start_x) + ": track length " + str(shortest_lap))

        distances = shortest_distances
        distances_print = distances.copy()
        distances_print[driveable_area == False] = 0.0  # remove high values outside driveable area
        self.save_map(distances_print/np.max(distances_print), "3_distances")

        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.get_rostime() 

        path = self.calculate_path(map, distances, path_msg)
        self.save_map(path, "4_path")

        # rospy.loginfo(self.path_points)
        rospy.loginfo("Path Length: %f", self.path_length)

    def get_distances(self, driveable_area, start_x):
        distances = np.full(self.shape, MAX_FLOAT, dtype=float)
        done_map = (~(driveable_area)).copy()

        # Mark starting line as done, distance 0 for start position
        y = self.start_pixel[1]
        for x in range(self.start_line_left, self.start_line_right+1):
            done_map[x, y] = True
            distances[x, y] =  abs(x-start_x)

        queue = []
        queue.append((self.start_pixel[0], self.start_pixel[1]-1))

        queued_map = done_map.copy()
        while queue:
            (x, y) = queue.pop(0)
            min_dist, min_x, min_y = self.get_min_dist_neighbour(done_map, distances, x, y)
            done_map[x, y] = True
            new_dist = np.linalg.norm(np.array([x, y]) - np.array([min_x, min_y])) + min_dist
            if new_dist < distances[x, y]:
                distances[x, y] = new_dist
            for [step_x, step_y] in [[0, 1],[0, -1],[1, 0],[-1, 0],[1, 1],[-1, 1],[1, -1],[-1, -1]]:
                new_x = x+step_x
                new_y = y+step_y
                if not done_map[new_x, new_y] and not queued_map[new_x, new_y]:
                    done_map[new_x, new_y] = True
                    queue.append((new_x, new_y))
        return distances

    def preprocess_map(self, data):
        map_data = np.asarray(data.data).reshape((data.info.width, data.info.height)) # parse map data into 2D numpy array
        map_normalized = map_data / np.amax(map_data.flatten()) # normalize map
        map_binary = map_normalized < (self.occupied_thresh) # make binary occupancy map
        return map_binary

    def get_driveable_area(self, data, map_binary):
        driveable_area = morphology.flood_fill(
            image = 1*map_binary,
            seed_point = self.start_pixel,
            new_value = -1,
        )
        driveable_area = driveable_area < 0
        return driveable_area

    def add_safety_foam(self, driveable_area):
        binary_image = morphology.binary_erosion(driveable_area, footprint=morphology.footprints.disk(radius=self.safety_dist/self.resolution, dtype=bool))

        x = self.start_pixel[0]+1
        while binary_image[x, self.start_pixel[1]]:
            x = x + 1
        self.start_line_right = x-1
        
        x = self.start_pixel[0]-1
        while binary_image[x, self.start_pixel[1]]:
            x = x - 1
        self.start_line_left = x+1
        return binary_image

    def get_min_dist_neighbour(self, done_map, distances, x, y):
        min_dist = MAX_FLOAT
        for [step_x, step_y] in [[0, 1],[0, -1],[1, 0],[-1, 0],[1, 1],[-1, 1],[1, -1],[-1, -1]]:
            if done_map[x+step_x, y+step_y]:
                if distances[x+step_x, y+step_y] < min_dist:
                    min_dist = distances[x+step_x, y+step_y]
                    x_min = x+step_x
                    y_min = y+step_y
        return min_dist, x_min, y_min

    def calculate_path(self, map, distances, path_msg):
        # Start a little above the starting line as the line has distance 0, the pixes above have distance 1
        START_OFFSET = 2

        x = self.start_x
        y = self.start_pixel[1] + START_OFFSET
        self.path_length += 1.0 * START_OFFSET
        path = map.copy()

        distance = distances[x, y]
        last_distance = distance

        curvature =  np.full(self.shape, MAX_FLOAT, dtype=float)

        path = map.copy()

        best_x = -1
        best_y = -1

        # no curvature initially

        path[x,y] = 0.0
        self.path_points.append((x, y))
        # Fist step: only steps up (or sideways) as first gradient points towards line
        for [step_x, step_y] in [[0, 1],[1, 0],[-1, 0],[1, 1],[-1, 1]]:  
            new_x = x + step_x
            new_y = y + step_y
            if distances[new_x, new_y] < distance:
                distance = distances[new_x, new_y]
                best_x = new_x
                best_y = new_y

        if best_x == -1 and best_y == -1:
            rospy.logerr("No valid path found from this starting position. Consider increasing START_OFFSET.")
            exit(-1)
        
        last_x = x
        last_y = y
        x = best_x
        y = best_y

        # Not broadcasting a position at the start reduces problems with a non-central start
        # self.add_pose_to_path(path_msg, x, y, last_x=last_x, last_y=last_y)

        if not self.USE_GRADIENT_DESCENT:
            while(distance > 0.0):
                path[x,y] = 0.0
                self.path_length += np.linalg.norm(np.array([x, y]) - np.array([last_x, last_y]))
                self.path_points.append((x, y))
                if last_distance-distance >= self.waypoint_distance:
                    self.add_pose_to_path(path_msg, x, y, last_x=last_x, last_y=last_y)
                    last_distance = distance

                for [step_x, step_y] in [[0, 1], [0, -1],[1, 0],[-1, 0],[1, 1],[-1, 1],[1, -1],[-1, -1]]:
                    new_x = x+step_x
                    new_y = y+step_y
                    if distances[new_x, new_y] < distance:
                        distance = distances[new_x, new_y]
                        best_x = new_x
                        best_y = new_y
                last_x = x
                last_y = y
                x = best_x
                y = best_y
        else:  # work in progress for gradient-based path
            STEP_SIZE = 5 * np.sqrt(2)
            SIGMA = 0 # 0.25
            EPS = 0.4 # 0.033
            EPS2 = 0.2
            MAX_REASONABLE_ANGLE = math.radians(60)

            g_x, g_y = np.gradient(distances)
            orientation = np.arctan2(g_x, g_y)+np.pi

            # Blur orientation field: problem around starting line
            orientation = gaussian(orientation, sigma=SIGMA)
            self.save_map((orientation+np.pi)/(np.max(orientation)* 2 * np.pi), "orientation")
            dir = orientation[x, y]

            i = 0
            # while(i < 2000):
            while(distance > STEP_SIZE-1):
                
                # rospy.loginfo(str(x) +" "+ str(y) + " dist: " + str(distance) + " orientation: " + str(math.degrees(orientation[x, y])))
                path[x,y] = 0.0
                self.path_length += np.linalg.norm(np.array([x, y]) - np.array([last_x, last_y]))
                self.path_points.append((x, y))
                
                if last_distance-distance >= self.waypoint_distance:
                    self.add_pose_to_path(path_msg, x, y, orientation=orientation[x, y])
                    # TODO use odometric distance?
                    last_distance = distance

                # TODO implement curve radius function
                LOOKAHEAD_DISTANCE = 5
                DISTANCE_WEIGHT = 0.7
                CURVATURE_WEIGHT = 1-DISTANCE_WEIGHT

                # get_reasonable_steps: return legal pixels (given position, step size, angular range, init dir)
                # get_reasonable_steps((x, y), STEP_SIZE, np.pi, last_direction)

                last_direction = dir

                if np.abs(dir - orientation[x, y]) < MAX_REASONABLE_ANGLE and \
                   np.abs(orientation[int(x + STEP_SIZE * np.sin(orientation[x,y])+0.5),
                          int(y + STEP_SIZE * np.cos(orientation[x,y])+0.5)] - orientation[x, y]
                         ) < MAX_REASONABLE_ANGLE:
                    dir = EPS * dir + (1-EPS - EPS2) * orientation[x, y] + EPS2 * orientation[int(x + STEP_SIZE * np.sin(orientation[x,y])+0.5), int(y + STEP_SIZE * np.cos(orientation[x,y])+0.5)]
                    next_step = STEP_SIZE
                else:
                    dir = orientation[x, y]
                    next_step = np.sqrt(2)

                new_x = int(x + next_step * np.sin(dir)+0.5)
                new_y = int(y + next_step * np.cos(dir)+0.5)

                # keep path in drivable area
                j = 1
                while distances[new_x, new_y] == MAX_FLOAT:
                    new_x = int(x + (next_step-j*np.sqrt(2)) * np.sin(dir)+0.5)
                    new_y = int(y + (next_step-j*np.sqrt(2)) * np.cos(dir)+0.5)
                    j = j + 1
                    if (next_step-j*np.sqrt(2)) <= 0 or (next_step-j*np.sqrt(2)) <= 0:
                        break

                last_x = x
                last_y = y

                x = new_x
                y = new_y
                distance = distances[x, y]
                i += 1

        # TODO clean up path
        
        self.path_pub.publish(path_msg)
        return path

    def add_pose_to_path(self, path_msg, x, y, last_x=-1, last_y=-1, orientation=0.0):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = rospy.get_rostime() 
        pose_msg.pose.position.x = self.resolution*y + self.start_position[0]
        pose_msg.pose.position.y = self.resolution*x + self.start_position[1]
        pose_msg.pose.position.z = 0.0

        if last_x == -1 and last_y == -1:
            orientation = orientation
        else:
            orientation = math.atan2(x-last_x, y-last_y)

        pose_msg.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0.0, 0.0, orientation))
        path_msg.poses.append(pose_msg)

    def save_map(self, map, name=""):
        save_path = self.savepath + '/' +  self.map_name + "_" + name + '.png'
        map_image = np.rot90(np.flip(map, 0), 1) # flip and rotate for rendering as image in correct direction
        io.imsave(save_path, img_as_ubyte(map_image), check_contrast=False) # save image, just to show the content of the 2D array for debug purposes
        rospy.loginfo("map saved to %s", save_path)

def main(args):
    rospy.init_node("planner_node", anonymous=True)
    rfgs = planner()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
