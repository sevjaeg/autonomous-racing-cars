#!/usr/bin/env python3
from __future__ import print_function
import sys
import math
import numpy as np
import os
import tf

#ROS Imports
import rospy
import geometry_msgs
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry

from skimage import io, morphology, img_as_ubyte


class planner:
    def __init__(self):

        # Topics & Subscriptions, Publishers
        map_topic = '/map'
        path_topic = '/path'

        self.map_sub = rospy.Subscriber(map_topic, OccupancyGrid, self.map_callback, queue_size=1)
        self.path_pub = rospy.Publisher(path_topic, Path, queue_size=10)

        # Parameters
        self.occupied_thresh = 0.65
        self.safety_dist = rospy.get_param('/planner/wall_distance', 0.4)  # safety foam radius (m)
        self.path_sparseness = rospy.get_param('/planner/path_sparseness', 20)  # distance between waypoints (in pixel)
        self.map_name = rospy.get_param('/planner/map_name', "")  # safety foam radius (m)

        # Path for saving maps
        self.savepath = os.path.dirname(os.path.realpath(__file__)) + "/../maps"
        if not os.path.exists(self.savepath):
            os.makedirs(self.savepath)

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
        
        map = self.preprocess_map(data)
    
        driveable_area = self.get_driveable_area(data, map)
        self.save_map(driveable_area, "1_drivable_area")

        driveable_area = self.add_safety_foam(driveable_area)
        self.save_map(driveable_area, "2_drivable_area_safety")

        # TODO consider potential fields
        # x = np.tile(np.linspace(0, driveable_area.shape[0]-1, driveable_area.shape[0]), driveable_area.shape[1]).reshape((driveable_area.shape[0], driveable_area.shape[1]))
        # y = np.transpose(x)
        # distances = np.where(driveable_area, np.sqrt((x - self.start[0]) ** 2 + (y - self.start[1]) ** 2), driveable_area)
        # distances = distances/np.max(distances)
        # self.save_map(distances, num=1)

        distances = self.get_distances(driveable_area)
        distances_print = distances.copy()
        distances_print[driveable_area == False] = 0.0  # remove high values outside driveable area
        self.save_map(distances_print/np.max(distances_print), "3_distances")

        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.get_rostime() 

        path = self.calculate_path(map, distances, path_msg)
        self.save_map(path, "4_path")

    def get_distances(self, driveable_area):
        distances = np.full(self.shape, 9999.9, dtype=float)
        done_map = (~(driveable_area)).copy()

        # Mark starting line as done, distance 0
        done_map[self.start_pixel[0], self.start_pixel[1]] = True
        distances[self.start_pixel[0], self.start_pixel[1]] = 0.0

        x = self.start_pixel[0]+1
        while driveable_area[x, self.start_pixel[1]]:
            done_map[x, self.start_pixel[1]] = True
            distances[x, self.start_pixel[1]] =  abs(x-self.start_pixel[0])
            x = x + 1
        x = self.start_pixel[0]-1
        while driveable_area[x, self.start_pixel[1]]:
            done_map[x, self.start_pixel[1]] = True
            distances[x, self.start_pixel[1]] =  abs(x-self.start_pixel[0])
            x = x - 1

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
        
    def update_distance(self, distances, done_map, x, y):
        print(x, y)
        min_dist, min_x, min_y = self.get_min_dist_neighbour(done_map, distances, x, y)
        done_map[x, y] = True
        new_dist = np.linalg.norm(np.array([x, y]) - np.array([min_x, min_y])) + min_dist
        if new_dist < distances[x, y]:
            distances[x, y] = new_dist

        all_done = True
        for [step_x, step_y] in [[0, 1],[0, -1],[1, 0],[-1, 0],[1, 1],[-1, 1],[1, -1],[-1, -1]]:
            if not done_map[x+step_x, y+step_y]:
                all_done = False
                self.update_distance(distances, done_map, x+step_x, y+step_y)
        if all_done:
            print("return at", x, y)
            return

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
        binary_image = morphology.binary_erosion(driveable_area, footprint=morphology.footprints.disk(radius=self.safety_dist/self.resolution, dtype=np.bool))
        return binary_image

    def get_min_dist_neighbour(self, done_map, distances, x, y):
        min_dist = 9999.9
        for [step_x, step_y] in [[0, 1],[0, -1],[1, 0],[-1, 0],[1, 1],[-1, 1],[1, -1],[-1, -1]]:
            if done_map[x+step_x, y+step_y]:
                if distances[x+step_x, y+step_y] < min_dist:
                    min_dist = distances[x+step_x, y+step_y]
                    x_min = x+step_x
                    y_min = y+step_y
        return min_dist, x_min, y_min

    def calculate_path(self, map, distances, path_msg):
        x = self.start_pixel[0]
        y = self.start_pixel[1]+2
        distance = distances[x, y]
        last_distance = distance

        # TODO close loop in a smarter way
        # xx = x
        # while driveable_area[xx, self.start[1]]:
        #     print(xx, y, distances[xx, y])
        #     xx = xx + 1
        # xx = x-1
        # while driveable_area[xx, y]:
        #     print(xx, y, distances[xx, y])
        #     xx = xx - 1

        path = map.copy()

        path[x,y] = 0.0
        for [step_x, step_y] in [[0, 1],[1, 0],[-1, 0],[1, 1],[-1, 1]]:
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

        self.add_pose_to_path(path_msg, x, y, last_x, last_y)

        while(distance > 0.0):
            path[x,y] = 0.0
            if last_distance-distance >= self.path_sparseness:
                self.add_pose_to_path(path_msg, x, y, last_x, last_y)
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
        print(x, y)

        self.path_pub.publish(path_msg)
        return path

    def add_pose_to_path(self, path_msg, x, y, last_x, last_y):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = rospy.get_rostime() 
        pose_msg.pose.position.x = self.resolution*y + self.start_position[0]
        pose_msg.pose.position.y = self.resolution*x + self.start_position[1]
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0.0, 0.0, math.atan2(x-last_x, y-last_y)))
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
