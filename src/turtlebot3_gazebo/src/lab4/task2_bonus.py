#!/usr/bin/env python3
import math
import yaml
import math
import random

import rclpy
import rclpy.logging
from rclpy.node import Node
import rclpy.time

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Twist, PointStamped
from nav_msgs.msg import Path

import cv2
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from scipy.ndimage import convolve
from PIL import Image, ImageOps
from graphviz import Graph
from copy import copy, deepcopy
from collections import defaultdict
import time


## CHANGE MAP LOCATION
map_name = '/home/lucifer/sim_ws/src/turtlebot3_gazebo/maps/map'

# ## CLASS FOR MAP
class Map():
    def __init__(self, map_name):
        self.map_im, self.map_df, self.limits = self.__open_map(map_name)
        self.image_array = self.__get_obstacle_map(self.map_im, self.map_df)

    def __repr__(self):
        fig, ax = plt.subplots(dpi=150)
        ax.imshow(self.image_array,extent=self.limits, cmap=cm.gray)
        ax.plot()
        return ""

    def __open_map(self,map_name):
        # Open the YAML file which contains the map name and other
        # configuration parameters
        f = open(map_name + '.yaml', 'r')
        map_df = pd.json_normalize(yaml.safe_load(f))
        # Open the map image
        # map_name = map_df.image[0]
        im = Image.open(map_name+'.pgm')
        size = 200, 200
        im.thumbnail(size)
        im = ImageOps.grayscale(im)
        # Get the limits of the map. This will help to display the map
        # with the correct axis ticks.
        xmin = map_df.origin[0][0]
        xmax = map_df.origin[0][0] + im.size[0] * map_df.resolution[0]
        ymin = map_df.origin[0][1]
        ymax = map_df.origin[0][1] + im.size[1] * map_df.resolution[0]

        return im, map_df, [xmin,xmax,ymin,ymax]

    def __get_obstacle_map(self,map_im, map_df):
        img_array = np.reshape(list(self.map_im.getdata()),(self.map_im.size[1],self.map_im.size[0]))
        up_thresh = self.map_df.occupied_thresh[0]*255
        low_thresh = self.map_df.free_thresh[0]*255

        for j in range(self.map_im.size[0]):
            for i in range(self.map_im.size[1]):
                if img_array[i,j] > up_thresh:
                    img_array[i,j] = 255
                else:
                    img_array[i,j] = 0
        return img_array

# Class to for graphnode for rrt
class GraphNode:
    """Class to represent nodes in the RRT graph"""
    def __init__(self, x, y):
        self.x = float(x)
        self.y = float(y)
        self.parent = None  # Parent node reference
        self.cost = float('inf')  # Initialize cost to a large value

# Class for RRT
class RRT:
    def __init__(self):
        self.node_list = []

    def is_obstacle_free(self, x1, y1, x2, y2, map_image):
        """Check if the path between two points is collision-free."""
        interpolated_x = np.linspace(x1, x2, num=20)
        interpolated_y = np.linspace(y1, y2, num=20)

        for ix, iy in zip(interpolated_x, interpolated_y):
            ix, iy = int(round(ix)), int(round(iy))
            if 0 <= ix < map_image.shape[1] and 0 <= iy < map_image.shape[0]:
                pixel_value = map_image[iy, ix]
                if pixel_value == 0:  # Obstacle
                    return False
        return True

    def calculate_distance_angle(self, x1, y1, x2, y2):
        x1, y1, x2, y2 = float(x1), float(y1), float(x2), float(y2)  # Ensure numeric types
        distance = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
        angle = math.atan2(y2 - y1, x2 - x1)
        return distance, angle


    def find_nearest_node(self, x, y):
        """Find the nearest node to the given point."""
        return min(self.node_list, key=lambda node: self.calculate_distance_angle(float(node.x), float(node.y), x, y)[0])

    def generate_random_point(self, height, width):
        return random.randint(0, width - 1), random.randint(0, height - 1)

    def perform_rrt(self, map_image, start_point, end_point, step_size):
        """Perform RRT and return the path as a list of points."""
        height, width = map_image.shape
        self.node_list = [GraphNode(start_point[0], start_point[1])]
        self.node_list[0].cost = 0

        max_iterations = 1000000
        for _ in range(max_iterations):
            random_x, random_y = self.generate_random_point(height, width)
            nearest_node = self.find_nearest_node(random_x, random_y)

            distance, angle = self.calculate_distance_angle(nearest_node.x, nearest_node.y, random_x, random_y)
            next_x = nearest_node.x + step_size * math.cos(angle)
            next_y = nearest_node.y + step_size * math.sin(angle)

            if next_x < 0 or next_y < 0 or next_x >= width or next_y >= height:
                continue

            if not self.is_obstacle_free(nearest_node.x, nearest_node.y, next_x, next_y, map_image):
                continue

            new_node = GraphNode(float(next_x), float(next_y))
            new_node.parent = nearest_node
            new_node.cost = nearest_node.cost + distance
            self.node_list.append(new_node)

            if self.calculate_distance_angle(next_x, next_y, end_point[0], end_point[1])[0] <= step_size:
                goal_node = GraphNode(end_point[0], end_point[1])
                goal_node.parent = new_node
                self.node_list.append(goal_node)
                return self.retrace_path(goal_node)

    def retrace_path(self, node):
        """Retrace the path from the goal node to the start node."""
        path = []
        while node:
            path.append((int(node.x), int(node.y)))
            node = node.parent
        return path[::-1]

    def visualize_path(self, map_image, path, output_path="visualized_path.png"):
        """Visualize the path on the map image."""
        visualization = cv2.cvtColor(map_image, cv2.COLOR_GRAY2BGR)
        for i in range(len(path) - 1):
            cv2.line(
                visualization,
                path[i],
                path[i + 1],
                (0, 255, 0), 2
            )
        cv2.imwrite(output_path, visualization)
        cv2.imshow("Path Visualization", visualization)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


## Main Class
class Task2(Node):

    def __init__(self, node_name='Navigation'):

        super().__init__(node_name)
        self.path = Path()
        self.goal_pose = PoseStamped()
        self.ttbot_pose = PoseWithCovarianceStamped()
        self.goal_point = PointStamped()

        self.create_subscription(PoseStamped, '/move_base_simple/goal', self.__goal_pose_cbk, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.__ttbot_pose_cbk, 10)

        self.path_pub = self.create_publisher(Path, '/plan', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.rate = self.create_rate(10)

        self.speed = 0
        self.heading = 0
        self.lin_integral_error = 0
        self.lin_previous_error = 0
        self.ang_integral_error = 0
        self.ang_previous_error = 0
        self.wp_reached = False
        self.index = 0
        self.prev_idx = 0

        self.world_dim = [10 ,7]
        self.graph_dim = [298, 209]
        self.world_origin = [0.0, 0.0]
        self.graph_origin = [110, 85]

        self.scale = [20,20]
        self.res   = [20,20] 


    def __goal_pose_cbk(self, data):
        self.goal_pose = data

    def __ttbot_pose_cbk(self, data):
        self.ttbot_pose = data
        # self.get_logger().error(f"wtf {self.ttbot_pose.pose.pose.position.x} {self.ttbot_pose.pose.pose.position.y}")


    def graph2pose(self, graph_y, graph_x):
        dx = graph_x - self.graph_origin[0]
        dy = graph_y - self.graph_origin[1]
        world_x = (dx / self.scale[0]) + self.world_origin[0]
        world_y = (-dy / self.scale[1]) + self.world_origin[1]
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose.position.x = world_x
        pose_stamped.pose.position.y = world_y
        pose_stamped.pose.position.z = 0.0 
        pose_stamped.pose.orientation.w = 1.0 
        return pose_stamped


    def pose2graph(self, pose):
        world_x = pose.pose.position.x
        world_y = pose.pose.position.y
        dx = world_x - self.world_origin[0]
        dy = world_y - self.world_origin[1]
        graph_x = int(dx * self.scale[0]) + self.graph_origin[0]
        graph_y = int(-dy * self.scale[1]) + self.graph_origin[1]
        return graph_x, graph_y


    def get_yaw_from_pose(self, pose):
        orientation_q = pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y ** 2 + orientation_q.z ** 2)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def PID_angular(self, angular_error):
        kp_ang, kd_ang, ki_ang, dt = 35, 35.5, 0.0001, 0.1
        self.ang_integral_error += angular_error * dt
        self.ang_integral_error = max(min(self.ang_integral_error, 1), -1)  # Anti-windup
        ang_derivative = (angular_error - self.ang_previous_error) / dt
        self.ang_previous_error = angular_error
        self.ang_vel = (kp_ang * angular_error) + (ki_ang * self.ang_integral_error) + (kd_ang * ang_derivative)
        self.ang_vel = min(max(abs(self.ang_vel), 0.0), 0.25)
        return self.ang_vel

    def PID_linear(self, linear_error):
        kp_lin, kd_lin, ki_lin, dt = 5.0, 2.5, 0.001, 0.1
        self.lin_integral_error += linear_error * dt
        self.lin_integral_error = max(min(self.lin_integral_error, 1.0), -1.0)  # Anti-windup
        lin_derivative = (linear_error - self.lin_previous_error) / dt
        self.lin_previous_error = linear_error
        self.lin_vel = (kp_lin * linear_error) + (ki_lin * self.lin_integral_error) + (kd_lin * lin_derivative)
        self.lin_vel = min(max(self.lin_vel, 0.0), 0.25)
        return self.lin_vel
    
    def reached_goal(self, current_pose, target_pose, tolerance=0.5):
        dx = target_pose.pose.position.x - current_pose.pose.position.x
        dy = target_pose.pose.position.y - current_pose.pose.position.y
        distance = np.sqrt(dx ** 2 + dy ** 2)
        # print(distance < tolerance)
        return distance < tolerance

    def inflate_map(self, image, kernel_sizes=[3,13]):
        inflated_image = image.copy()
        for kernel_size in kernel_sizes:
            kernel = np.ones((kernel_size, kernel_size), np.uint8)
            inflated_image = cv2.erode(inflated_image, kernel, iterations=1)
        # self.get_logger().info("inflated map")
        return inflated_image

    def rrt_path_planner(self, start_pose, end_pose):
        path = Path()
        start_point = self.pose2graph(start_pose)
        end_point = self.pose2graph(end_pose)
        step_size = 3
        self.get_logger().info(f"{start_point} {end_point}")

        # start_point = (110, 85)
        # end_point = (175, 50)
        
        image = '/home/lucifer/sim_ws/src/turtlebot3_gazebo/maps/map.pgm'
        map_image = cv2.imread(image, 0)
        map_image = self.inflate_map(map_image)

        rrt = RRT()
        node_path = rrt.perform_rrt(map_image, start_point, end_point, step_size)
        print(node_path)

        for y, x in node_path:
            waypoint = PoseStamped()
            waypoint = self.graph2pose(x, y)
            path.poses.append(waypoint)
        return node_path, path


    def get_path_idx(self, vehicle_pose, path, prev_idx):
        min_distance = np.inf
        angle_threshold = 0.1
        closest_deviation_idx = None
        vehicle_x = vehicle_pose.pose.position.x
        vehicle_y = vehicle_pose.pose.position.y
        vehicle_yaw = self.get_yaw_from_pose(vehicle_pose)
        
        for i in range((prev_idx+1), len(path.poses)):
            waypoint = path.poses[i]
            waypoint_x = ((waypoint.pose.position.x)) 
            waypoint_y = ((waypoint.pose.position.y))
            distance = np.sqrt((vehicle_x - waypoint_x) ** 2 + (vehicle_y - waypoint_y) ** 2)
            target_angle = math.atan2(waypoint_y - vehicle_y, waypoint_x - vehicle_x)
            angle_deviation = abs((target_angle - vehicle_yaw))
            
            if angle_deviation > angle_threshold and distance < min_distance:
                min_distance = distance
                closest_deviation_idx = i
        
        if closest_deviation_idx is not None:
            return closest_deviation_idx
        
        return (len(path.poses) - 1)


    def path_follower(self, vehicle_pose, current_goal, prev_goal):
        linear_error_margin = 0.25
        angular_error_margin = 0.25

        vehicle_x = vehicle_pose.pose.position.x
        vehicle_y = vehicle_pose.pose.position.y
        
        vehicle_yaw = self.get_yaw_from_pose(vehicle_pose)

        goal_x = (current_goal.pose.position.x)
        goal_y = (current_goal.pose.position.y)

        prev_goal_x = (prev_goal.pose.position.x) if (self.prev_idx>0) else 0
        prev_goal_y = (prev_goal.pose.position.y) if (self.prev_idx>0) else 0

        lin_ex = goal_x - vehicle_x
        lin_ey = goal_y - vehicle_y

        ang_ex = goal_x - prev_goal_x
        ang_ey = goal_y - prev_goal_y


        distance_to_goal = math.sqrt((lin_ex) ** 2 + (lin_ey) ** 2)
        target_angle = (math.atan2(ang_ey, ang_ex))
        angle_diff = self.normalize_angle(target_angle - vehicle_yaw)

        if ((abs(lin_ex)<linear_error_margin)) and ((abs(lin_ey)<linear_error_margin)) and (abs(angle_diff)<angular_error_margin):
            self.wp_reached = True
            self.get_logger().info('waypoint reached')
            speed = self.PID_linear(distance_to_goal)
            heading = self.PID_angular(abs(angle_diff)) if angle_diff > 0 else -self.PID_angular(abs(angle_diff))
        else: 
            self.wp_reached = False
            # self.get_logger().warn(f" G {self.index} {round(target_angle, 2)} {round(goal_x, 2)} {round(goal_y, 2)}")
            if (abs(angle_diff) > angular_error_margin):
                speed = 0.05 * distance_to_goal
                heading = self.PID_angular(abs(angle_diff)) if angle_diff > 0 else -self.PID_angular(abs(angle_diff))
                self.get_logger().warn(f" A {self.prev_idx} {self.index} {round(angle_diff, 2)} {round(distance_to_goal, 2)} ")
            else:
                speed = self.PID_linear(distance_to_goal)
                heading = 0.05 * angle_diff 
                # heading = self.PID_angular(abs(angle_diff)) if angle_diff > 0 else -self.PID_angular(abs(angle_diff))
                self.get_logger().warn(f" L {self.prev_idx} {self.index} {round(angle_diff, 2)} {round(distance_to_goal, 2)} ")           

        return speed, heading


    def path_tracer(self, current_goal, prev_goal):
        goal_x = (current_goal.pose.position.x)
        goal_y = (current_goal.pose.position.y)
        prev_goal_x = (prev_goal.pose.position.x)
        prev_goal_y = (prev_goal.pose.position.y)
        wp_ex = abs(goal_x - prev_goal_x)
        wp_ey = abs(goal_y - prev_goal_y)
        distance = math.sqrt((wp_ex) ** 2 + (wp_ey) ** 2)
        target_angle = self.normalize_angle(math.atan2(wp_ey, wp_ex)) 
        self.get_logger().warn(f" c {self.index} {round(target_angle, 2)} {round(goal_x, 2)} {round(goal_y, 2)}")          
        self.get_logger().warn(f" p {self.prev_idx} {round(target_angle, 2)} {round(prev_goal_x, 2)} {round(prev_goal_y, 2)}")           


    def move_ttbot(self, speed, heading):
        cmd_vel = Twist()
        cmd_vel.linear.x = float(speed)
        cmd_vel.angular.z = float(heading)
        # cmd_vel.linear.x = 0.0
        # cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

    def man_ttbot(self, speed=0.0, heading=0.0):
        self.move_ttbot(speed, -heading)
        time.sleep(0.5)
        self.move_ttbot(speed, heading)
        time.sleep(0.5)
        self.stop()
        self.get_logger().info(f"here1")
        time.sleep(1)
    
    def stop(self, ang=0.0, lin=0.0):
        msg = Twist()
        msg.linear.x = ang
        msg.angular.z = lin
        self.cmd_vel_pub.publish(msg) 

    def publish_path(self, poses):
        path_arr = Path()
        path_arr = poses
        path_arr.header.frame_id = "/map"
        offset_x = 0
        offset_y = 0
        for pose in path_arr.poses:
            pose.pose.position.x = 1 * (pose.pose.position.x) + offset_x
            pose.pose.position.y = 1 * (pose.pose.position.y) + offset_y
        self.path_pub.publish(path_arr)

    def visualize_path(self, map_image, path, output_path="visualized_path.png"):
        visualization = cv2.cvtColor(map_image, cv2.COLOR_GRAY2BGR)
        for i in range(len(path) - 1):
            cv2.line(
                visualization,
                path[i],
                path[i + 1],
                (0, 255, 0), 2
            )
        # cv2.imwrite(output_path, visualization)
        cv2.imshow("Path Visualization", visualization)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=2)
            if (self.goal_pose.pose.position.x != self.ttbot_pose.pose.pose.position.x or self.goal_pose.pose.position.y != self.ttbot_pose.pose.pose.position.y):
                self.get_logger().info('Planning path \n> from {}, {} \n> to {}, {}'.format(self.ttbot_pose.pose.pose.position.x, self.ttbot_pose.pose.pose.position.y, self.goal_pose.pose.position.x, self.goal_pose.pose.position.y))
                node_path, path = self.rrt_path_planner(self.ttbot_pose.pose, self.goal_pose)
                self.publish_path(path)
                
                # self.display_path(path, node_path)
                image = '/home/lucifer/sim_ws/src/turtlebot3_gazebo/maps/map.pgm'                
                # self.visualize_path(image, node_path)
                prev_idx = -1
                while not self.reached_goal(self.ttbot_pose.pose, self.goal_pose):
                    rclpy.spin_once(self, timeout_sec=0.1)
                    idx = self.get_path_idx(self.ttbot_pose.pose, path, prev_idx)
                    self.index = idx
                    self.prev_idx = prev_idx
                    
                    if idx != -1:
                        current_goal = path.poses[idx]
                        prev_goal = path.poses[prev_idx]
                        self.get_logger().info(f'ttx: {self.ttbot_pose.pose.pose.position.x}, tty: {self.ttbot_pose.pose.pose.position.y}')
                        while (self.wp_reached != True):
                            rclpy.spin_once(self, timeout_sec=0.1)
                            speed, heading = self.path_follower(self.ttbot_pose.pose, current_goal, prev_goal)
                            self.move_ttbot(speed, heading)                  
                    
                    print ("------------------------- wp reached ------------------------------")
                    prev_idx = idx 
                    self.wp_reached = False

                # Once all waypoints are reached, stop the robot
                self.get_logger().info("Goal reached, stopping robot")
                self.move_ttbot(0.0, 0.0)
                # break
        self.rate.sleep()


def main(args=None):

    rclpy.init(args=args)
    nav = Task2(node_name='Navigation')
    try:
        nav.run()
    except KeyboardInterrupt:
        pass
    finally:
        nav.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()



