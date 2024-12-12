# import cv2
# import numpy as np


# # Class to for graphnode for rrt
# class GraphNode:
#     """Class to represent nodes in the RRT graph"""
#     def __init__(self, x, y):
#         self.x = float(x)
#         self.y = float(y)
#         self.parent = None  # Parent node reference
#         self.cost = float('inf')  # Initialize cost to a large value

# # Class for RRT
# class RRT:
#     def __init__(self):
#         self.node_list = []

#     def is_obstacle_free(self, x1, y1, x2, y2, map_image):
#         """Check if the path between two points is collision-free."""
#         interpolated_x = np.linspace(x1, x2, num=20)
#         interpolated_y = np.linspace(y1, y2, num=20)

#         for ix, iy in zip(interpolated_x, interpolated_y):
#             ix, iy = int(round(ix)), int(round(iy))
#             if 0 <= ix < map_image.shape[1] and 0 <= iy < map_image.shape[0]:
#                 pixel_value = map_image[iy, ix]
#                 if pixel_value == 0:  # Obstacle
#                     return False
#         return True

#     def calculate_distance_angle(self, x1, y1, x2, y2):
#         x1, y1, x2, y2 = float(x1), float(y1), float(x2), float(y2)  # Ensure numeric types
#         distance = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
#         angle = math.atan2(y2 - y1, x2 - x1)
#         return distance, angle


#     def find_nearest_node(self, x, y):
#         """Find the nearest node to the given point."""
#         return min(self.node_list, key=lambda node: self.calculate_distance_angle(float(node.x), float(node.y), x, y)[0])

#     def generate_random_point(self, height, width):
#         return random.randint(0, width - 1), random.randint(0, height - 1)

#     def perform_rrt(self, map_image, start_point, end_point, step_size):
#         """Perform RRT and return the path as a list of points."""
#         height, width = map_image.shape
#         self.node_list = [GraphNode(start_point[0], start_point[1])]
#         self.node_list[0].cost = 0

#         max_iterations = 1000000
#         for _ in range(max_iterations):
#             random_x, random_y = self.generate_random_point(height, width)
#             nearest_node = self.find_nearest_node(random_x, random_y)

#             distance, angle = self.calculate_distance_angle(nearest_node.x, nearest_node.y, random_x, random_y)
#             next_x = nearest_node.x + step_size * math.cos(angle)
#             next_y = nearest_node.y + step_size * math.sin(angle)

#             if next_x < 0 or next_y < 0 or next_x >= width or next_y >= height:
#                 continue

#             if not self.is_obstacle_free(nearest_node.x, nearest_node.y, next_x, next_y, map_image):
#                 continue

#             new_node = GraphNode(float(next_x), float(next_y))
#             new_node.parent = nearest_node
#             new_node.cost = nearest_node.cost + distance
#             self.node_list.append(new_node)

#             if self.calculate_distance_angle(next_x, next_y, end_point[0], end_point[1])[0] <= step_size:
#                 goal_node = GraphNode(end_point[0], end_point[1])
#                 goal_node.parent = new_node
#                 self.node_list.append(goal_node)
#                 return self.retrace_path(goal_node)

#     def retrace_path(self, node):
#         """Retrace the path from the goal node to the start node."""
#         path = []
#         while node:
#             path.append((int(node.x), int(node.y)))
#             node = node.parent
#         return path[::-1]

#     def visualize_path(self, map_image, path, output_path="visualized_path.png"):
#         """Visualize the path on the map image."""
#         visualization = cv2.cvtColor(map_image, cv2.COLOR_GRAY2BGR)
#         for i in range(len(path) - 1):
#             cv2.line(
#                 visualization,
#                 path[i],
#                 path[i + 1],
#                 (0, 255, 0), 2
#             )
#         cv2.imwrite(output_path, visualization)
#         cv2.imshow("Path Visualization", visualization)
#         cv2.waitKey(0)
#         cv2.destroyAllWindows()


# def inflate_map(image, kernel_sizes=[3,13]):
#     inflated_image = image.copy()
#     for kernel_size in kernel_sizes:
#         kernel = np.ones((kernel_size, kernel_size), np.uint8)
#         inflated_image = cv2.erode(inflated_image, kernel, iterations=1)
#     return inflated_image




# # --

# # def inflate_map(image, kernel_sizes=[3,13]):
# #     inflated_image = image.copy()
# #     for kernel_size in kernel_sizes:
# #         kernel = np.ones((kernel_size, kernel_size), np.uint8)
# #         inflated_image = cv2.erode(inflated_image, kernel, iterations=1)

# #     return inflated_image

# if __name__ == "__main__":
#     map_name = '/home/lucifer/sim_ws/src/turtlebot3_gazebo/maps/map'
#     image_path = "/home/lucifer/sim_ws/src/turtlebot3_gazebo/maps/map.pgm"
#     start_point = (110, 85)
#     end_point = (175, 50)
#     step_size = 5

    
#     map_image = cv2.imread(image_path, 0)
#     map_image = inflate_map(map_image)

#     rrt = RRT()
#     path = rrt.perform_rrt(map_image, start_point, end_point, step_size)

#     print("Path:", path)

#     rrt.visualize_path(map_image, path)