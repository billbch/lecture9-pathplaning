import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import heapq
import numpy as np

from .voxel_grid import VoxelGrid

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner')

        # Parámetros
        self.declare_parameter('grid_size_x', 50.0)
        self.declare_parameter('grid_size_y', 50.0)
        self.declare_parameter('grid_size_z', 20.0)
        self.declare_parameter('resolution', 0.5)
        self.declare_parameter('min_altitude', 2.0)
        self.declare_parameter('max_altitude', 15.0)

        sx = self.get_parameter('grid_size_x').value
        sy = self.get_parameter('grid_size_y').value
        sz = self.get_parameter('grid_size_z').value
        res = self.get_parameter('resolution').value

        self.voxel = VoxelGrid(sx, sy, sz, res)
        self.voxel.min_altitude = self.get_parameter('min_altitude').value
        self.voxel.max_altitude = self.get_parameter('max_altitude').value

        # No-fly zones de ejemplo (configurables en params.yaml)
        self.voxel.add_no_fly_zone(10, 20, 10, 20, 0, 10)

        # Publisher del path
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

        # Subscriber al pointcloud (del LiDAR/depth cam)
        self.pc_sub = self.create_subscription(
            PointCloud2, '/pointcloud', self.pointcloud_cb, 10)

        # Goal y start fijos para demo (luego se pueden hacer dinámicos)
        self.start = (5.0, 5.0, 3.0)
        self.goal  = (40.0, 40.0, 5.0)

        self.get_logger().info('Path Planner Node iniciado')
        # Planificar una vez al inicio (sin obstáculos dinámicos aún)
        self.plan_and_publish()

    def pointcloud_cb(self, msg):
        """Actualiza el grid con los puntos del LiDAR"""
        for point in pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True):
            self.voxel.mark_obstacle(point[0], point[1], point[2], radius=0.5)
        self.plan_and_publish()

    def plan_and_publish(self):
        path = self.astar(self.start, self.goal)
        if path is None:
            self.get_logger().warn('No se encontró camino!')
            return
        self.get_logger().info(f'Path encontrado con {len(path)} waypoints')
        self.publish_path(path)

    def astar(self, start_world, goal_world):
        start = self.voxel.world_to_grid(*start_world)
        goal  = self.voxel.world_to_grid(*goal_world)

        def h(a, b):
            return np.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: h(start, goal)}

        # 26 vecinos en 3D
        neighbors_offsets = [
            (dx, dy, dz)
            for dx in [-1, 0, 1]
            for dy in [-1, 0, 1]
            for dz in [-1, 0, 1]
            if not (dx == 0 and dy == 0 and dz == 0)
        ]

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                # Reconstruir path
                path = []
                while current in came_from:
                    path.append(self.voxel.grid_to_world(*current))
                    current = came_from[current]
                path.append(start_world)
                return list(reversed(path))

            for dx, dy, dz in neighbors_offsets:
                neighbor = (current[0]+dx, current[1]+dy, current[2]+dz)
                if not self.voxel.is_free(*neighbor):
                    continue
                tentative_g = g_score[current] + h(current, neighbor)
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + h(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None  # No hay camino

    def publish_path(self, path):
        msg = Path()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        for (x, y, z) in path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.path_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()