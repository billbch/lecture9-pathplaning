import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseStamped
import time

class MavrosMission(Node):
    def __init__(self):
        super().__init__('mavros_mission')

        self.current_state = State()
        self.path = []
        self.current_wp = 0
        self.armed = False

        # Subscribers
        self.path_sub = self.create_subscription(
            Path, '/planned_path', self.path_cb, 10)
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_cb, 10)

        # Publisher de setpoint local
        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)

        # Servicios
        self.set_mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        self.arming_cli   = self.create_client(CommandBool, '/mavros/cmd/arming')

        # Timer para publicar setpoints a 10Hz (PX4 necesita stream continuo)
        self.timer = self.create_timer(0.1, self.timer_cb)
        self.target_pose = PoseStamped()
        self.target_pose.pose.position.z = 3.0  # hover inicial

        self.get_logger().info('Mavros Mission Node iniciado, esperando path...')

    def state_cb(self, msg: State):
        self.current_state = msg

    def path_cb(self, msg: Path):
        if not msg.poses:
            return
        self.path = msg.poses
        self.current_wp = 0
        self.get_logger().info(f'Path recibido con {len(self.path)} waypoints')
        self.arm_and_offboard()

    def arm_and_offboard(self):
        # Esperar conexión
        timeout = 0
        while not self.current_state.connected and timeout < 30:
            time.sleep(0.5)
            timeout += 1

        if not self.current_state.connected:
            self.get_logger().error('No hay conexión con FCU')
            return

        # Publicar algunos setpoints antes de cambiar modo (requisito PX4)
        self.get_logger().info('Publicando setpoints iniciales...')
        for _ in range(20):
            self.setpoint_pub.publish(self.target_pose)
            time.sleep(0.05)

        # Cambiar a OFFBOARD
        if self.set_mode_cli.wait_for_service(timeout_sec=5.0):
            req = SetMode.Request()
            req.custom_mode = 'OFFBOARD'
            self.set_mode_cli.call_async(req)
            self.get_logger().info('Modo OFFBOARD activado')
            time.sleep(0.5)

        # Armar
        if self.arming_cli.wait_for_service(timeout_sec=5.0):
            req = CommandBool.Request()
            req.value = True
            self.arming_cli.call_async(req)
            self.get_logger().info('Armando dron...')

    def timer_cb(self):
        """Publica el siguiente waypoint cuando el dron está cerca"""
        if not self.path:
            # Sin path, hover en z=3
            self.target_pose.header.stamp = self.get_clock().now().to_msg()
            self.target_pose.header.frame_id = 'map'
            self.setpoint_pub.publish(self.target_pose)
            return

        if self.current_wp < len(self.path):
            pose = self.path[self.current_wp]
            self.target_pose.header.stamp = self.get_clock().now().to_msg()
            self.target_pose.header.frame_id = 'map'
            self.target_pose.pose = pose.pose
            self.setpoint_pub.publish(self.target_pose)

            # Avanzar al siguiente waypoint cada 3 segundos (simple)
            # En producción usarías distancia al goal
            if not hasattr(self, '_wp_timer'):
                self._wp_timer = 0
            self._wp_timer += 1
            if self._wp_timer > 30:  # 30 * 0.1s = 3s por waypoint
                self._wp_timer = 0
                self.current_wp += 1
                if self.current_wp < len(self.path):
                    self.get_logger().info(f'Waypoint {self.current_wp}/{len(self.path)}')
                else:
                    self.get_logger().info('Misión completada!')
        else:
            # Hover en último punto
            self.setpoint_pub.publish(self.target_pose)

def main(args=None):
    rclpy.init(args=args)
    node = MavrosMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()