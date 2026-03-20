import math
import sys
import select
import termios
import tty
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Twist


class ResetCar(Node):
    def __init__(self):
        """
        Resets the car by deleting and respawning it in Gazebo.

        Press 'r' in the terminal running this node
        ros2 topic pub /reset_car std_msgs/msg/Empty --once

        Parameters:
        entity_name  - Gazebo model name (default: f1tenth_car)
        x, y, z      - Spawn position (default: 0.0, 0.0, 0.1)
        yaw          - Spawn heading in radians (default: 0.0)

        Usage:
        python3 reset_car.py
        python3 reset_car.py --ros-args -p x:=1.0 -p y:=-1.1 -p yaw:=1.57
        """
        super().__init__('reset_car')

        self.declare_parameter('entity_name', 'f1tenth_car')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.1)
        self.declare_parameter('yaw', 0.0)

        self.create_subscription(Empty, '/reset_car', self._reset_cb, 10)

        # Get robot description for respawning
        self.robot_description = None
        self.create_subscription(String, '/robot_description', self._desc_cb, 10)

        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.resetting = False

        # Keyboard polling
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        self.create_timer(0.1, self._check_keyboard)

        self.get_logger().info('Reset node ready — press "r" or publish to /reset_car')
        self.get_logger().info('Waiting for robot_description...')

    def _desc_cb(self, msg):
        self.robot_description = msg.data
        self.get_logger().info('Got robot description')

    def _reset_cb(self, msg):
        self._do_reset()

    def _check_keyboard(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            if key == 'r':
                self._do_reset()

    def _do_reset(self):
        if self.resetting:
            self.get_logger().warn('Reset already in progress...')
            return

        if self.robot_description is None:
            self.get_logger().error(
                'No robot_description received. '
                'Is robot_state_publisher running?'
            )
            return

        self.resetting = True
        # Run blocking sequence in a thread so we don't block the executor
        thread = threading.Thread(target=self._reset_sequence, daemon=True)
        thread.start()

    def _reset_sequence(self):
        entity = self.get_parameter('entity_name').value
        x = float(self.get_parameter('x').value)
        y = float(self.get_parameter('y').value)
        z = float(self.get_parameter('z').value)
        yaw = float(self.get_parameter('yaw').value)

        try:
            # Stop the car
            self.cmd_pub.publish(Twist())

            # 1. Delete entity
            self.get_logger().info(f'Deleting {entity}...')
            if not self.delete_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error('/delete_entity service not available')
                return

            del_req = DeleteEntity.Request()
            del_req.name = entity
            del_future = self.delete_client.call_async(del_req)
            rclpy.spin_until_future_complete(self, del_future, timeout_sec=5.0)

            if del_future.result() is not None:
                self.get_logger().info(f'Delete: {del_future.result().status_message}')
            else:
                self.get_logger().warn('Delete timed out, attempting spawn anyway...')

            # Brief pause for Gazebo cleanup
            import time
            time.sleep(0.5)

            # 2. Respawn entity
            self.get_logger().info(f'Spawning {entity} at ({x}, {y}, {z}, yaw={yaw})...')
            if not self.spawn_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error('/spawn_entity service not available')
                return

            spawn_req = SpawnEntity.Request()
            spawn_req.name = entity
            spawn_req.xml = self.robot_description
            spawn_req.robot_namespace = ''
            spawn_req.initial_pose.position.x = x
            spawn_req.initial_pose.position.y = y
            spawn_req.initial_pose.position.z = z
            spawn_req.initial_pose.orientation.x = 0.0
            spawn_req.initial_pose.orientation.y = 0.0
            spawn_req.initial_pose.orientation.z = math.sin(yaw / 2.0)
            spawn_req.initial_pose.orientation.w = math.cos(yaw / 2.0)

            spawn_future = self.spawn_client.call_async(spawn_req)
            rclpy.spin_until_future_complete(self, spawn_future, timeout_sec=10.0)

            if spawn_future.result() is not None and spawn_future.result().success:
                self.get_logger().info('Car respawned successfully')
            else:
                msg = spawn_future.result().status_message if spawn_future.result() else 'timeout'
                self.get_logger().error(f'Spawn failed: {msg}')

        except Exception as e:
            self.get_logger().error(f'Reset error: {e}')
        finally:
            self.resetting = False

    def destroy_node(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ResetCar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()