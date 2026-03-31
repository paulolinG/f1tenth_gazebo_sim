"""
ROS 2 Node for resetting a Gazebo model (specifically an F1Tenth car).
Allows triggering the reset sequence via a keyboard press ('r') in the terminal
or by publishing an Empty message to the '/reset_car' topic.
"""

import math
import sys
import select
import termios
import tty
from typing import Optional, List, Any

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.timer import Timer
from std_msgs.msg import Empty, String
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSDurabilityPolicy


class ResetCar(Node):
    """
    Resets the car by deleting and respawning it in Gazebo using asynchronous service calls.

    This node actively listens to standard input (stdin) for the 'r' key and also 
    subscribes to the `/reset_car` topic to trigger the sequence.

    Parameters
    ----------
    entity_name : str
        The Gazebo model name to delete and spawn (default: 'f1tenth_car').
    x : float
        The X coordinate for the spawn position (default: 0.0).
    y : float
        The Y coordinate for the spawn position (default: 0.0).
    z : float
        The Z coordinate for the spawn position (default: 0.1).
    yaw : float
        The spawn heading (yaw) in radians (default: 0.0).
    """
    
    def __init__(self) -> None:
        """Initializes the node, parameters, publishers, subscribers, and clients."""
        super().__init__('reset_car')

        self.declare_parameter('entity_name', 'f1tenth_car')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', -3.0)
        self.declare_parameter('z', 0.05)
        self.declare_parameter('yaw', 0.0)

        self.create_subscription(Empty, '/reset_car', self._reset_cb, 10)

        # Get robot description for respawning
        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.robot_description: Optional[str] = None
        self.create_subscription(String, '/robot_description', self._desc_cb, latched_qos)

        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.resetting: bool = False
        self._spawn_timer: Optional[Timer] = None

        # Keyboard polling setup
        self.old_settings: List[Any] = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        self.create_timer(0.1, self._check_keyboard)

        self.get_logger().info('Reset node ready — press "r" or publish to /reset_car')
        self.get_logger().info('Waiting for robot_description...')

    def _desc_cb(self, msg: String) -> None:
        """
        Callback for the robot_description topic.

        Args:
            msg (String): The URDF or SDF XML string of the robot.
        """
        self.robot_description = msg.data
        self.get_logger().info('Got robot description')

    def _reset_cb(self, msg: Empty) -> None:
        """
        Callback for the /reset_car topic. Triggers the reset sequence.

        Args:
            msg (Empty): The empty trigger message.
        """
        self._start_reset_sequence()

    def _check_keyboard(self) -> None:
        """
        Timer callback that non-blockingly polls stdin for the 'r' key.
        """
        if select.select([sys.stdin], [], [], 0)[0]:
            key: str = sys.stdin.read(1)
            if key == 'r':
                self._start_reset_sequence()

    def _start_reset_sequence(self) -> None:
        """
        Initiates the asynchronous reset sequence. 
        Stops the car and calls the Gazebo delete entity service.
        """
        if self.resetting:
            self.get_logger().warn('Reset already in progress, ignoring press...')
            return

        if self.robot_description is None:
            self.get_logger().error('No robot_description received. Is robot_state_publisher running?')
            return

        self.resetting = True

        # Stop the car before deleting
        self.cmd_pub.publish(Twist())

        entity: str = str(self.get_parameter('entity_name').value)
        self.get_logger().info(f'Deleting {entity}...')

        if not self.delete_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('/delete_entity service not available')
            self.resetting = False
            return

        # 1. Send asynchronous delete request
        del_req = DeleteEntity.Request()
        del_req.name = entity
        future: Future = self.delete_client.call_async(del_req)
        future.add_done_callback(self._delete_done_cb)

    def _delete_done_cb(self, future: Future) -> None:
        """
        Callback executed when the delete entity service call finishes.
        Sets a timer to briefly delay the spawn call to allow Gazebo to clear physics.

        Args:
            future (Future): The future object containing the service response.
        """
        try:
            result: DeleteEntity.Response = future.result()
            self.get_logger().info(f'Delete status: {result.status_message}')
        except Exception as e:
            self.get_logger().warn(f'Delete failed or timed out: {e}. Attempting spawn anyway...')

        # Trigger a brief pause using a one-shot ROS timer
        self._spawn_timer = self.create_timer(0.5, self._trigger_spawn)

    def _trigger_spawn(self) -> None:
        """
        Timer callback to execute the spawn logic. Cancels itself to act as a one-shot timer.
        """
        if self._spawn_timer:
            self._spawn_timer.cancel()
        
        self._spawn_car()

    def _spawn_car(self) -> None:
        """
        Prepares and sends the asynchronous service request to spawn the car in Gazebo.
        """
        entity: str = str(self.get_parameter('entity_name').value)
        x: float = float(self.get_parameter('x').value)
        y: float = float(self.get_parameter('y').value)
        z: float = float(self.get_parameter('z').value)
        yaw: float = float(self.get_parameter('yaw').value)

        self.get_logger().info(f'Spawning {entity} at ({x}, {y}, {z}, yaw={yaw})...')

        if not self.spawn_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('/spawn_entity service not available')
            self.resetting = False
            return

        # 2. Send asynchronous spawn request
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

        future: Future = self.spawn_client.call_async(spawn_req)
        future.add_done_callback(self._spawn_done_cb)

    def _spawn_done_cb(self, future: Future) -> None:
        """
        Callback executed when the spawn entity service call finishes. 
        Unlocks the reset mechanism.

        Args:
            future (Future): The future object containing the service response.
        """
        try:
            result: SpawnEntity.Response = future.result()
            if result.success:
                self.get_logger().info('Car respawned successfully!')
            else:
                self.get_logger().error(f'Spawn failed: {result.status_message}')
        except Exception as e:
            self.get_logger().error(f'Spawn error: {e}')
        finally:
            self.resetting = False

    def destroy_node(self) -> bool:
        """
        Cleans up terminal settings before the node is destroyed.
        
        Returns:
            bool: Success status of the node destruction.
        """
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    """
    Main execution loop.

    Args:
        args (Optional[List[str]]): Command line arguments passed to rclpy.init.
    """
    rclpy.init(args=args)
    node = ResetCar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()