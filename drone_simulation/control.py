import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class DroneControlNode(Node):
    def __init__(self):
        super().__init__('drone_control_node')
        self.publisher = self.create_publisher(Twist, '/drone/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.send_command)  # 10 Hz
        self.get_logger().info('Drone Control Node has been started.')

    def send_command(self):
        # Create a Twist message
        msg = Twist()

        # Set linear and angular velocities
        msg.linear.x = 0.5  # Move forward
        msg.linear.y = 0.0  # No sideways movement
        msg.linear.z = 0.0  # Maintain altitude
        msg.angular.x = 0.0  # No roll
        msg.angular.y = 0.0  # No pitch
        msg.angular.z = 0.5  # Rotate

        # Publish the command
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: linear={msg.linear}, angular={msg.angular}')

def main(args=None):
    rclpy.init(args=args)
    drone_control_node = DroneControlNode()
    rclpy.spin(drone_control_node)

    # Destroy the node explicitly
    drone_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

