#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
import math
import time

class DroneMissionNode(Node):
    def __init__(self):
        super().__init__('drone_mission_node')
        
        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.takeoff_publisher = self.create_publisher(Empty, 'takeoff', 10)
        self.land_publisher = self.create_publisher(Empty, 'land', 10)
        
        # Subscriber
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        
        self.current_position = None
        self.mission_complete = False
        self.position_initialized = False
        self.last_cmd_vel = None
        self.movement_start_time = None
        
    def odom_callback(self, msg):
        self.current_position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ]
        if not self.position_initialized:
            self.position_initialized = True
            self.get_logger().info(f"Initial position: ({self.current_position[0]:.2f}, {self.current_position[1]:.2f}, {self.current_position[2]:.2f})")
    
    def wait_for_position(self):
        self.get_logger().info("Waiting for initial position...")
        while rclpy.ok() and not self.position_initialized:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Initial position received.")
    
    def takeoff(self):
        self.get_logger().info("Taking off...")
        self.takeoff_publisher.publish(Empty())
        time.sleep(5)  # Wait for takeoff to complete
    
    def land(self):
        self.get_logger().info("Landing...")
        self.land_publisher.publish(Empty())
    
    def move_to_position(self, target_x, target_y, target_z):
        self.get_logger().info(f"Moving from ({self.current_position[0]:.2f}, {self.current_position[1]:.2f}, {self.current_position[2]:.2f}) to ({target_x}, {target_y}, {target_z})")
        
        self.movement_start_time = time.time()
        start_position = self.current_position.copy()
        
        while not self.mission_complete and rclpy.ok():
            dx = target_x - self.current_position[0]
            dy = target_y - self.current_position[1]
            dz = target_z - self.current_position[2]
            
            distance = math.sqrt(dx**2 + dy**2 + dz**2)
            
            if distance < 0.1:  # If we're close enough to the target
                self.mission_complete = True
                break
            
            # Create and publish Twist message
            twist = Twist()
            twist.linear.x = dx * 0.5
            twist.linear.y = dy * 0.5
            twist.linear.z = dz * 0.5
            
            self.cmd_vel_publisher.publish(twist)
            self.last_cmd_vel = twist
            
            # Log current position and velocity command every 2 seconds
            elapsed_time = time.time() - self.movement_start_time
            if int(elapsed_time) % 2 == 0:
                self.get_logger().info(f"Time: {elapsed_time:.2f}s, Position: ({self.current_position[0]:.2f}, {self.current_position[1]:.2f}, {self.current_position[2]:.2f})")
                self.get_logger().info(f"Velocity command: ({twist.linear.x:.2f}, {twist.linear.y:.2f}, {twist.linear.z:.2f})")
            
            # Check if drone has moved
            if elapsed_time > 10 and self.distance(start_position, self.current_position) < 0.1:
                self.get_logger().warn("Drone hasn't moved in 10 seconds. Attempting to takeoff...")
                self.takeoff()
                start_position = self.current_position.copy()
                self.movement_start_time = time.time()
            
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Stop the drone
        self.cmd_vel_publisher.publish(Twist())
        self.get_logger().info("Reached target position or mission aborted")
    
    def distance(self, pos1, pos2):
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(pos1, pos2)))
    
    def execute_mission(self):
        self.get_logger().info("Starting mission...")
        
        # Wait for the initial position
        self.wait_for_position()
        
        # Attempt takeoff
        self.takeoff()
        
        # First, rise to the target height
        self.move_to_position(self.current_position[0], self.current_position[1], 24)
        
        # Reset mission_complete flag
        self.mission_complete = False
        
        # Then, move to the target X and Y coordinates
        self.move_to_position(0, 0, 24)
        
        self.land()
        self.get_logger().info("Mission complete")

def main(args=None):
    rclpy.init(args=args)
    drone_mission_node = DroneMissionNode()
    
    try:
        drone_mission_node.execute_mission()
    except Exception as e:
        drone_mission_node.get_logger().error(f"An error occurred: {str(e)}")
    finally:
        # Ensure the drone stops moving if the script is interrupted
        drone_mission_node.cmd_vel_publisher.publish(Twist())
        rclpy.shutdown()

if __name__ == '__main__':
    main()
