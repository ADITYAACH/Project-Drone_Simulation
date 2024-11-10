#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from ament_index_python.packages import get_package_share_directory
import os
import time

class DroneSpawner(Node):
    def __init__(self, args):
        super().__init__('drone_spawner')
        
        if len(args) < 6:
            self.get_logger().error('Not enough arguments! Usage: spawn_drone [robot_description] [namespace] [x] [y] [z]')
            return

        # Create service client
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Spawn service not available, waiting...')

        # Prepare request
        request = SpawnEntity.Request()
        request.xml = args[1]            # Robot description from URDF
        request.name = args[2]           # Robot name/namespace
        request.robot_namespace = args[2] # Robot namespace
        request.initial_pose.position.x = float(args[3])
        request.initial_pose.position.y = float(args[4])
        request.initial_pose.position.z = float(args[5])
        request.reference_frame = "world"

        # Call service
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Spawn result: {future.result().status_message}')
            if future.result().success:
                self.get_logger().info(f'Successfully spawned {request.name}!')
            else:
                self.get_logger().error(f'Failed to spawn {request.name}')
        else:
            self.get_logger().error('Service call failed')

def main(args=None):
    rclpy.init(args=args)
    
    spawner = DroneSpawner(rclpy.utilities.remove_ros_args(args))
    
    # Give some time for the entity to spawn before shutting down
    time.sleep(2.0)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
