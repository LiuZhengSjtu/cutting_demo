#!/usr/bin/python3
'''

This file is imported by ExampleClient.py
This file is used to configure the robot driver client.

The lib RobotDriverClient is adopted to achieve:
    subscriber
    publisher
    joint state reading
    target joint positions sending
    kinematic parameter calculation


The robot driver client is a lib from smart_arm_stack_ROS2 (https://github.com/SmartArmStack/smart_arm_stack_ROS2/tree/jazzy)

It receive/send topics from/to server.

The server is launched through composed_with_coppeliasim_launch.py from above repository.

'''

import time
from sas_common import rclcpp_init, rclcpp_Node, rclcpp_spin_some, rclcpp_shutdown
from sas_robot_driver import RobotDriverClient
from sas_core import Clock, Statistics
import os


class SASClient():
    def __init__(self,Ts):

        rclcpp_init()
        time.sleep(0.2)
        self.node = rclcpp_Node("sas_robot_driver_ur_joint_space_example_node_cpp")

        # 10 ms clock
        self.clock = Clock(Ts)
        self.clock.init()

        time.sleep(0.2)

        # Initialize the RobotDriverClient
        if os.getcwd()[0:10] == '/home/e295':
            self.rdi = RobotDriverClient(self.node, 'ur_composed_realrobot')
        else:
            self.rdi = RobotDriverClient(self.node, 'ur_composed_mac')
        time.sleep(0.2)
        print('the state of rdi.is_enable = ', self.rdi.is_enabled())

        # Wait for RobotDriverClient to be enabled
        print('wait for Subscriber initialization')
        while not self.rdi.is_enabled():
            rclcpp_spin_some(self.node)
            time.sleep(0.1)
        print('Subscriber initialized')

        # Get topic information
        print(f"topic prefix = {self.rdi.get_topic_prefix()}")

        # Read the values sent by the RobotDriverServer
        joint_positions = self.rdi.get_joint_positions()
        print(f"joint positions = {joint_positions}")