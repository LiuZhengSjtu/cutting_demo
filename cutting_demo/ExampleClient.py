#!/usr/bin/python3
"""
----------------------------------------------------------------------------------------------------------------------
# This is a robot driver client example.
# It is based on `joint_interface_example.py` from `sas_robot_driver_kuka` (Author: Murilo M. Marinho, email: murilomarinho@ieee.org)
#

Jacobian (pose velocity <--> angle velocity) is used for pose control.
PseudoinverseController is also verified.


Edit, by Zheng Liu, Oct 2024


#   at the dir ~/ws_test
#   colcon build

#   at the dir ~/ws_test/src/test_pkg/test_pkg
#   ros2 run test_pkg ExampleClient
----------------------------------------------------------------------------------------------------------------------
"""


from sas_common import rclcpp_init, rclcpp_Node, rclcpp_spin_some, rclcpp_shutdown
import os
import sys
sys.path.insert(1,os.getcwd()+'/src/cut/cut/ExampleClientFiles')
sys.path.insert(1,os.getcwd()+'/ExampleClientFiles')
sys.path.insert(1,'/home/zheng/ws_test/src/cutting/cutting/ExampleClientFiles')
print(sys.path)


import DemoTask as Task


def main(args=None):
    try:
        for j in range(200):


            Task.pretask(j)

            # For some iterations. Note that this can be stopped with CTRL+C.
            for i in range(0, 40000):

                Task.DemoParameter.client.clock.update_and_sleep()

                if not Task.task_test(i) :
                    break

                rclcpp_spin_some(Task.DemoParameter.client.node)

            Task.posttask(j)

        rclcpp_shutdown()

    except KeyboardInterrupt:
        Task.posttask(only_clear_lines = 1, statis_rot = False)
        print("Interrupted by user")
    except Exception as e:
        print("Unhandled excepts", e)


if __name__ == '__main__':
    main()
