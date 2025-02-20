

print('in TaskConfig')
from numpy.ma.core import shape
from sas_core import Clock, Statistics
import  numpy as np
from math import *
from dqrobotics import *


import DemoParameter
import FunRobot
import FunTrajectory





#------------------------------------------------------------------------------------------------------
print('-' * 50 + ' TaskConfig start ' + '-' * 50)

pose_variable = FunTrajectory.pose_variable_class()


def pretask(j):
    global pose_variable

    print(f'\n\n\nstart {j}th trajectory -- pretask')
    DemoParameter.reconfig(j % 4)

    #   save the initial pose
    pose_variable.pose0, pose_variable.tra0, pose_variable.rot0 = FunRobot.savepose(DemoParameter.robot_hd, DemoParameter.client)

    pose_variable.position_home = DemoParameter.robot_hd.position_home[DemoParameter.para_object.object.home_position_n]
    pose_variable.pose_home = DemoParameter.robot_hd.pose_home[DemoParameter.para_object.object.home_position_n]
    pose_variable.poses, pose_variable.points_list, pose_variable.poses_reached, DemoParameter.color_turns = FunTrajectory.points2dq(
        FunTrajectory.getdesiredpoints(DemoParameter.para, DemoParameter.para_object.cylinder_cube, lettersize=DemoParameter.lettersize, objects= DemoParameter.objects)
                                                                                       )
    pose_variable.l_poses_plan = FunTrajectory.poses2trajectory(pose_home=pose_variable.pose_home, desiredposes = pose_variable.poses, robot_installation=DemoParameter.robot_installation,
                                               para= DemoParameter.para,durations=DemoParameter.para.durations)

    DemoParameter.drawing.drawlines(pose_variable.points_list)
    # RobotConfig.homepose(pose_variable.pose_home)
    FunRobot.homeposition(pose_variable.position_home, robot_hd=DemoParameter.robot_hd, client=DemoParameter.client, robot_installation=DemoParameter.robot_installation)

    pose_variable.target_joint_positions_p = DemoParameter.client.rdi.get_joint_positions()

    #   update the initial pose
    pose_variable.pose0, pose_variable.tra0, pose_variable.rot0 = FunRobot.savepose(DemoParameter.robot_hd, DemoParameter.client)

    print(f'at the beginning, first desired line DQ = {pose_variable.points_list[0]}')

    print('pretask done')


def posttask(j=0, only_clear_lines = 0, statis_rot = True):
    if not only_clear_lines:
        # RobotConfig.homepose(pose_variable.pose_home)
        FunRobot.homeposition(pose_variable.position_home, robot_hd=DemoParameter.robot_hd, client=DemoParameter.client, robot_installation=DemoParameter.robot_installation)
        DemoParameter.drawing.drawlinesclear(c=0)
    else:
        DemoParameter.drawing.drawlinesclear(c=0)
    print(f'posttask done with j = {j}')

    if statis_rot:
        DemoParameter.statistic_rotate(j, DemoParameter.client, DemoParameter.move_skip, DemoParameter.stepmotor)
    print(f'complete {j}th trajectory')

def task_test(i=0):
    return task_test2(i)


def task_test2(i=1):
    #   move the end-effector direction aligned with desired line. Use DQ_KinematicController to calculate the joint cmd.
    #   update: the desired pose (pose_t) is from the predefined trajectory.

    global pose_variable

    # print(f'in task_test2 i = {i}')
    if i >= shape(pose_variable.l_poses_plan)[0]:
        print(f'task done at i = {i}')
        return False
    else:
        pose_t = pose_variable.l_poses_plan[i]



    joint_positions = DemoParameter.client.rdi.get_joint_positions()

    #----------------------------------------------------------------------------
    #      add the point to line distance inequality constraint

    if 1:
        if DemoParameter.para_object.cylinder_cube == 2:
            pose_cur = FunRobot.point2lineconstraint(joint_positions, DemoParameter.robot_hd, DemoParameter.cons_object, DemoParameter.controller, DemoParameter.para_object)
        elif DemoParameter.para_object.cylinder_cube == 1:
    #   for the distance constraint from the cube upper surface to the tool
            pose_cur = FunRobot.point2planeconstraint(joint_positions, DemoParameter.robot_hd, DemoParameter.cons_object, DemoParameter.controller, DemoParameter.para_object)

        elif DemoParameter.para_object.cylinder_cube == 0:
            # print(f'in task_test2 i = {i}  2')
            #   for a series of discrete lines
            #   configure the constraint target, line or plane, in RobotConfig.cons_object
            pose_cur = FunRobot.point2planeconstraint(joint_positions, DemoParameter.robot_hd, DemoParameter.cons_object, DemoParameter.controller, DemoParameter.para_object)
        elif DemoParameter.para_object.cylinder_cube == 3:
            #   for the sphere constraint.
            pose_cur = FunRobot.point2point(joint_positions, DemoParameter.robot_hd,
                                                      DemoParameter.cons_object, DemoParameter.controller,
                                                      DemoParameter.para_object)


    # ----------------------------------------------------------------------------

    u = DemoParameter.controller.compute_setpoint_control_signal(joint_positions,vec8(pose_t))
    target_joint_positions = FunRobot.steplimit(u, step=0.3).reshape((6,)) * 0.2 + joint_positions

    DemoParameter.drawing.colorturn(cur_pose= pose_cur ,laserline=DemoParameter.robot_installation.laserline, color_turns=DemoParameter.color_turns, colorturn= DemoParameter.para.colorturn)

    target_joint_positions = target_joint_positions * 0.25 + pose_variable.target_joint_positions_p *0.75
    # Move the joints
    DemoParameter.client.rdi.send_target_joint_positions(target_joint_positions)

    pose_variable.target_joint_positions_p = target_joint_positions
    return True








