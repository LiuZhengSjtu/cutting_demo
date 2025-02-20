#!/usr/bin/python3
'''

This file is imported by ExampleClient.py
This file define the DH parameter for UR3

The world frame is configured.
The tool frame is configured.

'''

print('start the robotconfig.py')
import time
from sas_common import rclcpp_spin_some
from dqrobotics.utils import DQ_Geometry
from dqrobotics.solvers import DQ_QuadprogSolver
from dqrobotics.robot_control import ControlObjective, DQ_PseudoinverseController, DQ_ClassicQPController, DQ_QuadraticProgrammingController
import numpy as np
from dqrobotics import *
from math import *
from dqrobotics.robot_modeling import DQ_SerialManipulatorDH


import FunDq




class robotDHclass:
    def __init__(self):
        #   parameters in ur3e DH
        self.dh =   {
                    'd1' : 0.1519,
                    'd4' : 0.11235,
                    'd5' :0.08535,
                    'd6' : 0.0819,
                    'a2' : -0.24365,
                    "a3" : -0.21325
                    }

        #   setup the supporting base. from robot base frame to world frame.
        #   compare the base frame of the ur in coppeliasim and ur official website, there is a 90 degree rotation around Z
        t_support = DQ([0, 0.4 , 0 , 0.6])          #   translation from robot base to world, expressed on world frame

        c = cos(-45 * pi / 180 * 0.5)
        s = sin(-45 * pi / 180 * 0.5)
        r_support = DQ([c, s * 0, s * 0, s * 1])    #   rotation from robot base to world, expressed on world frame
        self.h_ur2w = r_support + E_ * 0.5 * t_support * r_support

        print(f'the translatio of ur2w = {translation(self.h_ur2w)} \nrotation = {rotation(self.h_ur2w)} \n rotation angle = {rotation_angle(self.h_ur2w)} \n rotation axis = {rotation_axis(self.h_ur2w)}')

        #   setup the tool. from tool frame to the terminal joint frame. it is considered in constraint avoidance calculation
        c = cos(-90 * pi / 180 * 0.5)
        s = sin(-90 * pi / 180 * 0.5)


        if False:
        #   False for the parallel torch -------------------------------------
            t_tool = DQ([0.0, 0.05, 0.05, +0.035])
            r_tool = DQ([1, 0, 0, 0.0])
        else:
        #   True for the vertical torch -------------------------------------
            t_tool = DQ([0.0, 0.0, -0.05, +0.030])
            r_tool = DQ([c, -s, 0.0, 0.0])
        self.h_t2ure = r_tool + E_ * 0.5 * t_tool * r_tool


        #   two example home joint positions.
        self.position_home = np.array([[-55, -75, -120, -90, 90, 0], [-55, -75, -80, -125, 130, 90] , [65, 30, 90, -30, -90, 0],[-55, -75, -120, -90, 110, 90]])/180* pi
        self.pose_home = []

        #      [-10, -55, -100, -125.0, 20, 90]

        #   set up the laser line pose related to end-effector. it is considered in line tracking.
        self.l_laser_e = np.array([ 0.0, 0.0, 1.0])  # laser line unit vector related to end-effector frame, usually along Z axis of the end-effector frame
        p_laser_e = np.array([ 0.0, 0.0, 0.0])
        self.laserline = DQ(self.l_laser_e) +  E_ * DQ(np.cross(p_laser_e , self.l_laser_e))

        #   time interval in robot control loop
        self.Ts = 0.01





class robotclass:
    def __init__(self):
        self.DH = robotDHclass()
        d1 = self.DH.dh['d1']
        d4 = self.DH.dh['d4']
        d5 = self.DH.dh['d5']
        d6 = self.DH.dh['d6']
        a2 = self.DH.dh['a2']
        a3 = self.DH.dh['a3']
        # robot_DH_theta = np.array([0, -pi / 2, 0, -pi / 2, 0, 0])  # ---- different from official web, here add the offset
        robot_DH_theta = np.array([0, 0, 0, 0, 0, 0])
        robot_DH_d = np.array([d1, 0, 0, d4, d5, d6])
        robot_DH_a = np.array([0, a2, a3, 0, 0, 0])
        robot_DH_alpha = np.array([pi / 2, 0, 0, pi / 2, -pi / 2, 0])
        robot_type = np.array([0, 0, 0, 0, 0, 0])
        robot_DH_matrix = np.array([robot_DH_theta, robot_DH_d, robot_DH_a, robot_DH_alpha, robot_type])
        print(f'finish DH matrix')
        robotUR3 = DQ_SerialManipulatorDH(robot_DH_matrix)
        print(f'finish the ur3 DH configuration')


        #   setup the supporting base. from robot base frame to world frame.
        h_ur2w = self.DH.h_ur2w
        print(f'the transformation from ur to world frame is {h_ur2w}, and its translation is {translation(h_ur2w)}')

        #   setup the tool. from tool frame to the terminal joint frame.
        h_t2ure = self.DH.h_t2ure

        #   impose the robot reference (world frame), and the tool frame.
        robot = robotUR3
        robot.set_base_frame(h_ur2w)
        robot.set_reference_frame(h_ur2w)
        robot.set_effector(h_t2ure)

        self.robot = robot
        print(f'finish the whole robot configuration')

        #   two example home joint positions.
        position_home = self.DH.position_home
        for p in position_home:
            self.DH.pose_home.append( robot.fkm( p))
        pose_home = self.DH.pose_home

        self.position_home = position_home
        self.pose_home = pose_home
        print('*'*20 + ' robot DH configuration finished ' + '*'*20)






robot_hd = robotclass()
robot_installation = robotDHclass()


    #  ---------- set up controller for pose control, using DQ_ClassicQPController  ----------
    #       the controller cannot be configured in class, must be created here
print('-'*20 + ' controllers configuration start ' + '-'*20)

qp_solver = DQ_QuadprogSolver()
ctrl_pose_qp = DQ_ClassicQPController(robot_hd.robot, qp_solver)
ctrl_pose_qp.set_control_objective(ControlObjective.Pose)
ctrl_pose_qp.set_gain(10.0)
ctrl_pose_qp.set_damping(0.001)


#  ---------- set up controller for pose control, using DQ_PseudoinverseController  ----------
ctrl_pose_inv = DQ_PseudoinverseController(robot_hd.robot)
ctrl_pose_inv.set_control_objective(ControlObjective.Pose)
ctrl_pose_inv.set_gain(10.0)
ctrl_pose_inv.set_damping(0.001)


#  ----------  set up controller for line control, using DQ_PseudoinverseController  ----------
ctrl_line_inv = DQ_PseudoinverseController(robot_hd.robot)
#   laser line in end-effector frame
ctrl_line_inv.set_primitive_to_effector(robot_installation.laserline)
ctrl_line_inv.set_control_objective(ControlObjective.Line)
ctrl_line_inv.set_gain(10.0)
ctrl_line_inv.set_damping(0.001)

#------------------------------------------------------------------------------------------------

#  ----------  set up controller for line control, using DQ_ClassicQPController  ----------
qp_solver = DQ_QuadprogSolver()
ctrl_line_qp = DQ_ClassicQPController(robot_hd.robot, qp_solver)
ctrl_line_qp.set_primitive_to_effector(robot_installation.laserline)
ctrl_line_qp.set_control_objective(ControlObjective.Line)
ctrl_line_qp.set_gain(10.0)
ctrl_line_qp.set_damping(0.001)


    #------------------------------------------------------------------------------------------------

def get_controller(tp = 4):
    global robot_installation, robot_hd

    if tp == 1:
        controller = ctrl_pose_qp
    elif tp == 2:
        controller = ctrl_pose_inv
    elif tp == 3:
        controller = ctrl_line_inv
    elif tp == 4:
        controller = ctrl_line_qp

    return robot_installation, robot_hd, controller



def steplimit(u,step = 0.1):
    angles = np.sum(abs(u))
    if angles > step:
        ulimit = u / angles * step
    else:
        ulimit = u
    return  ulimit


class constraint:
    def __init__(self, cylinder_or_cube, para, objects ):
        if para.constraint_manual == True:
            if para.constraint_plane == True:
                self.dq = FunDq.cal_pose_plane(para.constraint_vect, para.constraint_point)
            elif para.constraint_line == True:
                self.dq = FunDq.cal_pose_line(para.constraint_vect, para.constraint_point)
            elif para.constraint_sphere == True:
                self.dq = DQ(para.constraint_point)
        elif cylinder_or_cube == 0:
            # self.dq = WayConfig.objects.cylinder_v.dq
            self.dq = objects.cube.dq
        elif cylinder_or_cube == 2:
            self.dq = objects.cylinder_h.dq
        elif cylinder_or_cube == 1:
            self.dq = objects.cube.dq
            print(f'dq of cube constraint is {self.dq}')
        elif cylinder_or_cube == 3:
            self.dq = objects.sphere.dq




vlty = 0.025 * 1
def homepose(pose, robot_hd, robot_installation, client):
    global ctrl_pose_inv
    print('-' * 20 + ' home pose beginning ' + '-' * 20)
    # import FunCtroller
    controllerhome= ctrl_pose_inv
    while 1:
        joint_positions = client.rdi.get_joint_positions()
        cur_pose = robot_hd.robot.fkm(joint_positions)
        if np.sum(abs(vec8(cur_pose - pose))) < 0.001:
            break
        u = controllerhome.compute_setpoint_control_signal(joint_positions, vec8(pose))
        target_joint_positions = steplimit(u, step=0.5).reshape((6,)) * vlty + joint_positions
        # Move the joints
        client.rdi.send_target_joint_positions(target_joint_positions)
        rclcpp_spin_some(client.node)

    time.sleep(0.2)
    joint_positions = client.rdi.get_joint_positions()
    pose = robot_hd.robot.fkm(joint_positions)
    print(f'current pose after homepose is {pose}')
    t = translation(pose)
    r = rotation(pose)
    l_vect = r * DQ( robot_installation.l_laser_e) * conj(r)
    laser_dq = l_vect + E_ * DQ(np.cross(vec3(t), vec3(l_vect)))
    print(f'at home pose, the laser line dq is {laser_dq}')
    print('*' * 20 + ' home pose end ' + '*' * 20)


def homeposition(position, robot_hd, client, robot_installation):
    print('-' * 20 + ' home position beginning ' + '-' * 20)
    print(f'target position is {position}')
    while 1:

        joint_positions =  client.rdi.get_joint_positions()
        u = steplimit(position - joint_positions, step= 0.5) * vlty
        target_positions = u + joint_positions

        # print(f'approaching to homeposition, error = {position - joint_positions}')

        if np.sum(abs( position - joint_positions )) < 0.025:
            break
        client.rdi.send_target_joint_positions( target_positions )
        rclcpp_spin_some( client.node)



    time.sleep(0.2)
    joint_positions =  client.rdi.get_joint_positions()
    pose = robot_hd.robot.fkm(joint_positions)
    print(f'current pose after homeposition is {pose}')

    t = translation(pose)
    r = rotation(pose)

    l_vect = r * DQ( robot_installation.l_laser_e) * conj(r)
    laser_dq = l_vect + E_ * DQ(np.cross(vec3(t), vec3(l_vect)))
    print(f'at home position, the laser line dq is {laser_dq}')
    print('*'*20 + ' home position end ' + '*'*20)

def savepose(robot_hd,client):
    print('-' * 20 + ' save pose beginning ' + '-' * 20)
    pose0 = robot_hd.robot.fkm(client.rdi.get_joint_positions())
    tra0 = translation(pose0)
    rot0 = rotation(pose0)

    print(f'pose0= = {pose0} \n tra0 = {tra0} \n rot0 = {rot0} ')
    print('*' * 20 + ' save pose end ' + '*' * 20)
    return pose0, tra0, rot0

def point2planeconstraint(joint_positions, robot_hd, cons_object, ctrl_line_qp, para_object):
#    for the distance constraint from the cube upper surface to the tool
    pose_cur = robot_hd.robot.fkm(joint_positions)
    J_pose_cur = robot_hd.robot.pose_jacobian(joint_positions)

    tra_cur = translation(pose_cur)
    Jtra = robot_hd.robot.translation_jacobian(J_pose_cur, pose_cur)
    Jtra_fs = np.concatenate((Jtra, np.zeros((4, 6))), axis=0)
    J_dist_cubeplane = robot_hd.robot.point_to_plane_distance_jacobian(Jtra_fs, tra_cur,cons_object.dq)
    dist_cubeplane = DQ_Geometry.point_to_plane_distance(tra_cur, cons_object.dq)
    dist = np.reshape(dist_cubeplane, (1, 1))

    A = np.concatenate((J_dist_cubeplane, -J_dist_cubeplane), axis=0)
    obj = para_object
    distance  = obj.object.distance
    a = np.concatenate((distance[0] - dist, dist - distance[1]), axis=0)
    #   Aq < a
    ctrl_line_qp.set_inequality_constraint(A, a)

    # print(f'the current translation is {tra_cur}\n the constraint.dq = {cons_object.dq}\nthe distance range = {distance}\npractical dist = {dist}\n\n')

    return pose_cur


def point2lineconstraint(joint_positions, robot_hd, cons_object, ctrl_line_qp, para_object):

    pose_cur = robot_hd.robot.fkm(joint_positions)
    J_pose_cur = robot_hd.robot.pose_jacobian(joint_positions)

    tra_cur = translation(pose_cur)
    Jtra = robot_hd.robot.translation_jacobian(J_pose_cur, pose_cur)
    Jtra_fs = np.concatenate((Jtra, np.zeros((4, 6))), axis=0)
    # print(f'following is error at i = ')
    # print(f'Jtra_fs = {Jtra_fs}\ntra_cur = {tra_cur}\ncons_object.dq = {cons_object.dq}')
    J_dist_cylinderline = robot_hd.robot.point_to_line_distance_jacobian(Jtra_fs, tra_cur, cons_object.dq)
    # print(f'at i = ')
    dist_cylinderline = DQ_Geometry.point_to_line_squared_distance(tra_cur, cons_object.dq)
    dist = np.reshape(np.sqrt(dist_cylinderline) , (1,1))

    A = np.concatenate((J_dist_cylinderline, -J_dist_cylinderline), axis=0)
    a = np.concatenate((para_object.object.distance[0] - dist, dist - para_object.object.distance[1] ), axis=0)

    ctrl_line_qp.set_inequality_constraint(A, a)

    return pose_cur


def point2point(joint_positions, robot_hd, cons_object, ctrl_line_qp, para_object):

    pose_cur = robot_hd.robot.fkm(joint_positions)
    J_pose_cur = robot_hd.robot.pose_jacobian(joint_positions)

    tra_cur = translation(pose_cur)
    Jtra = robot_hd.robot.translation_jacobian(J_pose_cur, pose_cur)
    Jtra_fs = np.concatenate((Jtra, np.zeros((4, 6))), axis=0)
    # print(f'following is error at i = ')
    # print(f'Jtra_fs = {Jtra_fs}\ntra_cur = {tra_cur}\ncons_object.dq = {cons_object.dq}')
    J_dist_point = robot_hd.robot.point_to_point_distance_jacobian(Jtra_fs, tra_cur, cons_object.dq)
    # print(f'at i = ')
    dist_point = DQ_Geometry.point_to_point_squared_distance(tra_cur, cons_object.dq)
    dist = np.reshape(np.sqrt(dist_point) , (1,1))

    A = np.concatenate((J_dist_point, -J_dist_point), axis=0)
    a = np.concatenate((para_object.object.distance[0] - dist, dist - para_object.object.distance[1] ), axis=0)

    ctrl_line_qp.set_inequality_constraint(A, a)

    return pose_cur