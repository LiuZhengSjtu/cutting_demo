

from dqrobotics import *
import numpy as np
import math


def coppelisimPose2dq(pose):
    t = DQ(pose[0:3])
    r = DQ([pose[6]] + pose[3:6])
    return  r + E_ * t * r


def cal_pose_plane(vect, point):
    return DQ(vect) + E_ * np.dot(point, vect)



def cal_pose_line(vect, point):
    return DQ(vect) + E_ * DQ(np.cross(point, vect))


def get_distance(line_pose,robot_pose, laserline_pose):
    '''
    :param line_pose: the desired line dual quaternion
    :param robot_pose: the cutting laser pose dual quaterion (usually overlap with Z axis of the tool)
    :param laserline_pose: the laser line pose in end-effector frame
    :return: the angular distance dis_v and the point-line distance dis_l

    point-line distance.
    Given cutting laser 3D point P_c, a 3D point on the line P_l, and the line vector v_l,
    the point-line distance is defined as:
    dis_l = || ( P_c - P_l ) X v_l || / || v_l ||

    angular distance.
    Give cutting laser rotaiton quaternion  v_c, and line rotation quaternion v_l,
    the norm of their dot product is
    dv = norm( dot( v_c, v_l))
    and
    dis_v = cos^{-1} (dv)

    or
    dis_v = 1-dv^2


    note, for C = A x B,
    B = C X A / |A|^2
    A = B X C  / |B|^2

    '''

    #   regarding the laser line

    #   the robot pose --> rotation quaternion and translation quaternion.
    r = rotation(robot_pose)
    p = translation(robot_pose)

    #   the laser line vector, from end-effector frame to world frame
    l_laser = vec3(r * P(laserline_pose) * conj(r))
    #   one point vector on the laser line . if possible, this point can also be specified in the end-effector frame
    p_laserline_ee = np.cross(l_laser, vec3(D(laserline_pose))  )
    p_laser = vec3( p + r * DQ(p_laserline_ee) * conj(r) )

    # print(f'l_laser = {l_laser}, p_laser = {p_laser}')

    #   regarding the desired line
    #   the direction vector, and one point location vector
    l_pose = vec3(P(line_pose))
    p_pose = np.cross(l_pose   ,  vec3(D(line_pose)))

    dis_l = np.linalg.norm( np.cross((p_laser - p_pose), l_pose ) )
    dis_v = math.acos(np.sum( l_pose * l_laser ))       #   rad, 0 ~ pi
    #   return the distance from one point on laser to desired line, and distance from laser direction vector to desired line direction vector.
    return dis_l, dis_v