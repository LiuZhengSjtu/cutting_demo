

import copy
import numpy as np
import math
from dqrobotics import *
from numpy.ma.core import arccos, shape
from scipy.interpolate import CubicSpline




def points_update(para):
    points_a_o = []
    try:
        if para.linear_interpolation:
            x, y, z = 0.01, 0.0, 0.0
        else:
            x, y, z = para.bias

        d = para.scale
        points_a = {
            '1a':[x, y - d, z],
            '1b':[-x, y - d, z ],
            '2a':[x, y - d, z - d],
            '2b':[-x, y - d, z - d],
            '3a':[x, y + d , z - d],
            '3b':[-x, y + d  , z - d],
            '4a':[x, y + d  , z + d ],
            '4b':[-x, y + d  , z + d ],
            '5a':[x, y - d, z + d ],
            '5b':[-x, y - d, z + d ],
            '6a':[x, y - d, z ],
            '6b':[-x, y - d, z ]
        }

        points_a_o = {
            '1a':[x, y - d, z ],
            '1b':[-x, y - d, z ],
            '2a':[x, y - 0.5 * d, z - 0.8 * d],
            '2b':[-x, y - 0.5 * d, z - 0.8 * d],
            '3a':[x, y + 0.5 * d, z - 0.8 * d],
            '3b':[-x, y + 0.5 * d, z - 0.8 * d],
            '4a':[x, y + d , z ],
            '4b':[-x, y + d , z ],
            '5a':[x, y + d * 0.25, z + d * 0.0],
            '5b':[-x, y + d * 0.25, z + d * 0.0],
            '6a':[x, y + d * 0.5 , z  + d * 0.85],
            '6b':[-x, y + d * 0.5 , z  + d * 0.85],
            '7a':[x, y - d * 0.5, z + d * 0.0],
            '7b':[-x, y - d * 0.5, z + d * 0.0],
            '8a':[x, y - d, z ],
            '8b':[-x, y - d, z ],
        }
    except:
        pass

    return points_a_o



def pointsrevert(points):
    '''
    revert the trajectory direction
    :param points:
    :return:
    '''

    dic = copy.deepcopy(points)
    vs_ = []
    n = int(len(points)/2)
    for i in range(n):
        #   read two points
        key_ = str(i+1) + 'a'
        key_r = str(n-i) + 'a'
        dic[key_r] = np.array(points[key_])
        key_ = str(i+1) + 'b'
        key_r = str(n-i) + 'b'
        dic[key_r] = np.array(points[key_])

    return dic

def pointsmirror(points, para):
    '''
    mirror the trajectory direction
    :param points:
    :return:
    '''

    dic = copy.deepcopy(points)
    vs_ = []
    n = int(len(points)/2)
    try:
        if  para.linear_interpolation:
            k = 0
        else:
            k = 2
    except:
        pass
    for i in range(n):
        #   read two points
        key_ = str(i+1) + 'a'
        dic[key_] = np.array(points[key_])
        dic[key_][1] =  para.bias[1] * k - np.array(points[key_])[1]
        key_ = str(i+1) + 'b'
        dic[key_] = np.array(dic[key_])
        dic[key_][1] =  para.bias[1] * k - np.array(points[key_])[1]

    # print(f'orignal points = {points} \nafter mirror points = {dic}')

    return dic

def pointsrearrange(points, key_num):
    '''
    :param points: given a defined trajectory. divide this trajectory into two parts. drag the second half to the first half.
    :return:
    '''
    dic = {}
    n = int(len(points)/2)
    j = 1
    for i in range(1, n + 1):
        i_ = i + key_num - 1
        if i_ > n:
            i_ = i_ - n

        key_ = str(i_) + 'a'
        key_j = str(j) + 'a'
        dic[key_j] =  np.array( points[key_] )
        key_ = str(i_) + 'b'
        key_j = str(j) + 'b'
        dic[key_j] = np.array(  points[key_] )




        if j > 1:
            # print(    np.array(dic[str(j) + 'b'])  )
            # print(np.array(dic[str(j-1) + 'b']))
            # print( type(dic[str(j) + 'b']) )
            # print(type( np.array( dic[str(j) + 'b'] ) ))
            # print(type(dic[str(j-1) + 'b']))
            # print(dic[str(j) + 'b'].shape)
            #
            # print(np.array(dic[str(j) + 'b']) - np.array(str(j - 1) + 'b'))

            if np.sum(abs(  np.array(dic[str(j) + 'b'])  -  np.array( dic[str(j - 1) + 'b']  )))  + np.sum(abs( np.array( dic[str(j-1) + 'b'] )   -  np.array( dic[str(j - 1) + 'b' ] )  )) < 0.01 :
                continue

        j = j + 1


    #   compensate for the terminal point, should be the same as the initial point.
    #   in original points, the first and last are the same, so just n-1 points.
    #   after rearrangement, these two points overlap, so the extra point is ruled out
    dic[str(j) + 'a'] = dic['1a']
    dic[str(j) + 'b'] = dic['1b']
    return dic



def raicoletter(lettersize, scale):
    #   return a 2*n array, the first row is the x location and second is the y location of the letter (RAICo)
    #   bottom left is the origin. horizon width --x, vertical height --y
    #  --------------- following part is used to generate the letter 'RAICo'   --------------
        #   for each CAPITCAL letter size, the Rectangle Height = 1, With = 0.8. each letter occupies the 0.5 width.
        #   for each subscript letter size, the height = 0.5, width = 0.5. each letter occupies the 0.5 width.
    H = lettersize.H_letterspace
    W = lettersize.W_letterspace
    hc = lettersize.hc
    wc = lettersize.wc
    hs = lettersize.hs
    ws = lettersize.ws
    #  ----------------- for letter 'R'  -----------------
    r_y1 = np.linspace(0, H,  int(10 *  lettersize.density))
    r_x1 = np.linspace(0, 0, int(10 *  lettersize.density))
    r_y2 =  0.75*H + H*0.25 * np.cos(np.linspace(0, math.pi, int(10 *  lettersize.density)))
    r_x2 = 0.5 * H * np.sin(np.linspace(0,  math.pi, int(10 *  lettersize.density)))
    r_y3 = np.linspace(0.5 *H, 0, int(10 *  lettersize.density))
    r_x3 = np.linspace(0, H * 0.5, int(10 *  lettersize.density))

    r_y = r_y1.tolist() + r_y2.tolist() + r_y3.tolist()
    r_x = r_x1.tolist() + r_x2.tolist() + r_x3.tolist()

    #  -----------------  for letter 'A'  -----------------
    a_y_p = np.linspace(0, H*0.5, int(5 *  lettersize.density))

    a_y1 = np.hstack((a_y_p, a_y_p + H*0.5))
    a_x1 = np.hstack((a_y_p, a_y_p + H * 0.5)) * 0.25
    a_y2 = H - np.linspace(0, H*0.5, int(5 *  lettersize.density))
    a_x2 = a_y_p * 0.25 + 0.25 * H
    a_y3 = np.linspace(a_y2[-1], a_y2[-1], int(5 *  lettersize.density))
    a_x3 = 0.375 - a_y_p*0.5
    a_y4 = a_y3
    a_x4 = 0.125 + a_y_p*0.5
    a_y5 = a_y2 - H * 0.5
    a_x5 = a_x2 + 0.125*H

    a_y = a_y1.tolist() + a_y2.tolist() + a_y3.tolist() + a_y4.tolist() + a_y5.tolist()
    a_x = a_x1.tolist() + a_x2.tolist() + a_x3.tolist() + a_x4.tolist() + a_x5.tolist()
    a_x =  [item + W for item in a_x]


    #   -----------------  for letter 'I' -----------------
    i_y1 = np.linspace(0, H, int(10 *  lettersize.density))
    i_x1 = np.linspace(0.25*H, 0.25*H, int(10 *  lettersize.density))
    i_x_p = np.linspace(0, 0.1*H,int(3 *  lettersize.density))
    i_y2 = np.linspace(H, H, int(3 *  lettersize.density))
    i_x2 = 0.25 - i_x_p
    i_y3 = np.hstack((i_y2,i_y2))
    i_x3 = np.hstack((i_x_p - max(i_x_p) + 0.25*H, 0.25*H + i_x_p ))
    i_y4 = i_y2
    i_x4 = 0.25 - i_x_p + max(i_x_p)
    i_y5 = -i_y1 + max(i_y1)
    i_x5 = i_x1
    i_y = i_y1.tolist() + i_y2.tolist() + i_y3.tolist() + i_y4.tolist() + i_y5.tolist()
    i_x = i_x1.tolist() + i_x2.tolist() + i_x3.tolist() + i_x4.tolist() + i_x5.tolist()
    i_x = [item + W*2 for item in i_x]


    #   ----------------- for letter 'C' -----------------
    c_y1 = H * (0.5 + 0.5 * np.sin( -0.5*math.pi + np.linspace(0, math.pi, int(15 *  lettersize.density))))
    c_x1 = H *0.5 *(1 -  np.sin(np.linspace(0, math.pi, int(15 *  lettersize.density))) )
    c_y2 = H - c_y1
    c_x2 = c_x1
    c_y = c_y1.tolist() + c_y2.tolist()
    c_x = c_x1.tolist() + c_x2.tolist()
    c_x = [item + W*3 for item in c_x]


    #   -----------------  for letter 'o' -----------------
    o_y1 = 0.5 * hs * (1- np.cos(np.linspace(0, 2*math.pi, int(25 *  lettersize.density))))
    o_x1 = 0.5 * ( 1 + np.sin(np.linspace(0, 2*math.pi, int(25 *  lettersize.density))) )  * ws
    o_y = np.array(o_y1.tolist()).tolist()
    o_x = np.array(o_x1.tolist())
    o_x = [item + W*4 for item in o_x]

    #   connect above letters to one figure, RAICo
    connector_ra_y = np.linspace(0,0,int(10 *  lettersize.density))
    connector_ra_x = np.linspace(r_x[-1], a_x[0]  ,int(10 *  lettersize.density))
    connector_ai_y = np.linspace(0, 0, int(10 *  lettersize.density))
    connector_ai_x = np.linspace(a_x[-1], i_x[0]  , int(10 *  lettersize.density))
    connector_ic_y = np.linspace(0, 0, int(10 *  lettersize.density))
    connector_ic_x = np.linspace(i_x[-1], c_x[0] , int(10 *  lettersize.density))
    connector_co_y = np.linspace(0, 0, int(10 *  lettersize.density))
    connector_co_x = np.linspace(c_x[-1], o_x[0] , int(10 *  lettersize.density))
    connector_or_y = np.linspace(0, 0, int(30 *  lettersize.density))
    connector_or_x = np.linspace(o_x[-1], r_x[0] , int(30 *  lettersize.density))

    points_x = r_x + connector_ra_x.tolist() + a_x + connector_ai_x.tolist() + i_x + connector_ic_x.tolist() + c_x + connector_co_x.tolist() + o_x + connector_or_x.tolist()
    points_y = r_y + connector_ra_y.tolist() + a_y + connector_ai_y.tolist() + i_y + connector_ic_y.tolist() + c_y + connector_co_y.tolist() + o_y + connector_or_y.tolist()
    print(f'length is {len(points_y)}')
    for j in range(len(points_y)-1, 0, -1):
        if points_x[j]  == points_x[j-1] and points_y[j] == points_y[j-1]:
            points_y.pop(j)
            points_x.pop(j)
    print(f'after removing overlaps, length is {len(points_y)}')

    return np.vstack((np.array(points_x),np.array(points_y))) * scale



class parameter_turn_color():
    def __init__(self):
        self.ColorTurn_n = 0
        self.poses_reached = []
        self.poses_generate = []
        self.points_list_generate = []



def points2dq(dic ):
    '''
    :param dic: the point pairs dictionary. format as {'1a':[1.0,1.0,1.0], '1b':[1.0,1.0,1.0], '2a':[1.0,1.0,1.0], '2b':[1.0,1.0,1.0] }
    :return: the poses of lines defined by above paired points, and the list of the paired points coordinate system values.
    '''
    print('-' * 20 + ' points2dq start ' + '-' * 20)

    # print(f'dic = {dic}')
    color_turns = parameter_turn_color()
    color_turns.ColorTurn_n  = 0

    if len(dic) < 2 :
        raise ValueError(f'Points not enough, len(dic) = {len(dic)}, the length should be even number, and greater than 1')
    if len(dic) % 2 > 0 :
        raise ValueError(f'Points are not paired. len(dic) = {len(dic)}, odd number')

    #   record the poses in dq format, from paired points.
    #   e.g. poses = [0.57735i + 0.57735j + 0.57735k + E*( - 0.57735i + 1.154701j - 0.57735k), 0.31427i - 0.392837j + 0.864242k + E*(0.549972i + 0.094281j - 0.157135k)]

    #   then, generate the list of this series of point
    #   e.g. points_list = [[1.0, 2.0, 3.0, 2.0, 3.0, 4.0], [0.0, 0.5, 0.3, 0.4, 0.0, 1.4]]

    poses = []
    points_list = []
    for i in range(int(len(dic)/2)):
        #   read two points
        key_ = str(i+1) + 'a'
        value_a = np.array(dic[key_])
        key_ = str(i+1) + 'b'
        value_b = np.array(dic[key_])

        v = value_b - value_a
        v = v / np.linalg.norm(v)
        #   line vector l, and sXl
        l = v
        d = np.cross(value_b, l)
        pose_line = DQ(l) + E_ * DQ(d)
        poses.append(pose_line)

        v = np.hstack((value_a,value_b)).tolist()
        points_list.append(v)

    color_turns.poses_reached = np.zeros(len(poses))
    color_turns.points_list_generate = points_list
    color_turns.poses_generate = poses
    print('*' * 20 + ' points2dq finished ' + '*' * 20)
    # print(f'point list = {points_list}')
    return poses, points_list, np.zeros(len(poses)), color_turns





def poses2trajectory(pose_home, desiredposes, durations , robot_installation, para,  duration_first = 12.5, ):
    '''
    :param pose_home:   DQ 8, for pose of ur3e with tool
    :param desiredposes:   a sequence of desired line poses, DQ
    :param durations:   list of time duration, towards the nth desired line, like [10,4,5,7]. if durations is a float, it means the time interval between adjacent line DQ
                        #   in this case, the time interval from inital robot pose to first desired line should be specified seperately, by param 'duration_first'
    :return:    a sequence of DQ line poses

    for pose_home, convert to vector + point, as the first predefined line pose
    for desiredposes, convert to vector + point, as the following predefined line pose

    Then, for each dimension, seperately plan trajectory.

    then integrate all trajectories into line poses DQ.
    '''
    print('-' * 20 + ' poses2trajectory start ' + '-' * 20)
    #   calculate home state laser line vector and one point on this line.
    home_linevect =  vec3(  P(pose_home) *  DQ( robot_installation.l_laser_e)  * conj(P(pose_home)) )
    home_linepoint = vec3( translation(pose_home) )

    if isinstance(durations, list):
        if len(desiredposes) != len(durations) and len(durations) > 1 :
            raise ValueError(f'WayConfig.py: shape of desiredposes = {shape(desiredposes)}, but shape of durations = {shape(durations)}')

    try:
        if not para.linear_interpolation   :
            durations = durations * para.intervalnum
    except :
        pass

    # print(f'durations = {durations} \ndesiredposes = {desiredposes}')


    l_vects = []
    l_points = []
    l_vects.append(home_linevect)
    l_points.append(home_linepoint)

    if shape(desiredposes)[0] == 1:
        #   only one pose predefined
        l_vect = P(desiredposes)
        l_point = np.cross(vec3(l_vect), vec3(D(desiredposes)))
        l_vects.append(vec3(l_vect))
        l_points.append(l_point)
    else:
        #   multi poses predefined
        for l_pose in desiredposes:
            l_vect = P(l_pose)
            l_point = np.cross(vec3(l_vect), vec3(D(l_pose)))
            l_vects.append(vec3(l_vect) )
            l_points.append(l_point)

        #   the time instant at each predifined line pose
    duration_seq = np.zeros(len(desiredposes) + 1)
    for i in range(len(duration_seq)):
        if i > 0:
            if isinstance(durations, float) or isinstance(durations, int):
                duration_seq[i] = durations + duration_seq[i - 1]
                if i == 1:
                    duration_seq[i] = duration_first
            else:
                duration_seq[i] = durations[i - 1] + duration_seq[i - 1]

    # print(f'the duration_seq = {duration_seq}')
    # print(f'the desiredposes = {desiredposes}')

    for i in range(3):
        if i == 0:
            l_vects_dim = [val[i] for val in l_vects]
            l_points_dim = [val[i] for val in l_points]
        else:
            l_vects_dim = np.vstack((l_vects_dim, [val[i] for val in l_vects]))
            l_points_dim = np.vstack((l_points_dim, [val[i] for val in l_points]))
    #   l_seq_dims, from 1st to 6th row, it represents vect-x-y-z and point-x-y-z respectively, in world frame.
    l_seq_dims = np.vstack((l_vects_dim, l_points_dim))

    #   plan each dimension of the 6D line poses
    duration_seq_plan = np.arange(0, duration_seq[-1],  robot_installation.Ts)
    # print(f'the duration_seq_plan = {duration_seq_plan}')

    #   set up the start and ending first order derivation, velocity, to zero
    bc = ((1, 0.0), (1, 0.0))
    # print(f'duration_seq = {duration_seq} \n l_seq_dim = {l_seq_dims}')
    for i in range(6):
        cs = CubicSpline(duration_seq, l_seq_dims[i], bc_type=bc)
        if i == 0:
            l_seq_dims_plan = cs(duration_seq_plan)
        else:
            l_seq_dims_plan = np.vstack((l_seq_dims_plan, cs(duration_seq_plan)))

    #   resize the norm of the vector to 1. this part can ge omitted if the resize funtion is done in following array --> dq
    for i in range(shape(l_seq_dims_plan)[1]):
        vect = l_seq_dims_plan[0:3, i]
        vect = vect / np.linalg.norm(vect)
        l_seq_dims_plan[0:3, i] = vect

    #   array --> dq, turn the line info to dq. generate the planned line poses.
    l_poses_plan = []
    for i in range(shape(l_seq_dims_plan)[1]):
        v = l_seq_dims_plan[0:3, i]
        v = v / np.linalg.norm(v)
        #   line vector l, and use  s x l
        l = v
        d = np.cross(l_seq_dims_plan[3:6, i], l)
        pose_line = DQ(l) + E_ * DQ(d)
        l_poses_plan.append(pose_line)

    print(f'size of l_poses_plan = {len(l_poses_plan)}, and the inital state l_poses_plan[0] = {l_poses_plan[0]}')
    print('*' * 20 + ' poses2trajectory finished ' + '*' * 20)
    return l_poses_plan






def pointsdense(points, interv = 20):
    '''
    make the points more dense, suitable for trajectory tracking. linear interpolation
    :param points: some SPARE point pairs dictionary. format as {'1a':[1.0,1.0,1.0], '1b':[1.0,1.0,1.0], '2a':[1.0,1.0,1.0], '2b':[1.0,1.0,1.0] }
    :param interv: the interpolation number
    :return: return 2 * n array of a sequence of dense points
    '''

    dic = points



    vs_ = []
    for i in range(int(len(dic)/2)):
        #   read two points
        key_ = str(i+1) + 'a'
        value_a = np.array(dic[key_])
        key_ = str(i+1) + 'b'
        value_b = np.array(dic[key_])

        v = np.hstack((value_a, value_b)).tolist()
        #   v, 1*6 , (xa,ya,za,xb,yb,zb)
        vs_.append(v)
    vs = np.array(vs_)
    # print(f'dic = {dic}')
    # print(f'vs = {vs}')
    #   vs, n*6

    ho = []
    ve = []
    for i in range(vs.shape[0] -1 ):
        ho = ho + np.linspace(vs[i][1], vs[i + 1][1], interv).tolist()
        ve = ve + np.linspace(vs[i][2], vs[i + 1][2], interv).tolist()

    for j in range(len(ho)-1, 0, -1):
        if ho[j]  == ho[j-1] and ve[j] == ve[j-1]:
            ho.pop(j)
            ve.pop(j)
    # print(f'ho = {ho}')
    # print(f've = {ve}')
    return np.vstack((np.array(ho), np.array(ve)))



def generate_points_plane(points, para , xyz='xy'):
    '''
    :param points: 2*n. x and y (generally dense) reflect location of the letter RAICo
            xyz     indicate the plane in world frame
    :return: the dictionary of paired points. paired points form a line, vertical the floor
    '''
    if xyz == 'xy':
        #   resize RAICo
        points = points * para.scale
        #   fetch out x and y
        points_y_np =  points[0]
        points_x_np =  -points[1]
        raicopoints = {}
        bias =  para.bias
        for k in range(len(points_y_np)):
            #   a and b items
            a = np.array([points_x_np[k], points_y_np[k], 0]) + bias
            b = np.array([points_x_np[k], points_y_np[k], -0.05]) + bias
            raicopoints[str(k + 1) + 'a'] = a.tolist()
            raicopoints[str(k + 1) + 'b'] = b.tolist()

        return raicopoints
    elif xyz == 'yz':
        #   resize
        points = points
        #   fetch out x and y
        points_y_np = points[0]
        points_z_np = points[1]
        linepoints = {}
        # bias = np.array([0.05, 0, 0.95])
        bias =  para.bias
        for k in range(len(points_y_np)):
            #   a and b items
            a = np.array([0, points_y_np[k], points_z_np[k]]) + bias
            b = np.array([-0.05, points_y_np[k], points_z_np[k]]) + bias
            linepoints[str(k + 1) + 'a'] = a.tolist()
            linepoints[str(k + 1) + 'b'] = b.tolist()


        return linepoints

def generate_points_curve(points, objects, para):
    '''
    The projection from the letter figure to the curved trajectory.
    The image should not be projected downward directly to the cylinder surface. (BEFORE)
    The image should be projected to the cylinder axis. (AFTER)

    :param points: 2*n. x and y location of the letter RAICo
    :return: the dictionary of paired points. paired points form a line, vertical to the floor
    '''

    #   resize
    points = points *  para.scale
    #   fetch out x and y
    points_x_np =  points[0]
    points_y_np =  points[1]
    raicopoints = {}
    bias =  para.bias

    #   the x, y, z axes of the curve cylinder, expressed in world frame.
    #   origin at the center of the cyliner, z axis along the cylinder center line, y axis upward.
    curve_o =  objects.cylinder_h.translation
    curve_z =   objects.cylinder_h.vect
    # print(f'the curve_z = {curve_z}')
    curve_y = np.array([0.0, 0.0, 1.0])
    curve_x = np.cross(curve_y,curve_z)


    shift_orign = bias - curve_o
    traj_r =  np.sqrt(shift_orign[0]**2 + shift_orign[2]**2)
    traj_theta = math.asin(shift_orign[0] / traj_r)

    for k in range(len(points_y_np)):
        #   a and b items
        #   the frame transformation from figure to the simulation world frame
        a_letter = np.array([-points_y_np[k], points_x_np[k], 0]) #+ bias

        letter_theta = traj_theta + a_letter[0] / traj_r    #  math.asin(  a_letter[0] / traj_r  )

        letter_ya = bias[1] + a_letter[1]
        letter_xa = curve_o[0] + math.sin(letter_theta) * traj_r
        letter_za = curve_o[2] + math.cos(letter_theta) * traj_r

        letter_yb = letter_ya
        letter_xb = curve_o[0] + math.sin(letter_theta) * traj_r * 0.9
        letter_zb = curve_o[2] + math.cos(letter_theta) * traj_r * 0.9



        a = np.array([ letter_xa, letter_ya, letter_za ])
        b = np.array([letter_xb, letter_yb, letter_zb])

        raicopoints[str(k + 1) + 'a'] = a.tolist()
        raicopoints[str(k + 1) + 'b'] = b.tolist()

    return raicopoints


def generate_points_sphere(points, objects, para):
    '''
    :param points: 2*n.        x and y location of the letter RAICo
    :return: the dictionary of paired points. paired points form a line, vertical to the sphere surface

    calculate the bias related to the sphere center. yaw: rotation angle along z axis (world frame). pitch: the angle deviating from
    '''
    #   resize
    points = points *  para.scale
    #   fetch out x and y
    points_x_np =  points[0]
    points_y_np =  points[1]
    raicopoints = {}
    bias =  para.bias

    curve_o =  objects.sphere.translation

    shift_orign = bias - curve_o
    traj_r = np.linalg.norm(shift_orign)
    traj_r_xy = np.linalg.norm(shift_orign[0:2])
    traj_yaw = math.asin( shift_orign[1] / traj_r_xy )
    traj_pitch = math.asin(shift_orign[2] / traj_r)

    print(f'\n\nshift_origin = {shift_orign}\ntraj_r = {traj_r}\ntraj_r_xy = {traj_r_xy}')
    print(f'\n\nbias = {bias}\ncurve_o = {curve_o}\ntraj_yaw = {traj_yaw}\ntraj_pitch = {traj_pitch}')

    for k in range(len(points_y_np)):
        #   a and b items
        #   the frame transformation from figure to the simulation world frame
        a_letter = np.array([0, points_x_np[k], points_y_np[k]]) #+ bias

        letter_yaw = traj_yaw + a_letter[1] / traj_r_xy
        letter_pitch = traj_pitch + a_letter[2] / traj_r

        letter_v = np.array([ math.cos(letter_yaw) *  math.cos(letter_pitch) ,  math.sin(letter_yaw) *  math.cos(letter_pitch) ,  math.sin(letter_pitch)   ]) * traj_r

        a = curve_o + letter_v
        b = curve_o + letter_v * 0.9

        raicopoints[str(k + 1) + 'a'] = a.tolist()
        raicopoints[str(k + 1) + 'b'] = b.tolist()

    return raicopoints



def getdesiredpoints(para, cylinder_cube,lettersize, objects):
    if cylinder_cube == 0:

        ps = points_update(para=para)

        if para.pointmirror:
            ps = pointsmirror(ps, para=para)
        if  para.pointrevert:
            ps = pointsrevert(ps)
        if  para.pointrearrange:
            ps = pointsrearrange(ps, key_num=4)

        if  para.linear_interpolation:
            dense = pointsdense( ps , interv=  para.intervalnum)
            desiredpoints = generate_points_plane(dense, para= para ,xyz='yz')

        else:
            desiredpoints = ps

    elif   cylinder_cube == 1:
        desiredpoints = generate_points_plane(raicoletter(lettersize=lettersize, scale= para.scale), para=para, xyz=  para.plane)
    elif  cylinder_cube == 2:
        desiredpoints = generate_points_curve(raicoletter(lettersize=lettersize, scale= para.scale), objects=objects, para=para)
    elif cylinder_cube == 3:
        desiredpoints = generate_points_sphere(raicoletter(lettersize=lettersize, scale=para.scale), objects=objects,
                                              para=para)

    return desiredpoints



class pose_variable_class():
    def __init__(self):
        #   save the initial pose
        self.pose0 = []
        self.tra0 = []
        self.rot0 = []

        self.target_joint_positions_p = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.position_home = []
        self.pose_home = []
        self.poses = []
        self.points_list = []
        self.poses_reached = []
        # ------------------------------------------------------------------------------------------------------
        self.l_poses_plan = []