


#   for update parameters


print('start the DemoParameter.py')

import numpy as np



from FunRobot import constraint, get_controller
from FunInterface import get_interface, statistic_rotate
from FunClient import SASClient
from FunTrajectory import parameter_turn_color

import FunParameter


print('-'*20 + ' DemoParameter setup start ' + '-'*20)



para = FunParameter.parameter_class()







para_object,  lettersize= FunParameter.get_para(para)

#   find the host ip    # #   for desired line color turn
drawing, objects, move_skip,  stepmotor = get_interface(coppeliasim=para.coppeliasim)
color_turns = parameter_turn_color()
robot_installation, robot_hd, controller = get_controller(tp=4)

cons_object = constraint(para_object.cylinder_cube, para, objects=objects)

client = SASClient(robot_installation.Ts)


def update_par():
    global para, para_object, lettersize, drawing, objects, move_skip, stepmotor, cons_object


    para_object, lettersize = FunParameter.get_para(para)

    #   find the host ip    # #   for desired line color turn
    drawing, objects, move_skip, stepmotor = get_interface(coppeliasim=para.coppeliasim)


    cons_object = constraint(para_object.cylinder_cube, para, objects=objects)







# -------------------------------------------------------------------
#   adjust the motion speed.
dur = 1.5 / 1
#   in FunRobot.py, vlty is used to adjust the first_duration speed limit.

def reconfig(j):
    global para, para_object
    if j == 0:

        if 0:
            #   for curve surface cutting on a cylinder
            para.lines_type = 2

            para.coppeliasim = True

            para.durations = dur
            para.intervalnum = 15
            para.linear_interpolation = True

            para.constraint_manual = False
            para.constraint_plane = False
            para.constraint_line = False
            para.constraint_sphere = True
            para.constraint_vect = [1.0, 0.0, 0.0]
            para.constraint_point = [-0.2, 0.1, 0.875]
            para.constraint_dis = [0.3, 0.25]

            para.torch_vertical = True

            para.scale = 0.2
            para.bias = np.array([0.05, -0.15, 0.775])
            para.pointrevert = False
            para.pointmirror = False
            para.pointrearrange = False
        else:
            para.lines_type = 3

            para.coppeliasim = True

            para.durations = dur
            para.intervalnum = 15
            para.linear_interpolation = True

            para.constraint_manual = True
            para.constraint_plane = False
            para.constraint_line = False
            para.constraint_sphere = True
            para.constraint_vect = [1.0, 0.0, 0.0]
            para.constraint_point = [-0.2, 0.1, 0.875]
            para.constraint_dis = [0.275, 0.25]

            para.torch_vertical = True

            para.scale = 0.3
            para.bias = np.array([0.05, -0.0, 0.8])
            para.pointrevert = False
            para.pointmirror = False
            para.pointrearrange = False



    elif j == 2:
        para.lines_type = 0

        para.coppeliasim = True

        para.durations = dur
        para.intervalnum = 15
        para.linear_interpolation = True

        para.constraint_manual = True
        para.constraint_plane = True
        para.constraint_line = False
        para.constraint_vect = [1.0, 0.0, 0.0]
        para.constraint_point = [-0.05, 0, 0]
        para.constraint_dis = [0.05, 0.025]

        para.torch_vertical = True

        para.scale = 0.1
        para.bias = np.array([0.05, 0.08, 0.89])
        para.pointrevert = False
        para.pointmirror = True
        para.pointrearrange = True
    elif j == 3:
        para.lines_type = 1

        para.coppeliasim = True

        para.durations = dur * 0.75

        plane = 'yz'
        if plane == 'xy':
            para.constraint_manual = False
        else:
            para.constraint_manual = True
        para.constraint_plane = True
        para.constraint_line = False
        para.constraint_vect = [1.0, 0.0, 0.0]
        para.constraint_point = [-0.05, 0, 0]
        para.constraint_dis = [0.05, 0.025]

        para.torch_vertical = True
        para.scale = 0.05
        para.bias = np.array([0.05, -0.01, 0.9])
    elif j == 1:
        para.lines_type = 0

        para.coppeliasim = True

        para.durations = dur
        para.intervalnum = 15
        para.linear_interpolation = False

        para.plane = 'yz'
        if para.plane == 'xy':
            para.constraint_manual = False
        else:
            para.constraint_manual = True


        para.constraint_plane = True
        para.constraint_line = False
        para.constraint_vect = [1.0, 0.0, 0.0]
        para.constraint_point = [-0.05, 0, 0]
        para.constraint_dis = [0.05, 0.025]

        para.torch_vertical = True

        para.scale = 0.1
        para.bias = np.array([0.05, 0.08, 0.89])
        para.pointrevert = False
        para.pointmirror = False
        para.pointrearrange = False


        #
    # para_object = FunParameter.parameter_object(para)
    update_par()





print('*'*20 + ' Parameter setup finished ' + '*'*20)


'''
        NOTATION
        in installation, the shift from the ur3e origin to the rotation base center is about 62.5 cm and 11 cm.

'''