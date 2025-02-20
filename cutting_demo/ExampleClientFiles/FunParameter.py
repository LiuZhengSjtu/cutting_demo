
import numpy as np

class parameter_class:
    def __init__(self):
        self.lines_type = 0
        self.coppeliasim = True

        self.durations = 0.5

        self.constraint_manual = True

        self.constraint_plane = True
        self.constraint_line = False
        self.constraint_sphere = False

        self.constraint_vect = [1.0, 0.0, 0.0]
        self.constraint_point = [-0.05, 0, 0]
        self.constraint_dis = [0.075, 0.025]

        self.torch_vertical = True
        self.scale = 0.05
        self.bias = np.array([0.05, -0.05, 0.9])



        self.intervalnum = 15
        self.linear_interpolation = False


        self.pointrevert = True
        self.pointmirror = True
        self.pointrearrange = True

        self.colorturn = True

        self.plane = 'yz'





class letterclass:
    def __init__(self):
        #   size of letters
        #   the rectangle for the letter
        self.H_letterspace = 1
        self.W_letterspace = 0.8
        #   height and width of the CAPITCAL letter
        self.hc = 1
        self.wc = 0.5
        #   height ad width of the subscript letter
        self.hs = 0.5
        self.ws = 0.5
        self.density = 1.0      #   1.0 for height = 1






class parameter_plan:
    def __init__(self, home_position_n = 2, durations = None , distance = [0.35, 0.3] , para = parameter_class()):
        self.home_position_n = home_position_n
        self.durations = durations
        if para.constraint_manual:
            self.distance = para.constraint_dis
        else:
            self.distance = distance


class planclass:
    def __init__(self, cylinder_cube = 2, getobjectfromcoppeliasim = True, para = parameter_class()):
        #   num specify the target, 0 for cylinder, 1 for cube (on the floor), 2 for the curved surface of cylinder (on the floor)
        #   if the getobjectfromcoppeliasim if false, all reading/writing through RemoteAPIClient are unaccessible.
        self.coppeliasim = getobjectfromcoppeliasim
        self.cylinder_cube = cylinder_cube
        if self.cylinder_cube == 0:
            if para.torch_vertical:
                k = 3
            else:
                k = 0
            self.object = parameter_plan(home_position_n = k, durations =para.durations, distance= [0.325, 0.3], para = para)
        elif self.cylinder_cube == 1:
            self.object = parameter_plan(home_position_n = 1, durations=para.durations, distance=[0.2, 0.1] , para = para)
        elif self.cylinder_cube == 2:
            self.object = parameter_plan(home_position_n=1, durations=para.durations, distance = [0.475, 0.425] , para = para)
        elif self.cylinder_cube == 3:
            self.object = parameter_plan(home_position_n=1, durations=para.durations, distance=[0.275, 0.25],
                                         para=para)

def parameter_object(para):
    return planclass(cylinder_cube = para.lines_type, getobjectfromcoppeliasim = para.coppeliasim, para = para)


def get_para(para):
    para_object = parameter_object(para)
    letter_size = letterclass()

    return  para_object, letter_size