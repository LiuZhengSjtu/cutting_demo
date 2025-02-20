
'''
for the network and communication with CoppeliaSim and Step motor(Arduino)
'''

from multiprocessing import Process
import socket
import psutil
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from dqrobotics import *
import numpy as np
import math
from numpy.ma.core import shape
import time
import serial
from sas_core import Clock, Statistics

import FunDq


def get_ipv4_address():
    # Get network interfaces (addresses)
    addrs = psutil.net_if_addrs()

    ips = []
    # Iterate through network interfaces and get IPv4 address
    for interface, interface_addresses in addrs.items():
        for addr in interface_addresses:
            if addr.family == socket.AF_INET:  # Use socket.AF_INET for IPv4
                # return addr.address
                ips.append(addr.address)
                print(f'ip address: {addr.address}')

    for ip in ips:
        if ip[0:10] == '192.170.10':
            return '127.0.0.1'
    for ip in ips:
        if ip[0:10] == '192.168.1.':
            return '192.168.1.35'
    for ip in ips:
        if ip[0:10] == '192.168.0.':
            return '192.168.0.167'
    for ip in ips:
        if ip[0:10] == '192.168.50':
            return '192.168.50.43'
    print(f'unknown ip addres: {ips}')


#   config the drawing lines for the way points
class drawclass():
    def __init__(self, coppeliasim):
        self.coppeliasim = coppeliasim
        if self.coppeliasim:
            print('connecting to host Ip through RemoteAPIClient')


            client = RemoteAPIClient(get_ipv4_address())
            client.timeout = 5
            self.sim = client.getObject('sim')

            self.dr=self.sim.addDrawingObject(self.sim.drawing_lines,3,0,-1,5000,[1,0,0])
            self.dg = self.sim.addDrawingObject(self.sim.drawing_lines, 3, 0, -1, 5000, [0, 1, 0])
            self.d_circle = self.sim.addDrawingObject(self.sim.drawing_spherepoints, 0.01, 0, -1, 9999, [1,0,0])

            # poseur = self.sim.getObjectPose(  self.sim.getObject('/ForceSensor') )
            # print(f'the pose of the ur3 from the coppeliasim is {Parameter.coppelisimPose2dq(poseur)}')
        else:
            print('The coppeliasim is not used in real robot implementation')

    def drawlines(self, l=[0,0,0,0,0,0], c = 1):

        if self.coppeliasim:
            if  c == 1:
                color = self.dr
            elif c == 2:
                color = self.dg

            if len(shape(l)) == 2:
                for line in l:
                    self.sim.addDrawingObjectItem(color, line)
            elif len(shape(l)) == 1:
                self.sim.addDrawingObjectItem(color, l)
            else:
                raise ValueError(f'Points list format error. len(shape(lines)) = {len(shape(l))}')

    def drawlinesclear(self, c=1):
        if self.coppeliasim:
            if c == 1:
                color = self.dr
                self.sim.addDrawingObjectItem(color, [])
            elif c == 2:
                color = self.dg
                self.sim.addDrawingObjectItem(color, [])
            else:
                self.sim.addDrawingObjectItem(self.dr, None)
                self.sim.addDrawingObjectItem(self.dg, None)

    def colorturn(self, cur_pose, laserline, color_turns, colorturn):

        if colorturn:
            if self.coppeliasim:
                for j in range(color_turns.ColorTurn_n, min(len(color_turns.poses_reached), color_turns.ColorTurn_n + 3), 1):
                    if color_turns.poses_reached[j] == 0:
                        dis_l, dis_v = FunDq.get_distance(color_turns.poses_generate[j], cur_pose, laserline)
                        if dis_l < 0.01 and dis_v < 0.03:
                            color_turns.ColorTurn_n = j
                            color_turns.poses_reached[j] = 1
                            self.drawlines(color_turns.points_list_generate[j], c=2)
                            return 1


class move_skip_class():
    '''
    rotate the platform in coppeliasim
    '''
    def __init__(self, drawing, coppeliasim ):
        self.coppeliasim = coppeliasim
        if self.coppeliasim:
            self.sim  = drawing.sim
    def rot(self, name = '/Revolute_joint', delt = 90 ):
        if self.coppeliasim:
            object_hd = self.sim.getObject(name)
            object_position = self.sim.getJointPosition(object_hd)
            t = object_position + delt / 180 * math.pi
            self.sim.setJointTargetPosition(object_hd, t)

            print(f'in coppeliasim, the pos now is {self.sim.getJointPosition(object_hd)}')


            #   wait until target reached
            timo0 = time.time()
            while abs(t - self.sim.getJointPosition(object_hd)) > 0.1:

                if t - self.sim.getJointPosition(object_hd) > 2 * math.pi:
                    t = t - 2 * math.pi
                elif t - self.sim.getJointPosition(object_hd) < -2 * math.pi:
                    t = t + 2 * math.pi

                time.sleep(0.2)
                if time.time() - timo0 > 20:
                    print('coppeliasim skip rotation, overtime break')
                    break
                # print(f't = {t} and current pos = {self.sim.getJointPosition(object_hd)}')
            print(f'in coppeliasim, after rotation, position is {object_position + delt / 180 * math.pi}')

            return True
        else:
            return True







class object_pose:
    def __init__(self):
        self.translation = None
        self.vect = None
        self.dq = None
        self.r = None

class object_property(object_pose):
    def __init__(self, name, drawing):
        super().__init__()
        object_hd = drawing.sim.getObject(name)
        pose = drawing.sim.getObjectPose(object_hd)
        self.translation = np.array(pose[0:3])
        quater = [pose[6]]+pose[3:6]
        self.vect = None
        self.dq = None
        self.r = abs(drawing.sim.getObjectFloatParameter(object_hd, drawing.sim.objfloatparam_objbbox_max_x)[1])
        if name[0:5] == '/Cyli':
            self.cylinder(quater)
        elif name[0:5] == '/Cubo':
            self.cube(quater)
        elif name[0:5] == '/Sphe':
            self.sphere(quater)

        print(f'name = {name}\ntranslation = {self.translation}\nvect = {self.vect}\ndq = {self.dq}\npose= {pose}')

    def cylinder(self,quater):
        self.vect = vec3(DQ(quater) * DQ(np.array([0.0, 0.0, 1.0])) * conj(DQ(quater)))
        self.dq = DQ(self.vect) + E_ * DQ(np.cross(self.translation, self.vect))
        print(f'translation = ')

    def cube(self,quater):
        self.vect = vec3(DQ(quater) * DQ(np.array([0.0, 0.0, 1.0])) * conj(DQ(quater)))
        self.dq = DQ(self.vect) + E_ * np.dot(self.translation, self.vect ) * 2     #   the distance from top surface to floor is 2 times of the distance from cube center to surface.

    def sphere(self, quater ):
        # self.vect = vec3(DQ(quater) * DQ(np.array([0.0, 0.0, 1.0])) * conj(DQ(quater)))
        self.dq = DQ(self.translation)

class objectsclass:
    '''
    get the obstacle pose from coppeliasim or manually
    '''
    def __init__(self, drawing, coppeliasim):
        self.coppeliasim = coppeliasim
        if self.coppeliasim:
            self.cylinder_v = object_property(name= '/Cylinder[1]' , drawing=drawing)
            self.cube = object_property(name= '/Cuboid' , drawing=drawing)
            self.cylinder_h = object_property(name= '/Cylinder[2]' , drawing=drawing)
            self.sphere = object_property(name= '/Sphere' , drawing=drawing)
        else:
            print('use the manually defined object geometry and location parameters')
            self.cylinder_v = object_pose()
            self.cylinder_v.translation  =  np.array(  [-0.2,  0.0,   1.1] )
            self.cylinder_v.vect =  np.array( [0.0, 0.0, 1.0] )
            self.cylinder_v.dq = DQ([ 0.0, 0.0, 1.0 ]) + E_ * DQ([ 0.0, 0.2, 0.0 ])
            self.cylinder_v.r = 0.1

            self.cylinder_h = object_pose()
            self.cylinder_h.translation = np.array( [ 0.0, 0.0, 0.325 ] )
            self.cylinder_h.vect =  np.array( [0.0, -1.0, 0.0] )
            self.cylinder_h.dq = DQ([0.0, -1.0, 0.0]) + E_ * DQ([0.325, 0.0, 0.0])
            self.cylinder_h.r = 0.325

            self.cube = object_pose()
            self.cube.translation = np.array( [-0.1, 0.0, 0.3] )
            self.cube.vect =  np.array( [0.0, 0.0, 1.0] )
            self.cube.dq = DQ([0.0, 0.0, 1.0]) + E_ * DQ([0.6])
            self.cube.r = 0.3

            self.sphere = object_pose()
            self.cube.dq = DQ([ -0.2, 0.1, 0.875 ])
            self.cube.r = 0.225






class Stepmotor():
    def __init__(self):

        self.motor = False
        # Replace '/dev/cu.usbmodemXXXX' with your Arduino's port name
        try:
            self.arduino = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout=1)
            self.motor = True
            print('connected to the ubuntu')
        except:
            print('no motor connected to the ubuntu pc. Try the MAC')

            try:
                self.arduino = serial.Serial(port='/dev/cu.usbmodem1101', baudrate=9600, timeout=1)
                self.motor = True
                print('connected to the MAC')
            except:
                print('no motor is connected the MAC')

        time.sleep(2)  # Wait for the Arduino to initialize

        self.step_angle = 1.8 / 2
        self.gear_reduction = 15.0 / 85.0

    def send_and_receive(self, number = -90):
        if self.motor:
            # print(f"Starting send... at i = {i}")

            step = int(number / self.gear_reduction)

            # Send data to Arduino
            self.arduino.write(f"{step}\n".encode())  # Send the number with a newline character
            time.sleep(0.5)  # Wait for Arduino to process

            # Read response from Arduino
            time0 = time.time()
            while not self.arduino.in_waiting > 0:
                time.sleep(0.5)
                if time.time() - time0 > 20:
                    print(f'Overtime in waiting for receiving data from arduino, delt_time = {time.time() - time0}')
            while self.arduino.in_waiting > 0:
                data = self.arduino.readline().decode().strip()
                print(data + ' --> ' + str(int(data.split(':')[1]) * self.gear_reduction * self.step_angle) + ' from the arduino')  # Print each line received from Arduino
                time.sleep(0.1)

            return  True
        else:
            print('no arduino motor. so no send and receive. just return')
            return True



def statistic_rotate(j, client, move_skip,  stepmotor ):
    print('-' * 20 + ' statistic start' + '-' * 20)
    print(f"Statistics for the entire loop at j = {j}")
    print("  Mean computation time: {}".format(client.clock.get_statistics(
        Statistics.Mean, Clock.TimeType.Computational)
    ))
    print("  Mean idle time: {}".format(client.clock.get_statistics(
        Statistics.Mean, Clock.TimeType.Idle)
    ))
    print("  Mean effective thread sampling time: {}".format(client.clock.get_statistics(
        Statistics.Mean, Clock.TimeType.EffectiveSampling)
    ))
    print('*' * 20 + ' statistic end ' + '*' * 20)



    # rotate the real platform
    # arduino_stepper.send_and_receive(90)
    p2 = Process(target = stepmotor.send_and_receive)
    p2.start()
    #

    # meahwhile, rotate the platform in coppeliasim
    move_skip.rot('/Revolute_joint', 90)

    #   wait, until stepmotor done
    p2.join()






def get_interface(coppeliasim):
    drawing = drawclass(coppeliasim=coppeliasim)
    objects = objectsclass(drawing, coppeliasim)
    move_skip = move_skip_class(drawing, coppeliasim)
    #   for desired line color turn
    # color_turns = parameter_turn_color()
    stepmotor = Stepmotor()

    return drawing, objects, move_skip, stepmotor

