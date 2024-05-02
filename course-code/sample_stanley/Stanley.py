import numpy as np
import math
from scipy.optimize import minimize, Bounds
#from RRT_dubins import RRTDubins
import pandas as pd 



TIMESTEP = 0.01
ROBOT_RADIUS = 0.5
VMAX = 2
VMIN = 0.2

# collision cost parameters
Qc = 0.1
kappa = 4.

# nmpc parameters
HORIZON_LENGTH = int(3)
NMPC_TIMESTEP = 0.05

# stanley params
CROSSTRACK_LIMIT = 4
HEADING_LIMIT = 2.7
CROSSTRACK_GAIN = 2.8


######################################### STANLEY ##################################################
class LatControl():

    def __init__(self,path):
        self.path =  path
        self.i = 1
        self.m = 0
        self.c = 0
        self.k = 1
        self.obs = []
        self.t = 0

    def cross_track_error(self,curr_pos):
        self.m = (self.path[self.i -1,1] - self.path[self.i,1])/(self.path[self.i -1,0] - self.path[self.i,0]) 
        self.c = self.path[self.i-1,1] - self.m*self.path[self.i-1,0]
        e = (curr_pos[1] - self.m*curr_pos[0] - self.c )/ math.sqrt(1+self.m**2)
        return self.k*e

    def heading_error(self,yaw):
        h_vecx = self.path[self.i,0] - self.path[self.i-1,0]
        h_vecy = self.path[self.i,1] - self.path[self.i-1,1]
        k = h_vecy/h_vecx

        if k > 0:
             if h_vecx > 0:
                  #print(1)
                  self.k=1
                  self.path_angle = math.atan(k)
             else:
                  #print(2)
                  self.k =-1
                  self.path_angle = math.pi + math.atan(k)
        else:
             if h_vecx > 0:
                  #print(3)
                  self.k=1
                  self.path_angle = 2*math.pi+math.atan(k)
             else:
                  #print(4)
                  self.k=-1
                  self.path_angle = math.pi + math.atan(k)
        """ Require this small offsets for best performance  """
        self.path_angle += 0.08
        if self.path_angle > 6.1:
            self.path_angle =  2*math.pi - self.path_angle
        yaw = yaw + math.pi
        self.yaw = yaw
        e = self.path_angle - yaw
        #print("path",self.path_angle)
        #print("yaw",yaw)
        """ to avoid unwanted behaviour when path angle is near 0 and yaw 
        of car changes from 0 to 2pi abruptly, required """
        if self.path_angle < 0.3:
            if yaw < 3.14:
                return -e
            else:
                return e/7
        else:
            return -e

    def steer_control(self,curr_pos,yaw,speed):
        K = CROSSTRACK_GAIN
        self.curr_pos = curr_pos[0:2]
        self.yaw = yaw + math.pi
        self.pos_x = curr_pos[0]
        self.pos_y = curr_pos[1]
        self.updatePoints(curr_pos)
        C_err = self.cross_track_error(curr_pos)
        H_err = self.heading_error(yaw)

        #print("chasing point",self.i)
        #print("cross track error",C_err)
        #print("cross track correction",math.atan(K*C_err/speed)/3)
        #print("Heading correction",H_err/3)
        command =   H_err/HEADING_LIMIT + math.atan(K*C_err/speed)/CROSSTRACK_LIMIT
        return command

    def updatePoints(self,curr_pos):
        self.p_desired = self.path[self.i,0:2]
        if np.linalg.norm(self.path[self.i,0:1]-curr_pos[0:1]) < 0.2:
            self.i+=1
        else:
            pass
        return self.i


    
   