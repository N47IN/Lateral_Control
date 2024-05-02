import numpy as np
import math
from scipy.optimize import minimize, Bounds
#from RRT_dubins import RRTDubins
import pandas as pd 
from BicycleModel import KinematicBicycleModel


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
        self.model = KinematicBicycleModel(wheelbase=2.96, max_steer=1.4, delta_time=NMPC_TIMESTEP)

    def cross_track_error(self,curr_pos):
        self.m = (self.path[self.i -1,1] - self.path[self.i,1])/(self.path[self.i -1,0] - self.path[self.i,0]) 
        self.c = self.path[self.i-1,1] - self.m*self.path[self.i-1,0]
        e = (curr_pos[1] - self.m*curr_pos[0] - self.c )/ math.sqrt(1+self.m**2)
        return self.k*e

    def heading_error(self,yaw):
        h_vecx = self.path[self.i,0] - self.path[self.i-1,0]
        h_vecy = self.path[self.i,1] - self.path[self.i-1,1]
        k = h_vecy/h_vecx
        """ getting path angle in osm's frame """

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

################################################## MPC ############################################
  
    def setObs(self,obs):
        self.obs.append(obs)

    def resetObs(self):
        self.t=0
        self.done = False
        self.obs = []

    def mpc_classic(self,curr_pos,yaw,speed):
        robot_state = curr_pos
        self.yaw = yaw 
        self.updatePoints(curr_pos)
        print("chasing point",self.i)
        # predict the obstacles' position in future, stationary for now
        obs_x = curr_pos[0] + math.sin(yaw+self.obs[0][1])*self.obs[0][0]
        obs_y = curr_pos[1] + math.cos(yaw+self.obs[0][1])*self.obs[0][0]
        obstacle_predictions = np.vstack([obs_x,obs_y] * HORIZON_LENGTH)
        theta = self.compute_theta_mpc(
            robot_state, speed, obstacle_predictions)
        return theta/2
    
    def sine(self,speed,pose):
        theta = []
        for obs in pose:
            print(obs)
            dist = obs[0]
            angle = obs[1] - 0.09
            print(angle)
            Amplitude = angle/abs(angle)*speed/(0.5+ 2*abs(angle) + dist)
            #print(math.sin(2*3.14/(3/speed)*self.t))
            theta.append(Amplitude*math.sin(2*3.14/2*self.t))
        if self.t == 3:
            self.done = True
        cmd_theta = sum(theta)/len(theta)
        print(cmd_theta)
        theta = []
        self.t += 0.001
        return -cmd_theta
    

    def get_sine(self,speed,pose):
        theta = []
        for obs in pose:
            print(obs)
            dist = obs[0]
            angle = obs[1] - 0.09
            print(angle)
            Amplitude = angle/abs(angle)*speed/(0.5+ 2*abs(angle) + dist)
            #print(math.sin(2*3.14/(3/speed)*self.t))
            theta.append(Amplitude*math.sin(2*3.14/2*self.t))
        if self.t == 3:
            self.done = True
        cmd_theta = sum(theta)/len(theta)
        print(cmd_theta)
        theta = []
        self.t += 0.001
        return -cmd_theta
    
    def get_all_steer(self,coeffs,curr_pos,yaw,speed,pose):
        command_arr = np.zeros(HORIZON_LENGTH)
        for j in range(HORIZON_LENGTH):
            command = 0
            for i in range(len(coeffs)):
                command += coeffs[i]*self.steer_control(curr_pos,yaw) + self.sine(speed,pose)
            command_arr[j]=command
        return command_arr

    def total_cost(self,coeffs, robot_state, obstacle_predictions):
        speed = 8.33
        yaw = self.yaw
        steering_angle = self.get_all_steer(coeffs)
        robot_future = self.update_state(robot_state, speed, steering_angle, yaw)
        heading_cost = self.head_cost(steering_angle,robot_state)
        collision_cost = total_collision_cost(robot_future, obstacle_predictions)
        #print("track cost",heading_cost)
        return  collision_cost
        
    def update_state(self,robot_state,velocity,steering_angle,yaw ):
        robot_future = np.zeros((HORIZON_LENGTH,2))
        x = robot_state[0]
        y = robot_state[1]
        accel = 0
        for i in range(HORIZON_LENGTH):
            new_x, new_y, new_yaw, _, _, _ = self.model.update(x,y,yaw,velocity,accel,steering_angle[i])
            robot_future[i,:] = [new_x,new_y]
            x, y, yaw = new_x, new_y, new_yaw
        return robot_future
    
    def head_cost(self,theta,pos):
        cost  = 0
        yaw = self.yaw
        for i in range(len(theta)):
            speed = 8.33
            accel = 0
            pos[0], pos[1], new_yaw, _, _, _ = self.model.update(pos[0],pos[1],yaw,speed,accel,theta[i])
            k, i = self.steer_control([pos[0],pos[1]],new_yaw, speed)
            cost += abs(k)
            print("Predicted yaw",yaw)
            yaw = new_yaw
        #print("head cost",np.exp(cost))
        return np.exp(cost)
            
def total_collision_cost(robot, obstacles):
    total_cost = 0
    for i in range(HORIZON_LENGTH):
            rob = robot[i,:]
            obs = obstacles[i,:]
            Qc = 1.3
            total_cost += 1/(0.01*np.exp(Qc*np.linalg.norm(rob-obs)))
    #print("collision cost is", total_cost)
    return total_cost

def collision_cost(x0, x1):
    d = np.linalg.norm(x0 - x1)
    kappa = 10000000
    Qc = 0.5
    cost = kappa/np.exp(Qc*d)
    return cost




    
   