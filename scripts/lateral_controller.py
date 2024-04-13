# fixed stanley for any destination, minor webots offset might be an issue
# tried dubins rrt, almost no success
# MPC grind, state expansion using biycle model, TIme Horizon
# explain costs
# SLSQP issues ( randomization )
# cost functions gradients
# used BYLOS optim
# works for now, have to test with other obstacles
# implemented DWA, kind of bad ( doesnt work when we consider both costs together)
# fourier planner

import OSMnav
from OSMnav import llc
import Stanley
from controller import Robot, Keyboard
from controller import InertialUnit
from vehicle import Driver
import matplotlib.pyplot as plt
import numpy as np

l = []
action = []
coords = []


driver = Driver()
Yaw = driver.getDevice("inertial unit")
gps = driver.getDevice("gps")
driver.setGear(1)
timestep = 10
Yaw.enable(timestep)
gps.enable(timestep)
AutoDrive = OSMnav.OSMnav()
start_pos = gps.getValues()

step = 0
k = 1


while driver.step()!=-1:
   curr_pos = gps.getValues()
   print(curr_pos)

   if step == 120:

      AutoDrive.ResetCoords(curr_pos[0], curr_pos[1],12.988420, 80.228086)
      coordinates_LL = AutoDrive.ShortestPath()
      coordinates_LL[:,0], coordinates_LL[:,1] = coordinates_LL[:,0] + 0.000033, coordinates_LL[:,1] + 0.000053
      coordinates_cartesian = np.asarray(AutoDrive.getCartesian(coordinates_LL))
      coordinates_cartesian = np.insert(coordinates_cartesian, 0, np.asarray(llc(curr_pos)), axis=0)
      coordinates_cartesian[:,0] = coordinates_cartesian[:,0] - 2
      coordinates_cartesian[:,1] = coordinates_cartesian[:,1] - 2
      coordinates_cartesian[:,0] = coordinates_cartesian[:,0] - 2
      coordinates_cartesian[:,1] = coordinates_cartesian[:,1] + 0.5
      coordinates_bezier = np.asarray(AutoDrive.bezier_curve(coordinates_cartesian))
      coordinates_cartesian = np.delete(coordinates_cartesian, (0),axis =0)   
      stanley = Stanley.LatControl(coordinates_cartesian)
      #AutoDrive.plotDubins(coordinates_cartesian,curr_pos)
      prev_stter = 0
      k = 0
      

   elif step >120 : 
      curr_pos = llc(curr_pos)
      driver.setCruisingSpeed(30)
      speed = driver.getCurrentSpeed()*5/18
      steering = driver.getSteeringAngle()
      obs1 = llc([12.991479, 80.2317238])[0:2]
      #obs2 = llc([12.99103425, 80.2316851])[0:2]
      obs = [obs1]
      curr_coord = np.asarray([curr_pos[0],curr_pos[1]])
      obs_flag = False
      #print(f'Distance from obstacle, {np.linalg.norm(curr_coord - obs)}!')
      steer_angle, k = stanley.steer_control(curr_pos,yaw,speed)

      """ for i in obs :
         if obs_flag is False:
            obs_flag = True
            stanley.setObs(i)     
            if np.linalg.norm(curr_coord - i) > 10:
               steer_angle, k = stanley.steer_control(curr_pos,yaw,speed)
               sine_pose = curr_pos
               stanley.resetObs()
               obs_flag = False
            
      if obs_flag is True:
            steer_angle = -stanley.sine(sine_pose[0:2],yaw,speed)
            #aja, k = stanley.steer_control(sine_pose,yaw,speed)
            #steer_angle = 1.1*aja + 1.5*stanley.mpc_classic(curr_pos[0:2],yaw,speed)
            #print("MPC")

      #print(steer_angle) """

      driver.setSteeringAngle(steer_angle)

   yaw = Yaw.getRollPitchYaw()
   yaw = yaw[2] 
   step += 1  

   





