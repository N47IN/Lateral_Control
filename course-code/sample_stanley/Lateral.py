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

   if step == 120:

      AutoDrive.ResetCoords(curr_pos[0], curr_pos[1],12.988420, 80.228086)
      coordinates_LL = AutoDrive.ShortestPath()
      coordinates_cartesian = AutoDrive.getCartesian(coordinates_LL)
      coordinates_cartesian = np.insert(coordinates_cartesian, 0, np.asarray(llc(curr_pos)), axis=0)
      coordinates_cartesian[:,0] = coordinates_cartesian[:,0] - 2
      coordinates_cartesian[:,1] = coordinates_cartesian[:,1] + 0.5
      coordinates_bezier = np.asarray(AutoDrive.bezier_curve(coordinates_cartesian))
      coordinates_cartesian = np.delete(coordinates_cartesian, (0),axis =0)   
      stanley = Stanley.LatControl(coordinates_cartesian)
      prev_stter = 0
      k = 0
      

   elif step >120 : 
      curr_pos = llc(curr_pos)
      driver.setCruisingSpeed(30)
      speed = driver.getCurrentSpeed()*5/18
      steering = driver.getSteeringAngle()
      curr_coord = np.asarray([curr_pos[0],curr_pos[1]])
      obs_flag = False
      steer_angle = stanley.steer_control(curr_pos,yaw,speed)
      driver.setSteeringAngle(steer_angle)
      print("STANLEYYY")
   yaw = Yaw.getRollPitchYaw()
   yaw = yaw[2] 
   step += 1  

   