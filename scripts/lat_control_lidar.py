import Lateral_controller.scripts.OSMnav as OSMnav
from Lateral_controller.scripts.OSMnav import llc
import Stanley
from controller import Robot, Keyboard
from controller import InertialUnit
from vehicle import Driver
import matplotlib.pyplot as plt
import numpy as np

TIMESTEP = 10
l = []
action = []
coords = []

driver = Driver()
AutoDrive = OSMnav.OSMnav()

Yaw = driver.getDevice("inertial unit")
gps = driver.getDevice("gps")
lidar = driver.getDevice("lidar")

Yaw.enable(TIMESTEP)
gps.enable(TIMESTEP)
lidar.enable(TIMESTEP) 

driver.setGear(1)
step = 0
k = 1


def process(data): # processing lidar data to get position and angle
    pose = []
    count = 0
    dist =0
    lent = 0
    i = 0
    while i < len(data):
        if data[i] != float('inf'):
            while data[i] != float('inf') and i < len(data) -1 :
                lent +=1
                count += i
                dist += data[i]
                i +=1
        if lent>0:
            pose.append([dist/lent,count/(2048*lent)*3.14/0.22 - 0.1])
        count = 0
        lent = 0
        i += 1
    return pose


while driver.step()!=-1:
   curr_pos = gps.getValues()
   laser_data = lidar.getRangeImage()

   if step == 120:
        AutoDrive.ResetCoords(curr_pos[0], curr_pos[1],12.988420, 80.228086)
        coordinates_LL = AutoDrive.ShortestPath()
        coordinates_LL[:,0], coordinates_LL[:,1] = coordinates_LL[:,0] + 0.000033, coordinates_LL[:,1] + 0.000053
        coordinates_cartesian = np.asarray(AutoDrive.getCartesian(coordinates_LL))
        coordinates_cartesian = np.insert(coordinates_cartesian, 0, np.asarray(llc(curr_pos)), axis=0)
        coordinates_cartesian[:,0] = coordinates_cartesian[:,0] - 2
        coordinates_cartesian[:,1] = coordinates_cartesian[:,1] + 0.5
        coordinates_bezier = np.asarray(AutoDrive.bezier_curve(coordinates_cartesian))
        coordinates_cartesian = np.delete(coordinates_cartesian,(0),axis =0)   
        stanley = Stanley.LatControl(coordinates_cartesian)
        AutoDrive.plotDubins(coordinates_cartesian,curr_pos)
        prev_stter = 0
        k = 0
        obs_old = 0
      
   elif step >120 : 
        curr_pos = llc(curr_pos)
        driver.setCruisingSpeed(30)
        speed = driver.getCurrentSpeed()*5/18
        steering = driver.getSteeringAngle()
        obstacle_heading, obstacle_dist, num_obs = process(laser_data)
        
        if obs_old != num_obs and num_obs>0:
            stanley.resetObs()
            sine_pose = curr_pos
            stanley.setObs([obstacle_heading, obstacle_dist])

        if num_obs > 0:
            steer_angle = -stanley.sine(sine_pose[0:2],yaw,speed)

        else:
            steer_angle, k = stanley.steer_control(curr_pos,yaw,speed)

        obs_old = num_obs
        curr_coord = np.asarray([curr_pos[0],curr_pos[1]])
        driver.setSteeringAngle(steer_angle)

   yaw = Yaw.getRollPitchYaw()
   yaw = yaw[2] 
   step += 1  

   





