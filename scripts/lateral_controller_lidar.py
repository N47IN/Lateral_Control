import OSMnav
from OSMnav import llc
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
camera = driver.getDevice("camera")
Yaw = driver.getDevice("inertial unit")
gps = driver.getDevice("gps")
#lidar = driver.getDevice("lidar")
radar = driver.getDevice("radar")
Yaw.enable(TIMESTEP)
gps.enable(TIMESTEP)
#lidar.enable(TIMESTEP)
radar.enable(50)
camera.enable(TIMESTEP)


driver.setGear(1)
step = 0
k = 1


""" def process(data): # processing lidar data to get position and angle, returns [distance,angle]
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
            pose.append([dist/lent,count/(2048*lent)*3.14/0.22 - 1.5])
            print(count/(2048*lent)*3.14/0.22 - 1.5)
        count = 0
        lent = 0
        i += 1
    return pose[1:], len(pose[1:]) """
def radar_filter(sep):                      
    radar_filter.previousDist.append(sep)
    if len(radar_filter.previousDist) > 5:  # keep only 5 values
        radar_filter.previousDist.pop(0)
    return sum(radar_filter.previousDist) / float(len(radar_filter.previousDist))
radar_filter.previousDist = []

while driver.step()!=-1:
   curr_pos = gps.getValues()
   image = camera.getImage()
   
   #laser_data = lidar.getRangeImage()

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
        done = True
      
   elif step >120 : 
        curr_pos = llc(curr_pos)
        num_obs = radar.getNumberOfTargets()
        #num_obs = radar_filter(num_obs)
        targets = radar.getTargets()
        
        speed = driver.getCurrentSpeed()*5/18
        steering = driver.getSteeringAngle()
        detach = False
        #print("detecting",num_obs - 1," obstacles")
        

        if num_obs > 1:
            for i in range(0,num_obs):         # 0th target is the grounf
                distance = targets[i].distance
                azimuth = targets[i].azimuth
                print("SINE")
                print(" Distance :",distance)
                print(" Azimuth  :", azimuth)
                print(" Obstacles :", num_obs - 1)
        
        if obs_old != num_obs and num_obs>1:
            stanley.resetObs()
            
        if num_obs > 1 :
            pose = []
            distances = []
            for i in range(1,num_obs):         # 0th target is the grounf
                distance = targets[i].distance
                azimuth = targets[i].azimuth
                distances.append(distance)
                pose.append([distance,azimuth])
            #stanley.setObs(pose)
            speed = driver.getCurrentSpeed()*5/18
            steer_angle = stanley.steer_control(curr_pos,yaw,speed) + stanley.sine(speed,pose)
            
            for d in distances:
                    if d < 6:
                        brake = 1/distance
                        driver.setBrakeIntensity(brake)
                        print("braking with",brake)
                    if d< 1.5:
                        driver.setBrakeIntensity(1)
            
        else :
            print("STANLEY")
            steer_angle = stanley.steer_control(curr_pos,yaw,speed)
            print(k)
            #driver.setThrottle(0.3)
        print(" CMD_STEER :",steer_angle)
        obs_old = num_obs
        curr_coord = np.asarray([curr_pos[0],curr_pos[1]])
        driver.setSteeringAngle(steer_angle)
        driver.setBrakeIntensity(0)
        driver.setThrottle(0.3)


   yaw = Yaw.getRollPitchYaw()
   yaw = yaw[2] 
   step += 1  

   





