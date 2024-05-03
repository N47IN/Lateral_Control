import osm_interface
from osm_interface import llc
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
AutoDrive = osm_interface.OSMnav()

Yaw = driver.getDevice("inertial unit")
gps = driver.getDevice("gps")
radar = driver.getDevice("radar")

Yaw.enable(TIMESTEP)
gps.enable(TIMESTEP)
radar.enable(TIMESTEP)

driver.setGear(1)
iter = 1


while driver.step()!=-1:
    num_obs = radar.getNumberOfTargets()
    targets = radar.getTargets()
    curr_pos = gps.getValues()
    yaw = Yaw.getRollPitchYaw()[2]
    iter += 1  

    if iter == 120:
        AutoDrive.ResetCoords(curr_pos[0], curr_pos[1],12.988420, 80.228086)
        coordinates_LL = AutoDrive.ShortestPath()
        coordinates_cartesian = AutoDrive.getCartesian(coordinates_LL)
        coordinates_cartesian = np.insert(coordinates_cartesian, 0, np.asarray(llc(curr_pos)), axis=0)
        # Dealing with offsets
        coordinates_cartesian[:,0] = coordinates_cartesian[:,0] - 2
        coordinates_cartesian[:,1] = coordinates_cartesian[:,1] + 0.5
        coordinates_cartesian = np.delete(coordinates_cartesian,(0),axis =0)
        print(coordinates_cartesian)   
        #AutoDrive.plotDubins(coordinates_cartesian,curr_pos)






