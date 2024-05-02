"""gps_test controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from vehicle import Driver

# create the Robot instance.
driver = Driver()
gps = driver.getDevice("gps")# get the time step of the current world.
lidar = driver.getDevice("lidar")
timestep = 10
gps.enable(timestep)
lidar.enable(timestep)  ##  time step corresponds to wait time in mili seconds

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
def process(data):
 
    pose = []
    count = 0
    dist =0
    lent = 0
    i = 0
    #print(len(data))
    
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
# - perform simulation steps until Webots is stopping the controller
while driver.step() != -1:
    gps_car = gps.getValues()
    print(gps_car)
  
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
 
# Enter here exit cleanup code.
