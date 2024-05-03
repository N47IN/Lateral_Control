# Running the scripts
- Set controller argument for the car to " extern "

- Add the following to bashrc :
> export WEBOTS_HOME=/home/username/webots

- Run this in the command line :
> $WEBOTS_HOME/webots-controller --robot-name=vehicle path/to/controller/file [path-to-main.py]

- If your problem persists, follow up on https://cyberbotics.com/doc/guide/running-extern-robot-controllers?tab-os=linux

# World
- All the necesarry devices, such as GPS, IMU and a radar ( just in case) have been mounted on the vehicle. Check with https://cyberbotics.com/doc/guide/samples-devices for additional APIs. 
- All the devices have been initialised in the code, with boilerplate for getting the values already there. The world is derived from the Open Street Map's layout of the institute. 
- If you want to make a world of your own, do check out, https://cyberbotics.com/doc/automobile/openstreetmap-importer

- In case of any queries, email me at : ed21b044@smail.iitm.ac.in
