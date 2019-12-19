# auto_navs(Refer to final branch for the changes master is not updated)


Final Two Motor Code is for controlling Powerbot and please check the pins before running the code


arduino.py is used for communicating ROS with Arduino with less latency


To replicate the results of Autonomous Navigation on powerbot:

1. Run powerbot.launch and in arduino.py check the parameters for baudRate and the port on which arduino is connected to ideally 
   it would be /dev/ttyACM0 or /dev/ttyACM1
   
2. Launch move_base.launch.xml, change the parameters in map_server and locate your .pgm file in args.

3. http://wiki.ros.org/navigation/Tutorials/Using%20rviz%20with%20the%20Navigation%20Stack, go through this video on this link will show 
   step by step instructions for implemeting navigation stack with help of RVIZ
  
  
To replicate results of autonomous navigation on any robot:(like p3dx)

First you need to change the parameters of foorprint in costmap_commmon_params.yaml. Footprint basically is bounding box of the robot as
from the top view

For robot like p3dx you dont need to run arduino.py(i.e no need for step 1 as above) is there is inbuilt controller inside it and publishing the topics just remap the appropriatte topics 
of move_base.launch.xml


