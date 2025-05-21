# earth-rover
Affordable and Sustainable Mobility Autonomy for  4D Environmental Monitoring

Jnaneshwar Das, ASU School of Earth and Space Exploration 


## Abstract: 
For continuous ecological monitoring of urban or suburban environments, it is necessary to have affordable and versatile mobility autonomy. In this setting, we may wish to carry out 3D metric-semantic-topological mapping of trees, infrastructure, rocks, or other features of interest periodically, in intervals of hours to weeks. Autonomous Ground Vehicles (AGVs) from companies such as ClearPath Robotics cost about USD 25000, and need to be carried around for deployment, posing both cost and logistical challenges. 
![image](https://github.com/user-attachments/assets/07d53db6-031a-4a68-9338-75e33c5428a5)


This prorotype is a 26” Schwinn adult tricycle frame, augmented with a front electric motor for traction, a steering system, rear aerodynamic solar panel assembly,  a rear mast for avionics, sensing, and compute payloads. The vehicle’s large (26”) wheels and low weight (65 kg), enables stability and long range (up to 50 km a day with solar), and is capable of GPS-enabled vision based autonomous navigation on paved or unpaved paths. The vehicle can execute repeatable paths that are either learned by the system from experimental drives by a rider, or through specified or optimized mission plans. At a base cost of about USD 3000, our system is affordable, and can be assembled from either COTS components, or from custom hardware. Additionally, our system can be operated manually like a standard electric tricycle, providing additional modalities for expert data collection and imitation learning. 

## System Description 
Mass
65 kg with instrumentation and solar panels (no human)
Range
Tested 20 km (manual operation, without solar), 
Estimated 30 km with solar over whole day manual operation
Estimated 60+ km for autonomous operation with solar 
Power stores
Lithium Iron-Phosphate (LiFePO4) 25Ah, 57.6 V (traction)
LiFePO4 100Ah, 14.4 V (avionics, compute, sensing, autonomous traction and steering)
Power conversion
1000W inverter for 14.4V DC to 110V AC 
30A MPPT charge controller for 200W solar charging  
2 x 300W 110V AC to 12V DC power converters for computing and autonomous control  110V to 58V DC for traction battery charging  110V to 14.4V DC for avionics/compute battery charging  
Traction
Front 1000W (1.3 horsepower) direct-drive 3-phase electric motor 
1000W controller for manual ride controlled by throttle
300W motor controller for autonomous drives, controlled by Pixhawk flight controller over PWM channel 
Energy sequestration
2 x 100W ultralight panels, mounted on rear boom for aerodynamic shape
Regenerative braking 
Compute
11th Gen Intel(R) Core(TM) i7-11800H @ 2.30GHz, 32GB RAM, 512GB internal SSD, 2TB external SSD

Google Coral Tensor Processing Unit (TPU) 
Avionics
Pixhawk 2.1 Flight Controller, PX4 open source autopilot software stack 
Here + GPS with RTK option 
Sensor suite 
PointGrey Grasshopper3 with narrow angle (left), and wide angle (right) lenses, 
MicaSense Altum 6 band multi-spectral camera triggered by flight controller, 100m 
LiDARLite LASER ranger, 
OceanOptics FLAME UV-VIS-NIR spectrometer 
Intel RealSense T265 tracking camera  
Communication
Ubiquiti networks 2.4GHz AirMax PicoStation access point
WiFi hotspot on onboard compute 
915MhZ telemetry radio to Pixhawk 
2.4GHz DSMX RC transmitter to Pixhawk 

![image](https://github.com/user-attachments/assets/836d27e1-4022-4540-929a-9dad2fe70606)
![image](https://github.com/user-attachments/assets/10b7eb70-dca8-4777-9c3e-f0e3dd4d7faf)

## Mobility system:  
The vehicle has a 1 kW front brushless 3 phase electric motor that operates with a 48V Lithium Iron Phosphate (LiFePO4) battery. The trike can be steered manually or through a slip-drive clutch actuator system. 
The front traction motor is operated at a lower power of 300W with a different controller, for safety. This controller is commanded with a potentiometer-servo combination, providing isolation and an additional level of control since the potentiometer can be rotated like a throttle, by a test rider. 

When controlled manually, a throttle with a hall-effect sensor enables commandeering of the vehicle up to a speed of 12 m/s. Electronic braking serves as an additional source of charging. 

When steered by the Pixhawk flight controller, QGroundControl ground control station (GCS) software is used, and the traction system is switched to a 300W controller for safety, providing lower power and speeds. 

Payload mast: A load bearing mount-point at a height of 120cm from ground, is built using a spring assembly consisting of low-cost COTS carbon fiber and aluminum arrow bodies, and the trike’s metal rear basket, that yields to motions in both body frame x, y, and z axes. This assembly helps decouple the mounted imaging and compute system from bumps and jerks that could interfere with data collection, or in the worst case scenario, can damage aspects of the imaging suite assembly. With the mounting assembly, the trike is able to collect data while traversing at speeds up to 12 m/s. 


## Autonomy:  
The electric tricycle’s avionics package consists of a Pixhawk 2.1 flight controller running a rover airframe on the PX4 autopilot software stack. The vehicle is capable of GPS and IMU based waypoint missions, as a base feature, with additional computer vision capabilities through simultaneous localization and mapping 

Figure 2: QGroundControl GCS used for mission planning and situational awareness during a field trial at ASU campus (Dec 2023). 
![image](https://github.com/user-attachments/assets/8c02a1dc-cda6-431c-b298-c8562db48d53)

(SLAM), ORBSLAM3, ROS, PX4, ROS, Gazebo, PX4 SITL digital twin, 

The system’s onboard computer runs ROS2, with ros nodes for all the cameras, spectrometer, lidar ranger, and MAVROS package for communicating with the Pixhawk for telemetry, and commanding the vehicle. 

QGC allows GPS mission planning, with all operations possible without internet connectivity, on cached maps

## Mapping systems:
![image](https://github.com/user-attachments/assets/347fb552-1ba4-4496-9c28-1300db70184c)


Figure 3: Realtime mapping demo, while the vehicle is manually operated by a human. The system can leverage the 3D maps and localized vehicle path to plan unmanned operations to remap routes, for instance for biomass change estimation 

Applications and Research Areas  	
Environmental Monitoring in urban and suburban settings, including biomass mapping and heat/shade modeling. 	
With its onboard mapping suite consisting of global shutter multi-focal stereo cameras, a multi-spectral camera, and a UV-VIS-NIR spectrometer, EarthRover is able to collect rich data for 3D environmental analysis. The dense datasets collected by EarthRover (> 1GBps) presents avenues for further research for optimal data fusion for multi-scale, multi-modal hyperspectral datasets, for 4D environmental change monitoring. 
Geological and Ecological Mapping for Digital Twins





Point cloud uncertainty (blue=low)

Figure 4: Mapping of a rocky feature set by the vehicle, by orbiting and collecting imagery with its PointGrey Grasshopper3 camera with narrow angle lens.  

## Natural Language Interaction and lifelong learning
We anticipate interaction with a human user through command line interface, verbal interaction, and gestures. The command set may include navigational instructions such as “Follow me” which is integrated with the trike autonomy stack for visual tracking of a human leader. For mission planning however, the bulk of the commanding architecture may be natural language with generative AI in the backbone for tokenizing the instructions and generating optimal exploration plans.  

Example natural language prompts: 
Map all trees 
Count fruits along the path I take
Map as much as you can in the next 15 mins, stay within 50m of me. 
Collect imagery at this scene to improve existing maps 
Estimate plant biomass of this patch
What is the rock trait distribution along this pavement? 
Orbit those rocks I am pointing at, with a 10m radius, and show me the 3D model of the rocks.  
 
## Videos: 

https://drive.google.com/file/d/1M8JIJIpk5DaTXnk8shY55RajGW9rd4tN/view?usp=sharing



https://drive.google.com/file/d/1zt0DhuATs35bWYduTn2sGGhrnV1hIs_N/view?usp=sharing

https://drive.google.com/file/d/1mKV-D2FhJC4ZnpSakJQ_sJLD6O1SGgua/view?usp=sharing



https://www.youtube.com/watch?v=l2MmlcPx6kE



https://www.youtube.com/watch?v=NZj4yiCzRHI



Monitoring normalized difference vegetation index (NDVI) with side-mounted imaging suite with OceanOptics VIS-NIR spectrometer. Imagery from narrow angle and wide angle Grasshopper3 cameras also shown. 
https://youtu.be/PZGcjdSuags?si=71zC8FyeZaOeyud6

Data capture with the vehicle with a Prophesee metavision event camera (Alphacore) in the imaging suite, providing high dynamic range. 
https://www.youtube.com/watch?v=2V3Mc3UAJss

