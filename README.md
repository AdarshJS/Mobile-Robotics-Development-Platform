# Mobile-Robotics-Development-Platform
The low cost robot that mimics the capabilities of the TurtleBot. MRDP has a Microsoft Kinect, Arduino Uno, rotary encoders, and a 9 DOF IMU. The user's laptop or netbook running the ROS nodes in this repository in placed on the MRDP to execute SLAM. The Kinect and the Arduino are coapnnected to the laptop. Freenect package is used to interface the Kinect with ROS (OpenNI may also be used). Arduino Un board establiches communication with the laptop through ROSserial Arduino package. Odometry folder contains programs that can be run on the Arduino connected to a rotary encoder.

The mrdp package contains ROS and OpenCV programs for navigation and localization. Please explore Gmapping, and amcl ROS packages first.

I have included an exhaustive list of all the packages one might need for SLAM using the TurtleBot.

NOTE: This repository is only for the reference of the members of RMI NIT-Trichy. Since the project uses custom hardware, it is not meant to be replicated. 
