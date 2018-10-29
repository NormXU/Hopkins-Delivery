# Hopkins Delivery

![ProcessKim](https://github.com/NormXU/Hopkins-Delivery/tree/master/Doc/hopkins_delivers_design_day_2018_poster.jpg)




# Sonar-Based Obstacle Avoidance

## dependent on rosserial library
sudo apt-get install ros-indigo-rosserial-arduino

sudo apt-get install ros-indigo-rosserial

 cd <sketchbook>/libraries
 
  rm -rf ros_lib
  
  rosrun rosserial_arduino make_libraries.py <output path>

## open serial port
sudo chmod 777 /dev/ttyACM0

## set baud Rate = 9600
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=9600

## MaxSonar.ino use Ultra Sonar MB1024. 
There are two ways to calculate the distance. The minimum of the distance is 17 cm

library    git clone https://github.com/Diaoul/arduino-Maxbotix.git

if use Analog pin, use  rangeSensorPW.getRange()

if use PWM pin, use rangeSensorAD.getRange()

## Electro_sonar_pkg
by sending Int16 type message 1; the electromagnetic can work to pick up the piazza

**rostopic pub -1 /electromagnetic std_msgs/Int16 1** // open the electromangetics on both sides

PID_Sonar_DroneCtrl.cpp is for Matrix 100 PID sonar control signal

## Parameter Tuning in client machine

export ROS_IP

hostname -I //to get ip adress

At the bottom of bashrc file, add

export ROS_Master_URI=http://192.168.31.114:11311

export ROS_IP=192.168.31.114

// 192.168.31.114 IP adress

// 11311 端口

rqt_plot /controlPosYaw

# Apriltag Landing
roslaunch usb_cam usb_cam-test.launch   
 
roslaunch apriltags apriltags.launch   

roslaunch dji_sdk sdk.launch         

roslaunch hd_landing position_track.launch

roslaunch hd_landing wtf.launch     
**Run this only when wtf.launch is not included into position_track.launch*

## For Landing Flag
rostopic pub -1 /hd/position_track/position_track_enable std_msgs/Bool true

rostopic pub -1 /hd/position_track/landing_enable std_msgs/Bool true
