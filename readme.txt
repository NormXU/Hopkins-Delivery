//Drive 2 sonars and 2 servo motors simultaneously

//dependent on rosserial library
sudo apt-get install ros-indigo-rosserial-arduino
sudo apt-get install ros-indigo-rosserial

 cd <sketchbook>/libraries
  rm -rf ros_lib
  rosrun rosserial_arduino make_libraries.py <output path>

//open serial port
sudo chmod 777 /dev/ttyACM0

// set baud Rate = 9600
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM3 _baud:=9600

// MaxSonar.ino use Ultra Sonar MB1024. There are two ways to calculate the distance. The minimum of the distance is
17 cm
library    git clone https://github.com/Diaoul/arduino-Maxbotix.git

if use Analog pin, use  rangeSensorPW.getRange()
if use PWM pin, use rangeSensorAD.getRange()

// Electro_sonar_pkg
by sending Int16 type message 1; the electromagnetic can work to pick up the 
piazza
