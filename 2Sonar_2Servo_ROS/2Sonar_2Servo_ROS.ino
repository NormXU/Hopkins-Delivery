
#define SERIAL_CLASS Serial_
#include <ros.h>
#include<std_msgs/Int16.h>
#include <Servo.h>
// Pin Numbers
//const int lockerPin1 = 12;
//const int lockerPin2 = 13;
Servo myServo;
//       
#define echoPin 7
#define trigPin 8
#define echoPin1 4
#define trigPin1 2
// variables will change:
int lockerState = 0;         
int incomingByte;
int object_distance;
int object_distance1;
int motor_angle;
long duration, distance;
long duration1, distance1;

ros::NodeHandle  nh;

std_msgs::Int16 str_msg1;
ros::Publisher sonar1("sonar1", &str_msg1);



void setup()
{
  nh.initNode();
  nh.advertise(sonar1);
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT); // Trigger PIN
  pinMode(echoPin, INPUT); // Echo PIN
  pinMode(trigPin1, OUTPUT); // Trigger PIN
  pinMode(echoPin1, INPUT); // Echo PIN
  myServo.attach(12);
}

void loop()
{
  Servo_with_sonar();
   object_distance=calculateDistance();
   object_distance1=calculateDistance1();
  str_msg1.data = object_distance,object_distance1;
  sonar1.publish( &str_msg1 );
  nh.spinOnce();
  delay(1000);
}

// zi function
long microsecondsToCentimeters (long microseconds) {
 // The speed of sound is 340 m/s or 29 microseconds per centimeter
 // The ping travels forth and back, so, the distance is half the distance traveled
 return microseconds / 29 / 2;
}

 //****** Sonar*************
int calculateDistance(){
      //control sonars
         digitalWrite (trigPin, LOW);
         delayMicroseconds(2);
         digitalWrite(trigPin, HIGH);
         delayMicroseconds(10);
         digitalWrite (trigPin, LOW);
 //read the bounced wave
        duration = pulseIn(echoPin, HIGH);
 //calculate the distance
       distance = microsecondsToCentimeters(duration);
       delay(10);
       return distance;
}
int calculateDistance1(){
      //control sonars1
         digitalWrite (trigPin1, LOW);
         delayMicroseconds(2);
         digitalWrite(trigPin1, HIGH);
         delayMicroseconds(10);
         digitalWrite (trigPin1, LOW);
 //read the bounced wave
        duration1 = pulseIn(echoPin1, HIGH);
 //calculate the distance
       distance1 = microsecondsToCentimeters(duration1);
       delay(10);
       return distance1;
}
//*****Servo motor with sonar
void Servo_with_sonar(){
  for(int i=15;i<=165;i=i+10){  
      myServo.write(i);
      delay(30);
   }
   for(int i=165;i>15;i=i-10){  
      myServo.write(i);
      delay(30);
   } 
}
//rosrun rosserial_python serial_node.py _port:=/dev/ttyACM3 _baud:=9600
