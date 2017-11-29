/*
    * PW is digital pin 8
    * TX is digital pin 6
    * AN is analog pin A0
 */
#define SERIAL_CLASS Serial_
#include <ros.h>
#include <sensor_msgs/Range.h>
#include "Maxbotix.h"
//#include<std_msgs/Float32.h>
#include <Servo.h>
// Pin Numbers
//const int lockerPin1 = 12;
//const int lockerPin2 = 13;
Servo myServo;
//
Maxbotix rangeSensorPW(8, Maxbotix::PW, Maxbotix::LV);

Maxbotix rangeSensorAD(A0, Maxbotix::AN, Maxbotix::LV);
       
//#define echoPin 7
//#define trigPin 8
//#define echoPin1 4
//#define trigPin1 2

// variables will change:
int lockerState = 0;         
float object_distance;
//int object_distance1;
int motor_angle;
long duration, distance;
long duration1, distance1;

ros::NodeHandle  nh;

//std_msgs::Float32 str_msg1;
//ros::Publisher sonar("sonar", &str_msg1);
sensor_msgs::Range range_msg;
ros::Publisher pub_range( "/ultrasound", &range_msg);
char frameid[] = "/ultrasound1";

void setup()
{
  nh.initNode();
//  nh.advertise(sonar);
  nh.advertise(pub_range);
  Serial.begin(9600);
  myServo.attach(12); //Servo trigger PIN
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.1;  // fake
  range_msg.min_range = 17;
  range_msg.max_range = 300;
}

void loop()
{
  Servo_with_sonar();
//   object_distance=rangeSensorPW.getRange();
//   object_distance1=calculateDistance1();
//  str_msg1.data = object_distance;
//  sonar.publish( &str_msg1 );
    range_msg.range = rangeSensorPW.getRange();
    range_msg.header.stamp = nh.now();
    pub_range.publish(&range_msg);

  nh.spinOnce();
  delay(100);
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
