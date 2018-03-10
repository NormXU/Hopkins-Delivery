/*
    * PW is digital pin 8
    * PW1 is digital pin 7
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
Maxbotix rangeSensorPW_s(7, Maxbotix::PW, Maxbotix::LV);
Maxbotix rangeSensorAD(A0, Maxbotix::AN, Maxbotix::LV);
       

// variables will change:
int lockerState = 0;         
float object_distance;
//int object_distance1;

long duration, distance;
long duration1, distance1;

ros::NodeHandle  nh;

//std_msgs::Float32 str_msg1;
//ros::Publisher sonar("sonar", &str_msg1);
sensor_msgs::Range range_msg;
ros::Publisher pub_range( "/ultrasound", &range_msg);
char frameid[] = "/ultrasound1";
char frameid1[] = "/ultrasound2";
void setup()
{
  nh.initNode();
//  nh.advertise(sonar);
  nh.advertise(pub_range);
  Serial.begin(9600);
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;

  range_msg.field_of_view = 0.1;  // fake
  range_msg.min_range = 17;
  range_msg.max_range = 300;
}

void loop()
{

    range_msg.range = rangeSensorPW.getRange();
    range_msg.header.frame_id =  frameid;
    range_msg.header.stamp = nh.now();
    pub_range.publish(&range_msg);
    //
    range_msg.header.frame_id =  frameid1;
    range_msg.range = rangeSensorPW_s.getRange();
    range_msg.header.stamp = nh.now();
    pub_range.publish(&range_msg);
  nh.spinOnce();
  delay(100);
}


