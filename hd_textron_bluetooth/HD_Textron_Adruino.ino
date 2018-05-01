
/*
    * PW is digital pin 8
    * PW1 is digital pin 7
    * TX is digital pin 6
    * AN is analog pin A0
    * Electromagnetic pin 13
    * rxPin 2
    * txPin 3
 */
// #define SERIAL_CLASS Serial_
#include <ros.h>
#include <sensor_msgs/Range.h>
#include "Maxbotix.h"
#include <std_msgs/String.h>
#include<std_msgs/Int16.h>
#include <SoftwareSerial.h>
#include "bluetooth.h"

// variables will change:
ros::NodeHandle  nh;

sensor_msgs::Range range_msg;
ros::Publisher pub_range( "/ultrasound", &range_msg);


// Bluetooth communication
Bluetooth *blue = new Bluetooth("DroneBox");

std_msgs::String drone_box_signal;
ros::Publisher pub_signal( "/bluetooth", &drone_box_signal);

// For TX2 descending program

std_msgs::Int16 box_signal;
ros::Publisher pub_box_signal( "/TxBox", &box_signal);


//***** call back function****
void BluetoothCb(const std_msgs::Int16& msg){
  if(msg.data==1){
            box_signal.data = 1;
            pub_box_signal.publish(&box_signal);
  }
  else{
           box_signal.data = 0;
           pub_box_signal.publish(&box_signal);
  }
}



void messageCb(const std_msgs::Int16& msg){
  if(msg.data==1){
            digitalWrite(13,HIGH);
  }
  else{
           digitalWrite(13,LOW);
  }
}



void setup()
{
  ros::Subscriber<std_msgs::Int16> subTextron("TXdrone", &BluetoothCb);
  ros::Subscriber<std_msgs::Int16> sub("electromagnetic", &messageCb);
  nh.initNode();
  nh.advertise(pub_range);
  nh.advertise(pub_signal); // For Bluetooth
  nh.advertise(pub_box_signal); // For Bluetooth
  nh.subscribe(sub);
  nh.subscribe(subTextron);
  
  Serial.begin(9600);
  blue->setupBluetooth();
    
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  pinMode(13,OUTPUT);
  range_msg.field_of_view = 0.1;  
  range_msg.min_range = 17;
  range_msg.max_range = 300;

}

void loop()
{
    Maxbotix rangeSensorPW(8, Maxbotix::PW, Maxbotix::LV);
    Maxbotix rangeSensorPW_s(7, Maxbotix::PW, Maxbotix::LV);
    Maxbotix rangeSensorAD(A0, Maxbotix::AN, Maxbotix::LV);
    char frameid[] = "/ultrasound1";
    char frameid1[] = "/ultrasound2";
    String msg = blue->Read();
    
    // Bluetooth Node
    if(msg.length() > 1){
    Serial.print("Received: ");
    Serial.println(msg);
    drone_box_signal.data = "Find Box! Descending \n ";
    pub_signal.publish(&drone_box_signal);
    }
    else{
    drone_box_signal.data = "Drone is arriving, Plese open the box \n ";
    pub_signal.publish(&drone_box_signal);
    }
    
    // Sonar Node
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

