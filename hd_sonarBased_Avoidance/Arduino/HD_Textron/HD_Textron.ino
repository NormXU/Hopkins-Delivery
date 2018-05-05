#include <Maxbotix.h>
#include <AltSoftSerial.h>
/*
    * PW is digital pin 11
    * PW1 is digital pin 7
    * TX is digital pin 6
    * AN is analog pin A0
    * Electromagnetic pin 13
    * Electromagnetic pin 12
    * D8 - AltSoftSerial RX
    * D9 - AltSoftSerial TX
 */
#define SERIAL_CLASS Serial_
#include <ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int16.h>


// Pin Numbers
const int lockerPin1 = 13;
const int lockerPin2 = 12;
//
Maxbotix rangeSensorPW(11, Maxbotix::PW, Maxbotix::LV);
Maxbotix rangeSensorPW_s(7, Maxbotix::PW, Maxbotix::LV);
Maxbotix rangeSensorAD(A0, Maxbotix::AN, Maxbotix::LV);
       
// variables will change:
ros::NodeHandle  nh;

AltSoftSerial BTserial; 

sensor_msgs::Range range_msg;
ros::Publisher pub_range( "/ultrasound", &range_msg);
char frameid[] = "/ultrasound1";
char frameid1[] = "/ultrasound2";

//***** call back function****
void messageCb(const std_msgs::Int16& msg){
  if(msg.data==1){
            digitalWrite(lockerPin1,HIGH);
            digitalWrite(lockerPin2,HIGH);
  }
  else{
           digitalWrite(lockerPin1,LOW);
           digitalWrite(lockerPin2,LOW);
  }
}

ros::Subscriber<std_msgs::Int16> sub("electromagnetic", &messageCb);

void setup()
{
  nh.initNode();
  nh.advertise(pub_range);
  nh.subscribe(sub);
  Serial.begin(9600);
  Serial.println(__FILE__);
  Serial.println(__DATE__);
  Serial.println(" ");
  BTserial.begin(9600);
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  pinMode(lockerPin1,OUTPUT);
  pinMode(lockerPin2,OUTPUT);
  range_msg.field_of_view = 0.1;  
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
    //
    BTserial.println("unlock_1010101");
    delay(100);
}


