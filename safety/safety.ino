//#include<ros.h>
#include <Wire.h>
#include <LSM303.h>
//#include<std_msgs/Int16.h>

LSM303 compass;

char report[80];
double accelarationZ;
//double accelerationX;
//double accelerationY;
const int BuzzerPin = 10;
const int CrashAcceleration = 0;
//const int TiltAcceleration = 0.1;
//const int StaticAcceleration = -15000;
//
//  ros::NodeHandle  nh;
// ********** Call back function**********
// void sdkCb(const std_msgs::Int16& msg){
//  int height1 = msg.data;
//  delay(500);
//  int height2 = msg.data;
//  if (height2-height1)
//   {
//    return;
//    }
//    else
//    {
//      accelarationZ =  compass.a.z;
//      AccelerationCheck(accelarationZ)?  digitalWrite(BuzzerPin,LOW);
//      }
  
//}
  
//ros::Subscriber<std_msgs::Int16> position_sub("/dji_sdk/local_position", &sdkCb);

void setup()
{
 // nh.initNode();
  //nh.subscribe(position_sub);
  Serial.begin(9600);
  Wire.begin();
  compass.init();
  compass.enableDefault();
  pinMode(BuzzerPin, OUTPUT);
}

void loop()
{
  compass.read();
    
  snprintf(report, sizeof(report), "A: %6d %6d %6d    M: %6d %6d %6d",
    compass.a.x, compass.a.y, compass.a.z,
    compass.m.x, compass.m.y, compass.m.z);
    
  Serial.println(report);
  accelarationZ =  compass.a.z;
  if (AccelerationCheck(accelarationZ)){
    digitalWrite(BuzzerPin,HIGH);
    delay(2000);
    digitalWrite(BuzzerPin,LOW);
  } else {
    digitalWrite(BuzzerPin,LOW);
  }
  delay(10);
}

// check whether acceleration is continuously high
bool AccelerationCheck(double ac)
{
   if (ac > CrashAcceleration)
  {
          delay(100);// delay in ms
          compass.read();
          double ac2 = compass.a.z;
          Serial.println(ac2);
          if(ac2 > CrashAcceleration)
          {
              return true;
              //digitalWrite(BuzzerPin,HIGH);
           }
          else
          {
              return false;
              //digitalWrite(BuzzerPin,LOW);
          }
    }
    else {return false;}    
  }
