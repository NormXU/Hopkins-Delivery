#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/tf.h>
#include <ros/console.h>
// #include <apriltags_ros/AprilTagDetection.h>
// #include <apriltags_ros/AprilTagDetectionArray.h>
// #include <dji_sdk/Gimbal.h>
// #include <dji_sdk/LocalPosition.h>
// #include <dji_sdk/AttitudeQuaternion.h>
// #include <dji_sdk/GlobalPosition.h>
//DJI SDK includes
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SetLocalPosRef.h>

#include "dji_sdk/dji_sdk.h"
#include <tag.h>

#include <cmath>
#include <Eigen/Geometry> 

#include <vector>
#include <iterator>
#include <algorithm>

// #define APRILTAGS_ROS
#define APRILTAGS

ros::Subscriber apriltags_36h11_sub;
ros::Subscriber apriltags_16h5_sub;
//ros::Subscriber gimbal_ori_sub;
ros::Subscriber local_position_sub;
ros::Subscriber landing_enable_sub;
//ros::Subscriber attitude_quaternion_sub;
ros::Subscriber flight_status_sub;
ros::Subscriber global_position_sub;

ros::Subscriber attitude_sub;
ros::Subscriber gps_sub;

ros::Publisher setpoint_x_pub;
ros::Publisher setpoint_y_pub;
ros::Publisher setpoint_yaw_pub;
ros::Publisher landing_condition_met_pub;
ros::Publisher relanding_condition_met_pub;
ros::Publisher yaw_state_pub;
ros::Publisher x_state_pub;
ros::Publisher y_state_pub;
ros::Publisher z_state_pub;


double gimbal_roll;
double gimbal_yaw;
double gimbal_pitch;
//double gimbal_roll = 0.0 * M_PI / 180;
//double gimbal_yaw = 0.0 * M_PI / 180;
//double gimbal_pitch = -90.0 * M_PI / 180;

double local_x;
double local_y;
double local_z;
double flight_height;

double error_yaw;

double heading_q0;
double heading_q1;
double heading_q2;
double heading_q3;

Eigen::Quaternion<double> drone_heading;
double yaw_state = 0;

double setpoint_x = 0;
double setpoint_y = 0;
double setpoint_z = 0;
double setpoint_yaw = 0;

const int tag_16h5_num = 7;
double landing_threshold_36h11 = 1.0;
double landing_threshold_16h5 = 0.1;
double reland_height_min_threshold = 1.3;
double reland_height_max_threshold = 2.5;

bool relanding = false;

int flight_status;


std_msgs::Bool landing_condition_met_msg;
std_msgs::Bool relanding_condition_met_msg;

std_msgs::Float64 x_state_msg;
std_msgs::Float64 y_state_msg;
std_msgs::Float64 z_state_msg;

Tag *tag_36h11_6;

std::vector<Tag *> tags_16h5;
std::vector<int> found_tag_id;
std::vector<double> yaw_errors;

std::string tag_16h5_detection_topic;
std::string tag_36h11_detection_topic;

bool found_36h11 = false;
bool found_16h5 = false;

bool first_start = true;

bool landing_enabled = false;
// bool landing_enabled = true;// it was false at the beginning, for testing, i edited into true

#ifdef APRILTAGS
#include <apriltags/AprilTagDetections.h>

void apriltags36h11Callback(const apriltags::AprilTagDetections::ConstPtr& apriltag_pos_msg)
{
  if(std::begin(apriltag_pos_msg->detections) == std::end(apriltag_pos_msg->detections))
  {
    found_36h11 = false;
    return;
  }
  else
  {
    for(auto it = std::begin(apriltag_pos_msg->detections); it != std::end(apriltag_pos_msg->detections); ++ it)
    {
      if((*it).id == 2)
      {
        tag_36h11_6->updateTagState((*it).pose);
      }
      
    }
    found_36h11 = true;
    
  }
}


void apriltags16h5Callback(const apriltags::AprilTagDetections::ConstPtr& apriltag_pos_msg)
{
  if(std::begin(apriltag_pos_msg->detections) == std::end(apriltag_pos_msg->detections))
  {
    found_16h5 = false;
    return;
  }
  else
  {
    found_tag_id.clear();
    for(auto it = std::begin(apriltag_pos_msg->detections); it != std::end(apriltag_pos_msg->detections); ++ it)
    {
      if((*it).id >= 0 && (*it).id <= 6)
      {
        tags_16h5[(*it).id]->updateTagState((*it).pose);
        found_tag_id.push_back((*it).id);
      }
      
    }
    for(int i = 0; i < tag_16h5_num; i++)
    {
      if(std::find(found_tag_id.begin(), found_tag_id.end(), i) == found_tag_id.end())
      {
        tags_16h5[i]->setMissing();
      }
    }

    found_16h5 = true;
  }
}

#else

#include <apriltags_ros/AprilTagDetectionArray.h>

void apriltags36h11Callback(const apriltags_ros::AprilTagDetectionArray::ConstPtr& apriltag_pos_msg)
{
  if(std::begin(apriltag_pos_msg->detections) == std::end(apriltag_pos_msg->detections))
  {
    found_36h11 = false;
    tag_36h11_6->setMissing();
    return;
  }
  else
  {
    for(auto it = std::begin(apriltag_pos_msg->detections); it != std::end(apriltag_pos_msg->detections); ++ it)
    {
      if((*it).id == 2)
      {
        tag_36h11_6->updateTagState((*it).pose.pose);
      }
      
    }

    found_36h11 = true;

  }
}


void apriltags16h5Callback(const apriltags_ros::AprilTagDetectionArray::ConstPtr& apriltag_pos_msg)
{
  if(std::begin(apriltag_pos_msg->detections) == std::end(apriltag_pos_msg->detections))
  {
    found_16h5 = false;
    for(int i = 0; i < tag_16h5_num; i++)
    {
      tags_16h5[i]->setMissing();
    }
    return;
  }
  else
  {
    found_tag_id.clear();
    for(auto it = std::begin(apriltag_pos_msg->detections); it != std::end(apriltag_pos_msg->detections); ++ it)
    {
      if((*it).id >= 0 && (*it).id <= 6)
      {
        tags_16h5[(*it).id]->updateTagState((*it).pose.pose);
        found_tag_id.push_back((*it).id);
      }
      
    }
    for(int i = 0; i < tag_16h5_num; i++)
    {
      if(std::find(found_tag_id.begin(), found_tag_id.end(), i) == found_tag_id.end())
      {
        tags_16h5[i]->setMissing();
      }
    }

    found_16h5 = true;
  }
}

#endif

// void gimbalOrientationCallback(const dji_sdk::Gimbal::ConstPtr& gimbal_ori_msg)
// {
//   gimbal_roll = gimbal_ori_msg->roll * M_PI / 180;
//   gimbal_yaw = gimbal_ori_msg->yaw * M_PI / 180;
//   gimbal_pitch = gimbal_ori_msg->pitch * M_PI / 180;
// }

void localPositionCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  local_x = msg->point.x;
  local_y = msg->point.y;
  local_z = msg->point.z;
}

// void attitudeQuaternionCallback(const dji_sdk::AttitudeQuaternion::ConstPtr& attitude_quaternion_msg)
// {
//   heading_q0 = attitude_quaternion_msg->q0;
//   heading_q1 = attitude_quaternion_msg->q1;
//   heading_q2 = attitude_quaternion_msg->q2;
//   heading_q3 = attitude_quaternion_msg->q3;

//   drone_heading = Eigen::Quaternion<double>(heading_q0, heading_q1, heading_q2, heading_q3);
//   yaw_state = atan2(2*(heading_q0 * heading_q3 + heading_q1 * heading_q2), 1 - 2 * (heading_q2 * heading_q2 + heading_q3 * heading_q3)) / M_PI * 180;
// }

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  heading_q0 = msg->quaternion.w;
  heading_q1 = msg->quaternion.x;
  heading_q2 = msg->quaternion.y;
  heading_q3 = msg->quaternion.z;
  drone_heading = Eigen::Quaternion<double>(heading_q0, heading_q1, heading_q2, heading_q3);
  yaw_state = atan2(2*(heading_q0 * heading_q3 + heading_q1 * heading_q2), 1 - 2 * (heading_q2 * heading_q2 + heading_q3 * heading_q3)) / M_PI * 180;

}

// void globalPositionCallback(const dji_sdk::GlobalPosition::ConstPtr& global_position_msg)
// {
//   flight_height = global_position_msg->height;
// }
void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  flight_height = msg->altitude;
}

void landingEnableCallback(const std_msgs::Bool& landing_enable_msg)
{
  landing_enabled = landing_enable_msg.data;

}

void flightStatusCallback(const std_msgs::UInt8& flight_status_msg)
{
  flight_status = (int)flight_status_msg.data;
}

void descend()
{
  std::cout << "landing_enable? " << landing_enabled << std::endl;
  if(landing_enabled)
  {
    landing_condition_met_msg.data = true;
    landing_condition_met_pub.publish(landing_condition_met_msg);

    relanding_condition_met_msg.data = false;
    relanding_condition_met_pub.publish(relanding_condition_met_msg);

    relanding = false;
    std::cout << "descending" << std::endl;
    ROS_DEBUG_THROTTLE(2, "Descending");
  }
  
}

void ascend()
{
  if(landing_enabled)
  {
    landing_condition_met_msg.data = false;
    landing_condition_met_pub.publish(landing_condition_met_msg);

    relanding_condition_met_msg.data = true;
    relanding_condition_met_pub.publish(relanding_condition_met_msg);

    relanding = true;
    ROS_DEBUG_THROTTLE(2, "Ascending");
    std::cout << "ascending" << std::endl;
  }
}

void hover()
{
  landing_condition_met_msg.data = false;
  landing_condition_met_pub.publish(landing_condition_met_msg);

  relanding_condition_met_msg.data = false;
  relanding_condition_met_pub.publish(relanding_condition_met_msg);

  relanding = false;
  std::cout << "hovering" << std::endl;
}

void print_parameters()
{
  ROS_INFO("Listening to 36h11 apriltag detection topic: %s", tag_36h11_detection_topic.c_str());
  ROS_INFO("Listening to 16h5 apriltag detection topic: %s", tag_16h5_detection_topic.c_str());
  ROS_INFO("landing_threshold_36h11: %f", landing_threshold_36h11);
  ROS_INFO("landing_threshold_16h5: %f", landing_threshold_16h5);
  ROS_INFO("reland_height_min_threshold: %f", reland_height_min_threshold);
  ROS_INFO("reland_height_max_threshold: %f", reland_height_max_threshold);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "position_track_setpoint_node");
  ROS_INFO("Starting position track setpoint publisher");
  ros::NodeHandle nh;
  ros::NodeHandle node_priv("~");

  node_priv.param<std::string>("tag_36h11_detection_topic", tag_36h11_detection_topic, "/apriltags/36h11/detections");
  node_priv.param<std::string>("tag_16h5_detection_topic", tag_16h5_detection_topic, "/apriltags/16h5/detections");
  node_priv.param<double>("landing_threshold_36h11", landing_threshold_36h11, 1.0);
  node_priv.param<double>("landing_threshold_16h5", landing_threshold_16h5, 0.1);
  node_priv.param<double>("reland_height_min_threshold", reland_height_min_threshold, 1.3);
  node_priv.param<double>("reland_height_max_threshold", reland_height_max_threshold, 2.5);

  print_parameters();

  setpoint_x_pub = nh.advertise<std_msgs::Float64>("/hd/position_track/setpoint_x", 10);
  setpoint_y_pub = nh.advertise<std_msgs::Float64>("/hd/position_track/setpoint_y", 10);
  setpoint_yaw_pub = nh.advertise<std_msgs::Float64>("/hd/position_track/setpoint_yaw", 10);
  landing_condition_met_pub = nh.advertise<std_msgs::Bool>("/hd/position_track/landing_condition_met", 10);
  relanding_condition_met_pub = nh.advertise<std_msgs::Bool>("/hd/position_track/relanding_condition_met", 10);
  yaw_state_pub = nh.advertise<std_msgs::Float64>("/hd/position_track/yaw_state", 10);
  x_state_pub = nh.advertise<std_msgs::Float64>("/hd/position_track/x_state", 10);
  y_state_pub = nh.advertise<std_msgs::Float64>("/hd/position_track/y_state", 10);
  z_state_pub = nh.advertise<std_msgs::Float64>("/hd/position_track/z_state", 10);


  apriltags_36h11_sub = nh.subscribe(tag_36h11_detection_topic, 1, apriltags36h11Callback);
  apriltags_16h5_sub = nh.subscribe(tag_16h5_detection_topic, 1, apriltags16h5Callback);
  //gimbal_ori_sub = nh.subscribe("/dji_sdk/gimbal", 1000, gimbalOrientationCallback);
  local_position_sub = nh.subscribe("dji_sdk/local_position", 10, &localPositionCallback);
  // we  need to rostopic pub -1  1  here, in order to landing
  landing_enable_sub = nh.subscribe("/hd/position_track/landing_enable", 1, landingEnableCallback ); 

  //attitude_quaternion_sub = nh.subscribe("/dji_sdk/attitude_quaternion", 1, attitudeQuaternionCallback ); 
  flight_status_sub = nh.subscribe("/dji_sdk/flight_status", 1, flightStatusCallback);
  //global_position_sub = nh.subscribe("/dji_sdk/global_position", 10, globalPositionCallback);
  gps_sub      = nh.subscribe("dji_sdk/gps_position", 10,
                                             &gps_callback);
  attitude_sub = nh.subscribe("dji_sdk/attitude", 10,
                                             &attitude_callback);

  for(int i = 0; i < tag_16h5_num; i++)
  {
    tags_16h5.push_back(new Tag());
  }
  
  tags_16h5[0]->setToLandingCenterTranslation(Eigen::Vector3d(-0.3,  0.0, 0.0));
  tags_16h5[1]->setToLandingCenterTranslation(Eigen::Vector3d(0.0,   0.0, 0.0));
  tags_16h5[2]->setToLandingCenterTranslation(Eigen::Vector3d(0.0,0.115, 0.0));
  tags_16h5[3]->setToLandingCenterTranslation(Eigen::Vector3d(0.3,   0.0, 0.0));
  tags_16h5[4]->setToLandingCenterTranslation(Eigen::Vector3d(0.0, -0.115, 0.0));
  tags_16h5[5]->setToLandingCenterTranslation(Eigen::Vector3d(0.0, 0.23, 0.0));
  tags_16h5[6]->setToLandingCenterTranslation(Eigen::Vector3d(0.0,  -0.23, 0.0));

  tag_36h11_6 = new Tag();
  //tag_36h11_6->setToLandingCenterTranslation(Eigen::Vector3d(0.0,  0.485, 0.0));
  tag_36h11_6->setToLandingCenterTranslation(Eigen::Vector3d(0.0,  0.165, 0.0));

  //camera to gimbal transformation
  Eigen::Matrix3d camera_to_gimbal_transformation;
  camera_to_gimbal_transformation << 0, 0, 1,
                                     1, 0, 0,
                                     0, 1, 0;

  Eigen::AngleAxisd yawAngle;
  Eigen::AngleAxisd pitchAngle;
  Eigen::AngleAxisd rollAngle;

  Eigen::Quaternion<double> q;

  //camera to drone transformation
  Eigen::Matrix3d camera_to_drone_transformation;

  std_msgs::Float64 setpoint_x_msg;
  std_msgs::Float64 setpoint_y_msg;
  std_msgs::Float64 setpoint_yaw_msg;
  std_msgs::Float64 yaw_state_msg;

  double yaw_sum = 0;
  double average_yaw_error = 0;
  double found_tag_count = 0;
  double tag_36h11_yaw_state;


  Eigen::Vector3d landing_center_position_sum(0, 0, 0);
  Eigen::Vector3d average_landing_center_position(0, 0, 0);
  Eigen::Vector3d landing_center_position;


  ros::Rate loop_rate(200); 

  ros::spinOnce();
  if(!found_16h5 && !found_36h11)
  {
    setpoint_x = local_x;
    setpoint_y = local_y;
  }

  while (ros::ok())
  {
    ros::spinOnce();

    if(flight_status != 3)
    {
      std::cout << "flight status" << flight_status << std::endl;
      continue;
    }

    if(found_16h5 || found_36h11)
    {
      ROS_DEBUG_ONCE("Setpoint node: Found Apriltag");
      first_start = false;
      yawAngle = Eigen::AngleAxisd(gimbal_yaw, Eigen::Vector3d::UnitZ());
      pitchAngle = Eigen::AngleAxisd(gimbal_pitch, Eigen::Vector3d::UnitY());
      rollAngle = Eigen::AngleAxisd(gimbal_roll, Eigen::Vector3d::UnitX());

      q = rollAngle * pitchAngle * yawAngle;

      camera_to_drone_transformation = q.matrix() * camera_to_gimbal_transformation;

      std::cout << "cam to drone: " <<  camera_to_drone_transformation <<std::endl; 

      //get the averaging landing center position
      landing_center_position_sum = Eigen::Vector3d(0, 0, 0);
      found_tag_count = 0;  

    
      for(int i = 0; i < tag_16h5_num; i++)
        {
          if(tags_16h5[i]->isFound())
          {
            found_tag_count ++;
            tags_16h5[i]->calculateDroneFramePosition(camera_to_drone_transformation);
            tags_16h5[i]->calculateDroneFrameOrientation(camera_to_drone_transformation);
            landing_center_position = tags_16h5[i]->getLandingCenterPosition();
            landing_center_position_sum += landing_center_position;
          }
        }
        if(tag_36h11_6->isFound())
        {
          found_tag_count ++;
          tag_36h11_6->calculateDroneFramePosition(camera_to_drone_transformation);
          tag_36h11_6->calculateDroneFrameOrientation(camera_to_drone_transformation);
          landing_center_position_sum += tag_36h11_6->getLandingCenterPosition();

        }

        average_landing_center_position = landing_center_position_sum / found_tag_count;

      //get the averaging landing center yaw error
      yaw_sum = 0;
      yaw_errors.clear();
      for(int i = 0; i < tag_16h5_num; i++)
      {
        if(tags_16h5[i]->isFound())
          {
            yaw_errors.push_back(tags_16h5[i]->getYawError());
          }
      }

      if(tag_36h11_6->isFound())
        {
          yaw_errors.push_back(tag_36h11_6->getYawError());
        }
      if(yaw_errors.size() > 0)
      {
        //Take the median as an estimator
        std::sort (yaw_errors.begin(), yaw_errors.end());
        average_yaw_error = yaw_errors[found_tag_count / 2] / M_PI * 180;
      }

      //landing logic
      if(found_16h5)
      {
        ROS_DEBUG_ONCE("Setpoint node: Found 16h5 tags, entering fine tune state");
        if(fabs(average_landing_center_position(0)) < landing_threshold_16h5 && fabs(average_landing_center_position(1)) < landing_threshold_16h5)
        {
            descend();       
        }
        else//not within the range
        {
          if(flight_height < reland_height_min_threshold)
          {
            ascend();
          }
          else
          {
            hover();
          }
        }
      }
      else//found_16h5 not found
      {
        std::cout << "Found 36h11" << std::endl;
	std::cout << "average_landing_position  " << (average_landing_center_position(0)) << " ," << (average_landing_center_position(1)) << std::endl;
        std::cout << "local_position  " << local_x << " ," << local_y << std::endl;
	std::cout << "landing_threshold_36h11  " << landing_threshold_36h11 << std::endl;
        if(fabs(average_landing_center_position(0)) < landing_threshold_36h11 && fabs(average_landing_center_position(1)) < landing_threshold_36h11)
        {
	 std::cout << "Ready to descending " << std::endl;
         descend();
        }
        else
        {
          hover();
        }
      }

      setpoint_yaw = yaw_state + average_yaw_error - 90;
      setpoint_yaw_msg.data = setpoint_yaw;
      setpoint_yaw_pub.publish(setpoint_yaw_msg);

      setpoint_x = average_landing_center_position(0) + local_x;
      setpoint_y = average_landing_center_position(1) + local_y;
std::cout << "average_landing_position  " << (average_landing_center_position(0)) << " ," << (average_landing_center_position(1)) << std::endl;
      std::cout << "setpoint  " << setpoint_x << " ," << setpoint_y<<","<<setpoint_yaw<< std::endl;

      setpoint_x_msg.data = setpoint_x;
      setpoint_y_msg.data = setpoint_y;

      setpoint_x_pub.publish(setpoint_x_msg);
      setpoint_y_pub.publish(setpoint_y_msg);

      yaw_state_msg.data = yaw_state;
      yaw_state_pub.publish(yaw_state_msg);


      x_state_msg.data = local_x;
      y_state_msg.data = local_y;
      z_state_msg.data = flight_height;
      if(flight_height < 0)
      {
        ROS_DEBUG_THROTTLE(1, "Setpoint node: Flight height is negative, go to hell DJI");
      }

      x_state_pub.publish(x_state_msg); 
      y_state_pub.publish(y_state_msg); 
      z_state_pub.publish(z_state_msg); 
    }
    else
    {

      if(relanding)
      {
        if(flight_height < reland_height_max_threshold)
        {
          ascend();
        }
        else
        {
          hover();
        }
      }
      else
      {
        hover();
      }

      if(first_start)
      {
        setpoint_x_msg.data = local_x;
        setpoint_y_msg.data = local_y;
        setpoint_yaw_msg.data = yaw_state;
        yaw_state_msg.data = yaw_state;
        
      }
      else
      {
        setpoint_x_msg.data = setpoint_x;
        setpoint_y_msg.data = setpoint_y;
        setpoint_yaw_msg.data = yaw_state;
        yaw_state_msg.data = yaw_state;
      }

      setpoint_x_pub.publish(setpoint_x_msg);
      setpoint_y_pub.publish(setpoint_y_msg);
      setpoint_yaw_pub.publish(setpoint_yaw_msg);
      yaw_state_pub.publish(yaw_state_msg);

      x_state_msg.data = local_x;
      y_state_msg.data = local_y;
      z_state_msg.data = flight_height;

      x_state_pub.publish(x_state_msg); 
      y_state_pub.publish(y_state_msg); 
      z_state_pub.publish(z_state_msg); 
    }

    loop_rate.sleep();
  }
}
