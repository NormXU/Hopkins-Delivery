#include <ros/ros.h>
#include <ros/console.h>

#include <vector>
#include <iterator>
#include <string>

#include "dji_sdk/dji_sdk.h"
// #include <dji_sdk/GimbalAngleControl.h>
// #include <dji_sdk/GimbalSpeedControl.h>
// #include <dji_sdk/VelocityControl.h>
//DJI SDK includes
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SetLocalPosRef.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>

#include <geometry_msgs/PoseArray.h>
#include <cmath>
#include <iostream>

#include <Eigen/Geometry> 

ros::Subscriber velocity_control_x_sub;
ros::Subscriber velocity_control_y_sub;
ros::Subscriber velocity_control_yaw_sub;
ros::Subscriber position_track_enable_sub;
ros::Subscriber landing_condition_met_sub;
ros::Subscriber relanding_condition_met_sub;

ros::Publisher ctrlPosYawPub;
ros::Publisher ctrlPub;

//ros::ServiceClient velocity_control_service;
//ros::ServiceClient sdk_permission_control_service;
ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;

double velocity_control_effort_x;
double velocity_control_effort_y;
double velocity_control_effort_yaw;

const double descending_speed = -0.5;
const double ascending_speed = 0.5;

bool position_track_enabled = false; //ben lai shi false, for testing, i edited into true
bool landing_condition_met = false;
bool relanding_condition_met = false;

std::string topic_from_controller;

void velocityControlEffortXCallback(const std_msgs::Float64ConstPtr& velocity_control_effort_x_msg)
{
	velocity_control_effort_x = (velocity_control_effort_x_msg->data<=-15||velocity_control_effort_x_msg->data>=15)?0:velocity_control_effort_x_msg->data;
}

void velocityControlEffortYCallback(const std_msgs::Float64ConstPtr& velocity_control_effort_y_msg)
{
	velocity_control_effort_y = (velocity_control_effort_y_msg->data<=-15||velocity_control_effort_y_msg->data>=15)?0:velocity_control_effort_y_msg->data;
}

void velocityControlEffortYawCallback(const std_msgs::Float64ConstPtr& velocity_control_effort_yaw_msg)
{
	velocity_control_effort_yaw = velocity_control_effort_yaw_msg->data;
}

void positionTrackEnableCallback(const std_msgs::Bool& position_track_enable_msg)
{
  position_track_enabled = position_track_enable_msg.data;
}

void landingConditionMetCallback(const std_msgs::Bool& landing_condition_met_msg)
{
	landing_condition_met = landing_condition_met_msg.data;
}

void relandingConditionMetCallback(const std_msgs::Bool& relanding_condition_met_msg)
{
	relanding_condition_met = relanding_condition_met_msg.data;
}
/*
bool takeoff_land(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
    ROS_ERROR("takeoff_land fail");
    return false;
  }

  return true;
}
*/
bool obtain_control()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }

  return true;
}

bool reslease_control()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=0;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("Release control failed!");
    return false;
  }

  return true;
}
/*!
 * This function demos how to use M100 flight_status
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */
/*
bool
M100monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  float home_altitude = current_gps_position.altitude;
  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
  {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1: If M100 is not in the air after 10 seconds, fail.
  while (ros::Time::now() - start_time < ros::Duration(10))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ||
     current_gps_position.altitude - home_altitude < 1.0)
  {
    ROS_ERROR("Takeoff failed.");
    return false;
  }
  else
  {
    start_time = ros::Time::now();
    ROS_INFO("Successful takeoff!");
    ros::spinOnce();
  }
  return true;
}
*/
bool set_local_position()
{
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "position_track_controller");
	ros::NodeHandle nh;

	nh.param<std::string>("topic_from_controller", topic_from_controller, "/hd/position_track/velocity_control_effort_x");

	velocity_control_x_sub = nh.subscribe("/hd/position_track/velocity_control_effort_x", 10, &velocityControlEffortXCallback);
	velocity_control_y_sub = nh.subscribe("/hd/position_track/velocity_control_effort_y", 10, &velocityControlEffortYCallback);
	velocity_control_yaw_sub = nh.subscribe("/hd/position_track/velocity_control_effort_yaw", 10, &velocityControlEffortYawCallback);
	position_track_enable_sub = nh.subscribe("/hd/position_track/position_track_enable", 1, &positionTrackEnableCallback );
	landing_condition_met_sub = nh.subscribe("/hd/position_track/landing_condition_met", 1, &landingConditionMetCallback );   
	relanding_condition_met_sub = nh.subscribe("/hd/position_track/relanding_condition_met", 1, &relandingConditionMetCallback );   

	// velocity_control_service = nh.serviceClient<dji_sdk::VelocityControl>("dji_sdk/velocity_control");
	// sdk_permission_control_service = nh.serviceClient<dji_sdk::SDKPermissionControl>("dji_sdk/sdk_permission_control");

	// Subscribe to messages from dji_sdk_node
	// ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
	// ros::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);
	// ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 10, &local_position_callback);
	// ros::Subscriber gpsSub      = nh.subscribe("dji_sdk/gps_position", 10, &gps_position_callback);
	// ros::Subscriber gpsHealth      = nh.subscribe("dji_sdk/gps_health", 10, &gps_health_callback);
	// Publish the control signal
	//ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_ENUposition_yawrate", 10);// zhi qian bei zhu shi diao le

	ctrlPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);
std::cout << "Finally, I come here ! " << std::endl;
	// Basic services
	sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
	drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
	query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
	set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");

	//dji_sdk::VelocityControl velocity_control;
	sensor_msgs::Joy controlPosYaw;
    	sensor_msgs::Joy controlVelYawRate;
	sensor_msgs::Joy controlVelDesCondition; // in case of dui zhan overflow
	int frame = 0;
	double yaw_rate = 0;

	// dji_sdk::SDKPermissionControl sdk_permission_control;
	// sdk_permission_control.request.control_enable = 1;
	// bool control_requested = false;

	// while(!(sdk_permission_control_service.call(sdk_permission_control) && sdk_permission_control.response.result))
	// {
	// 	ROS_ERROR("Velocity controller: request control failed!");
	// }

	bool obtain_control_result = obtain_control();
  	set_local_position();
	ROS_INFO("Setting local pos!");
        //bool takeoff_result = M100monitoredTakeoff();
        //TODO
        //if(takeoff_result)
	//    {
		// //! Enter total number of Targets
		// num_targets = 2;
		// //! Start Mission by setting Target state to 1
		// target_set_state = 1;
	 //   }

	while(ros::ok())
	{ 
	        ros::spinOnce();
		std::cout << "position_track_enabled " << position_track_enabled << std::endl;
		std::cout << "landing_condition_met " << landing_condition_met << std::endl;
		//std::cout << "Get into while loop? " << std::endl;
		if(position_track_enabled)
		{
			if(landing_condition_met)
			{
				std::cout << "Get into landing condition met ?? " << std::endl;
				ROS_DEBUG_ONCE("Velocity controller: Landing condition met, going down");
				 //controlPosYaw.axes.push_back(velocity_control_effort_x);
				 //controlPosYaw.axes.push_back(velocity_control_effort_y);
				 //controlPosYaw.axes.push_back(descending_speed);
				 //controlPosYaw.axes.push_back(velocity_control_effort_yaw);
				 //ctrlPosYawPub.publish(controlPosYaw);
				uint8_t flag = (DJISDK::VERTICAL_VELOCITY   |
                DJISDK::HORIZONTAL_VELOCITY |
                DJISDK::YAW_RATE            |
                DJISDK::HORIZONTAL_BODY   |//TODO: figure out it...
				//DJISDK::VERTICAL_BODY   |
                DJISDK::STABLE_ENABLE);
				//controlVelYawRate.axes.push_back(velocity_control_effort_x);
				//controlVelYawRate.axes.push_back(velocity_control_effort_y);
				std::cout << "flag " << flag << std::endl;
                                controlVelDesCondition.axes.push_back(0.0);
				controlVelDesCondition.axes.push_back(0.0);
				controlVelDesCondition.axes.push_back(descending_speed);
				controlVelDesCondition.axes.push_back(velocity_control_effort_yaw);
				controlVelDesCondition.axes.push_back(flag);
				std::cout << "controlVelDesCondition.axes ?? " << controlVelDesCondition << std::endl;
				std::cout << "---------------------------------------------- " << std::endl;

				ctrlPub.publish(controlVelDesCondition);
				// velocity_control.request.frame = frame;
				// velocity_control.request.vx = velocity_control_effort_x;
				// velocity_control.request.vy = velocity_control_effort_y;
				// velocity_control.request.vz = descending_speed;
				// velocity_control.request.yawRate = velocity_control_effort_yaw;

				// if(!(velocity_control_service.call(velocity_control) && velocity_control.response.result))
				// {
				// 	ROS_ERROR("Velocity controller: velocity control failed!");
				// }
			}
			else if(relanding_condition_met)
			{
				std::cout << "Get into relanding ?? " << std::endl;
				ROS_DEBUG_ONCE("Velocity controller: Relanding condition met, going up");
				// velocity_control.request.frame = frame;
				// velocity_control.request.vx = velocity_control_effort_x;
				// velocity_control.request.vy = velocity_control_effort_y;
				// velocity_control.request.vz = ascending_speed;
				// velocity_control.request.yawRate = velocity_control_effort_yaw;

				// if(!(velocity_control_service.call(velocity_control) && velocity_control.response.result))
				// {
				// 	ROS_ERROR("Velocity controller: velocity control failed!");
				// }
				// controlPosYaw.axes.push_back(velocity_control_effort_x);
				// controlPosYaw.axes.push_back(velocity_control_effort_y);
				// controlPosYaw.axes.push_back(ascending_speed);
				// controlPosYaw.axes.push_back(velocity_control_effort_yaw);
				// ctrlPosYawPub.publish(controlPosYaw);
				
				uint8_t flag = (DJISDK::VERTICAL_VELOCITY   |
                DJISDK::HORIZONTAL_VELOCITY |
                DJISDK::YAW_RATE            |
                DJISDK::HORIZONTAL_BODY   |//TODO: figure out it...
				//DJISDK::VERTICAL_BODY   |
                DJISDK::STABLE_ENABLE);
				std::cout << "flag " << flag << std::endl;
				controlVelYawRate.axes.push_back(velocity_control_effort_x);
				controlVelYawRate.axes.push_back(velocity_control_effort_y);
				controlVelYawRate.axes.push_back(ascending_speed);
				controlVelYawRate.axes.push_back(velocity_control_effort_yaw);
				controlVelYawRate.axes.push_back(flag);
				std::cout << "controlVelYawRate.axes ?? " << controlVelYawRate << std::endl;
				std::cout << "---------------------------------------------- " << std::endl;
				ctrlPub.publish(controlVelYawRate);
			}
			else
			{
				ROS_DEBUG_THROTTLE(3, "Velocity controller: Received control effort, flying the drone");
				// velocity_control.request.frame = frame;
				// velocity_control.request.vx = velocity_control_effort_x;
				// velocity_control.request.vy = velocity_control_effort_y;
				// velocity_control.request.vz = 0;
				// velocity_control.request.yawRate = velocity_control_effort_yaw;

				// if(!(velocity_control_service.call(velocity_control) && velocity_control.response.result))
				// {
				// 	ROS_ERROR("Velocity controller: velocity control failed!");
				// }
				// controlPosYaw.axes.push_back(velocity_control_effort_x);
				// controlPosYaw.axes.push_back(velocity_control_effort_y);
				// controlPosYaw.axes.push_back(0.0);
				// controlPosYaw.axes.push_back(velocity_control_effort_yaw);
				// ctrlPosYawPub.publish(controlPosYaw);
				std::cout << "shaking ?? " << std::endl;
				uint8_t flag = (DJISDK::VERTICAL_VELOCITY   |
                DJISDK::HORIZONTAL_VELOCITY |
                DJISDK::YAW_RATE            |
                DJISDK::HORIZONTAL_BODY   |//TODO: figure out it...
				//DJISDK::VERTICAL_BODY   |
                DJISDK::STABLE_ENABLE);
				controlVelYawRate.axes.push_back(velocity_control_effort_x);
				controlVelYawRate.axes.push_back(velocity_control_effort_y);
				controlVelYawRate.axes.push_back(0.0);
				controlVelYawRate.axes.push_back(velocity_control_effort_yaw);
				controlVelYawRate.axes.push_back(flag);
				std::cout << "controlVelYawRate.axes ?? " << controlVelYawRate << std::endl;
				std::cout << "---------------------------------------------- " << std::endl;
				ctrlPub.publish(controlVelYawRate);
			}
			
		}
//		else
//		{
//			std::cout << "Continue" << std::endl;			
//			continue;
//		}
		ros::Duration(1).sleep();
		//	
	}

	// sdk_permission_control.request.control_enable = 0;
	// sdk_permission_control_service.call(sdk_permission_control);
	bool release_control_result = reslease_control();
}
