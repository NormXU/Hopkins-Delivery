/** @file demo_flight_control.cpp
*  @version 3.3
*  @date September, 2017
*
*  @brief
*  demo sample of how to use Local position control
*
*  @copyright 2017 DJI. All rights reserved.
*
*/

#include "dji_sdk_demo/demo_local_position_control.h"
#include "dji_sdk/dji_sdk.h"

ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;

ros::Publisher ctrlPosYawPub;

// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode = 255;
uint8_t current_gps_health = 0;

const int wsize = 10;
std::vector<double> left_sonar;
std::vector<double> right_sonar;

double left_sonar_curr = 0.0;
double right_sonar_curr = 0.0;
double yCmd;
int num_targets = 0;
geometry_msgs::PointStamped local_position;
sensor_msgs::NavSatFix current_gps_position;

double angle = 0.0;

struct _pid
{
	float SetDistance;
	float err_last_left;
	float err_last_right;
	float Kp, Kd;
	float K_saturate;
}pid;

void PID_INI() // Set PID Parameters
{
	pid.SetDistance = 0.5;// minimum safe distance
	pid.Kd = 0.6;  
	pid.Kp = 1.8;  
	pid.err_last_left = 0;
	pid.err_last_right = 0;
	pid.K_saturate = 60;
	ROS_INFO("PID Parameters Initialization Done !");
}
int main(int argc, char** argv)
{
	PID_INI();
	ros::init(argc, argv, "demo_local_position_control_node");
	ros::NodeHandle nh;
	// Subscribe to messages from dji_sdk_node
	ros::Subscriber JoySub = nh.subscribe("cmd_vel", 10, &twist_callback);
	ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
	ros::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);
	ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 10, &local_position_callback);
	ros::Subscriber gpsSub = nh.subscribe("dji_sdk/gps_position", 10, &gps_position_callback);
	ros::Subscriber gpsHealth = nh.subscribe("dji_sdk/gps_health", 10, &gps_health_callback);

	ros::Subscriber sonarSub = nh.subscribe("/ultrasound", 1, &sonar_callback);
	// Publish the control signal
	ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);
	// Basic services
	sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority>("dji_sdk/sdk_control_authority");
	drone_task_service = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
	query_version_service = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
	set_local_pos_reference = nh.serviceClient<dji_sdk::SetLocalPosRef>("dji_sdk/set_local_pos_ref");

	bool obtain_control_result = obtain_control();
	bool takeoff_result;
	set_local_position();
	if (is_M100())
	{
		ROS_INFO("M100 taking off!");
		takeoff_result = M100monitoredTakeoff();
	}
	else
	{
		ROS_INFO("A3/N3 taking off!");
		takeoff_result = monitoredTakeoff();
	}

	if (takeoff_result)
	{
		//! Enter total number of Targets
		num_targets = 2;
		//! Start Mission by setting Target state to 1
		target_set_state = 1;
	}

	ros::spin();
	return 0;
}

void twist_callback(const geometry_msgs::Twist::ConstPtr& msg) {
	static ros::Time start_time = ros::Time::now();
	ros::Duration elapsed_time = ros::Time::now() - start_time;


	position_ctrl(msg->linear.x, msg->angular.z);

}


/*!
* This function is called when local position data is available.
* In the example below, we make use of two arbitrary targets as
* an example for local position control.
*
*/

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg) {
	static ros::Time start_time = ros::Time::now();
	ros::Duration elapsed_time = ros::Time::now() - start_time;
	local_position = *msg;
	double xCmd, yCmd, zCmd;
	sensor_msgs::Joy controlPosYaw;

	/*
	// Down sampled to 50Hz loop
	if (elapsed_time > ros::Duration(0.02)) {
	start_time = ros::Time::now();
	if (target_set_state == 1) {
	//! First arbitrary target
	if (current_gps_health > 3) {
	setTarget(10, 15, 25, 2);
	local_position_ctrl(xCmd, yCmd, zCmd);
	}
	else
	{
	ROS_INFO("Cannot execute Local Position Control");
	ROS_INFO("Not enough GPS Satellites");
	//! Set Target set state to 0 in order to stop Local position control mission
	target_set_state = 0;
	}
	}
	if (target_set_state == 2) {
	//! Second arbitrary target
	if (current_gps_health > 3) {
	setTarget(-10, 5, 5, 2);
	local_position_ctrl(xCmd, yCmd, zCmd);
	}
	else
	{
	ROS_INFO("Cannot execute Local Position Control");
	ROS_INFO("Not enough GPS Satellites");
	//! Set Target set state to 0 in order to stop Local position control mission
	target_set_state = 0;
	}
	}
	}
	*/
}

/*!
* This function calculates the difference between target and current local position
* and sends the commands to the Position and Yaw control topic.
*
*/

void local_position_ctrl(double &xCmd, double &yCmd, double &zCmd)
{
	xCmd = target_offset_x - local_position.point.x;
	yCmd = target_offset_y - local_position.point.y;
	zCmd = target_offset_z;

	sensor_msgs::Joy controlPosYaw;
	controlPosYaw.axes.push_back(xCmd);
	controlPosYaw.axes.push_back(yCmd);
	controlPosYaw.axes.push_back(zCmd);
	controlPosYaw.axes.push_back(target_yaw);
	ctrlPosYawPub.publish(controlPosYaw);

	// 0.1m or 10cms is the minimum error to reach target in x y and z axes.
	// This error threshold will have to change depending on aircraft/payload/wind conditions.
	if (((std::abs(xCmd)) < 0.1) && ((std::abs(yCmd)) < 0.1) &&
		(local_position.point.z > (target_offset_z - 0.1)) && (local_position.point.z < (target_offset_z + 0.1))) {
		if (target_set_state <= num_targets) {
			ROS_INFO("%d of %d target(s) complete", target_set_state, num_targets);
			target_set_state++;
		}
		else
		{
			target_set_state = 0;
		}
	}
}

void position_ctrl(double lCmd, double aCmd)
{
	angle += aCmd;
	angle = fmod(angle, 2 * M_PI);
	double xCmd = lCmd * cos(angle);
	double yCmd = lCmd * sin(angle);
	ROS_INFO("angle:%f", angle);
	sensor_msgs::Joy controlPosYaw;
	controlPosYaw.axes.push_back(xCmd);
	controlPosYaw.axes.push_back(yCmd);
	controlPosYaw.axes.push_back(1.5);
	controlPosYaw.axes.push_back(angle);
	ctrlPosYawPub.publish(controlPosYaw);
}

bool takeoff_land(int task)
{
	dji_sdk::DroneTaskControl droneTaskControl;

	droneTaskControl.request.task = task;

	drone_task_service.call(droneTaskControl);

	if (!droneTaskControl.response.result)
	{
		ROS_ERROR("takeoff_land fail");
		return false;
	}

	return true;
}

bool obtain_control()
{
	dji_sdk::SDKControlAuthority authority;
	authority.request.control_enable = 1;
	sdk_ctrl_authority_service.call(authority);

	if (!authority.response.result)
	{
		ROS_ERROR("obtain control failed!");
		return false;
	}

	return true;
}

bool is_M100()
{
	dji_sdk::QueryDroneVersion query;
	query_version_service.call(query);

	if (query.response.version == DJISDK::DroneFirmwareVersion::M100_31)
	{
		return true;
	}

	return false;
}


void gps_position_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
	current_gps_position = *msg;
}

void gps_health_callback(const std_msgs::UInt8::ConstPtr& msg) {
	current_gps_health = msg->data;
}

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
	flight_status = msg->data;
}

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
{
	display_mode = msg->data;
}


/*!
* This function demos how to use the flight_status
* and the more detailed display_mode (only for A3/N3)
* to monitor the take off process with some error
* handling. Note M100 flight status is different
* from A3/N3 flight status.
*/
bool
monitoredTakeoff()
{
	ros::Time start_time = ros::Time::now();

	if (!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)) {
		return false;
	}

	ros::Duration(0.01).sleep();
	ros::spinOnce();

	// Step 1.1: Spin the motor
	while (flight_status != DJISDK::FlightStatus::STATUS_ON_GROUND &&
		display_mode != DJISDK::DisplayMode::MODE_ENGINE_START &&
		ros::Time::now() - start_time < ros::Duration(5)) {
		ros::Duration(0.01).sleep();
		ros::spinOnce();
	}

	if (ros::Time::now() - start_time > ros::Duration(5)) {
		ROS_ERROR("Takeoff failed. Motors are not spinnning.");
		return false;
	}
	else {
		start_time = ros::Time::now();
		ROS_INFO("Motor Spinning ...");
		ros::spinOnce();
	}


	// Step 1.2: Get in to the air
	while (flight_status != DJISDK::FlightStatus::STATUS_IN_AIR &&
		(display_mode != DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode != DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
		ros::Time::now() - start_time < ros::Duration(20)) {
		ros::Duration(0.01).sleep();
		ros::spinOnce();
	}

	if (ros::Time::now() - start_time > ros::Duration(20)) {
		ROS_ERROR("Takeoff failed. Aircraft is still on the ground, but the motors are spinning.");
		return false;
	}
	else {
		start_time = ros::Time::now();
		ROS_INFO("Ascending...");
		ros::spinOnce();
	}

	// Final check: Finished takeoff
	while ((display_mode == DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode == DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
		ros::Time::now() - start_time < ros::Duration(20)) {
		ros::Duration(0.01).sleep();
		ros::spinOnce();
	}

	if (display_mode != DJISDK::DisplayMode::MODE_P_GPS || display_mode != DJISDK::DisplayMode::MODE_ATTITUDE)
	{
		ROS_INFO("Successful takeoff!");
		start_time = ros::Time::now();
	}
	else
	{
		ROS_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
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
bool
M100monitoredTakeoff()
{
	ros::Time start_time = ros::Time::now();

	float home_altitude = current_gps_position.altitude;
	if (!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
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

	if (flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ||
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

bool set_local_position()
{
	dji_sdk::SetLocalPosRef localPosReferenceSetter;
	set_local_pos_reference.call(localPosReferenceSetter);
}


void sonar_callback(const sensor_msgs::Range::ConstPtr& msg) { // in cm
	if (msg->range < msg->min_range || msg->range > msg->max_range) return;
	if (msg->header.frame_id == "/ultrasound1") {
		left_sonar.push_back((double)msg->range);
		if (left_sonar.size() >= wsize) left_sonar.erase(left_sonar.begin());
		left_sonar_curr = std::accumulate(left_sonar.begin(), left_sonar.end(), 0) / left_sonar.size();
		//ROS_INFO("lll:%f", left_sonar_curr);
	}
	else if (msg->header.frame_id == "/ultrasound2") {
		right_sonar.push_back((double)msg->range);
		if (right_sonar.size() >= wsize) right_sonar.erase(right_sonar.begin());
		right_sonar_curr = std::accumulate(right_sonar.begin(), right_sonar.end(), 0) / right_sonar.size();
		//ROS_INFO("rrr:%f", right_sonar_curr);
	}
	sonar_position_ctrl(left_sonar_curr, right_sonar_curr);
}


void sonar_position_ctrl(double l, double r)
{ //cm to m
	l = l / 100.0;
	r = r / 100.0;

	//double yCmd = 0.0
	if (l <= pid.SetDistance)
	{
		if (-yCmd <= pid.K_saturate)
		{
			yCmd -= (pid.Kp * (pid.SetDistance - l) + pid.Kd * (pid.SetDistance - l - pid.err_last_left));
			pid.err_last_left = pid.SetDistance - l;
		}

	}
	if (r <= pid.SetDistance)
	{
		if (yCmd <= pid.K_saturate)
		{
			yCmd += (pid.Kp * (pid.SetDistance - r) + pid.Kd * (pid.SetDistance - r - pid.err_last_right));
			pid.err_last_right = pid.SetDistance - r;
		}
	}

	if (l > pid.SetDistance && r > pid.SetDistance) { yCmd = 0.0; pid.err_last_left = 0.0; pid.err_last_right = 0.0; }
	ROS_INFO("l:%f", l);
	ROS_INFO("LAST_left:%f", pid.err_last_left);
	ROS_INFO("r:%f", r);
	ROS_INFO("LAST_right:%f", pid.err_last_right);
	ROS_INFO("y:%f", yCmd);
	sensor_msgs::Joy controlPosYaw;
	controlPosYaw.axes.push_back(0.0);
	controlPosYaw.axes.push_back(yCmd);
	controlPosYaw.axes.push_back(1.5);
	controlPosYaw.axes.push_back(0.0);
	ctrlPosYawPub.publish(controlPosYaw);
}
