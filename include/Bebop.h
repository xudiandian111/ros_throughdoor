//ROS Lib
#pragma once
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <bebop_msgs/Ardrone3PilotingStateAttitudeChanged.h>
#include <bebop_msgs/Ardrone3PilotingStatePositionChanged.h>
#include <bebop_msgs/Ardrone3PilotingStateAltitudeChanged.h>
#include <bebop_msgs/Ardrone3PilotingStateGpsLocationChanged.h>
#include <bebop_msgs/Ardrone3PilotingStateSpeedChanged.h>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include "Common.h"
#include "Control.h"
#include "Detections.h"
#include "Parameter.h"

class Bebop
{
public:
        Bebop(void);
        ~Bebop(void);
public:
	/*********fly command**************/
	geometry_msgs::Twist fly_up;
	geometry_msgs::Twist fly_down;
	geometry_msgs::Twist fly_left;
	geometry_msgs::Twist fly_right;
	geometry_msgs::Twist fly_forward;
	geometry_msgs::Twist fly_backward;
	geometry_msgs::Twist fly_turn_right;
	geometry_msgs::Twist fly_turn_left;
	geometry_msgs::Twist fly_pause;
	geometry_msgs::Twist change_camera;
	geometry_msgs::Twist auto_command;
	/*********about ros**************/
	ros::Publisher pub_takeoff;
	ros::Publisher pub_land;
	ros::Publisher pub_control;
	ros::Publisher pub_eland;
	ros::Publisher auto_control;
	ros::Publisher camera_control;
	ros::Subscriber angle;
	ros::Subscriber height;
	ros::Subscriber speed;
    image_transport::Subscriber image_sub;
    std_msgs::Empty emp_msg;

	static void readAngle(const bebop_msgs::Ardrone3PilotingStateAttitudeChanged::ConstPtr& msg);
	static void readHeight(const bebop_msgs::Ardrone3PilotingStateAltitudeChanged::ConstPtr& msg);
	static void readSpeed(const bebop_msgs::Ardrone3PilotingStateSpeedChanged::ConstPtr& msg);

    void cameraAngleControl();
    void rosVersion();
	void keyVersion();
	void initFlyParam();

    void generalControl();

    /**********for safe exit************/
	static bool   threadrosout;
	static bool   threadkeyout;
	/*********others**************/
	static bool   down_flag;
	static bool   drop_flag;
	/*********autoswitch**************/
	static bool   autoswitch;
	static double ardrone_speed;
	static double ardrone_height;
	static double ardrone_z_attitude;

	static int statictime;

};
