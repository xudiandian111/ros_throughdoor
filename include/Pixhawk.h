#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include "Parameter.h"
#include "Control.h"

class Pixhawk
{
private:
    ros::Subscriber pose_sub;
    ros::Subscriber state_sub;
    ros::Subscriber angle_sub;
    ros::Subscriber dis_sub;
    ros::Subscriber line_sub;

    ros::Publisher local_vel_pub;
    
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;

    geometry_msgs::Twist control_cmd;

    mavros_msgs::SetMode land_set_mode;
    mavros_msgs::CommandBool arm_cmd;
    mavros_msgs::SetMode offb_set_mode;
    static mavros_msgs::State current_state;

    static std_msgs::Float64 angle;
    static std_msgs::Float64 dis;
    static std_msgs::Bool flag_line;

    void initParam();
    void keyControl(std::string key);
    static void readPose(const geometry_msgs::PoseStamped::ConstPtr &msg);
    static void state_cb(const mavros_msgs::State::ConstPtr &msg);
    static void get_dis(const std_msgs::Float64::ConstPtr &msg);
    static void get_angle(const std_msgs::Float64::ConstPtr &msg);
    static void get_flag_line(const std_msgs::Bool::ConstPtr &msg);

    Control control;

public:
    Pixhawk();
    ~Pixhawk();
    void mavrosMain();
    void keyMain();
};
