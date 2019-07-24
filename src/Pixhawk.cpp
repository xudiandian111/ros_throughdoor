#include "Pixhawk.h"
#define keytest
/**
 * mode 参数解释
 * 可能的分类
 * 分类一：
 * mode分为三个模式
 * land 模式设置为AUTO.LAND
 * takeoff 模式设置为定高飞行
 * auto 模式设置为自动飞行
 * 分类二
 * mode分为四个模式
 * land 模式设置为AUTO.LAND
 * takeoff setlocalpoint 定点定高飞行
 * wait 定高飞行
 * auto 任务飞行
 * **/
mavros_msgs::State Pixhawk::current_state;
Pixhawk::Pixhawk()
{
}
Pixhawk::~Pixhawk()
{
}
void Pixhawk::initParam()
{
    Parameter::initParam();
    control_cmd.angular.x = 0.0;
    control_cmd.angular.y = 0.0;
    control_cmd.angular.z = 0.0;
    control_cmd.linear.x = 0.0;
    control_cmd.linear.y = 0.0;
    control_cmd.linear.z = 0.0;
}
void Pixhawk::get_angle(const std_msgs::Float64::ConstPtr &msg)
{
    Parameter::set("/auto/angle/x", msg->data);
}
void Pixhawk::get_dis(const std_msgs::Float64::ConstPtr &msg)
{
    Parameter::set("/auto/point/y", msg->data);
    ROS_ERROR("okk");
}
void Pixhawk::get_flag_line(const std_msgs::Bool::ConstPtr &msg)
{
    Parameter::set("/auto/flag/line", msg->data);
}
void Pixhawk::state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

void Pixhawk::readPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    Parameter::set<double>({{"/state/pose/x", msg->pose.position.x},
                            {"/state/pose/y", msg->pose.position.y},
                            {"/state/pose/z", msg->pose.position.z}});
}
void Pixhawk::mavrosMain()
{
    ROS_WARN("init 1.1");
    Pixhawk::initParam();

    ros::NodeHandle nh;
    state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 1, Pixhawk::state_cb);
    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 1, Pixhawk::readPose);
    
    angle_sub = nh.subscribe<std_msgs::Float64>("/Lines/angle", 2, Pixhawk::get_angle);
    dis_sub = nh.subscribe<std_msgs::Float64>("/Lines/dis", 2, Pixhawk::get_dis);
    line_sub = nh.subscribe<std_msgs::Bool>("/Lines/flag_line", 2, Pixhawk::get_flag_line);

    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    local_vel_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 1);
    ros::Rate rate(20.0);
    
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_WARN("cennected");

    offb_set_mode.request.custom_mode = "OFFBOARD";
    land_set_mode.request.custom_mode = "AUTO.LAND";
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    int time=0;
    std::string mode;

    while (ros::ok())
    {
        mode = Parameter::getString("/state/mode/current");

        if( mode == "land" &&
            current_state.mode != "AUTO.LAND" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            arm_cmd.request.value = false;
            if( set_mode_client.call(land_set_mode)&&
                land_set_mode.response.mode_sent)
            {
                ROS_INFO("Autoland enabled");
                time = 0;
            }
            if( arming_client.call(arm_cmd) &&
                arm_cmd.response.success)
            {
                ROS_INFO("Arm disabled");
            }
            arm_cmd.request.value = true;
            last_request = ros::Time::now();
        }
        else if ( current_state.mode != "OFFBOARD" &&
                  mode != "land" &&
                  (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else if( !current_state.armed &&
                 mode != "land" &&
                 (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( arming_client.call(arm_cmd) &&
                arm_cmd.response.success)
            {
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }

        if(current_state.armed)
        {
            control.controlMain(mode);
        }
        else
        {
            Pixhawk::initParam();
        }
        control_cmd.linear.x = Parameter::getDouble("/control/speed/x");
        control_cmd.linear.y = Parameter::getDouble("/control/speed/y");
        control_cmd.linear.z = Parameter::getDouble("/control/speed/z");
        control_cmd.angular.x = 0.0;
        control_cmd.angular.y = 0.0;
        control_cmd.angular.z = Parameter::getDouble("/control/angle/z");
        local_vel_pub.publish(control_cmd);
        time++;
        ros::spinOnce();
        rate.sleep();
    }
    Parameter::set("/params/other/threadrosout", true);
}