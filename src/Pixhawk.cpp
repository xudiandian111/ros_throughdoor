#include "Pixhawk.h"
#define keytest
/**
 * mode 参数解释
 * mode分为三个模式
 * land 模式设置为AUTO.LAND
 * fly 模式设置为定高飞行
 * auto 模式设置为自动飞行
 * 每次land和关闭auto会记录降落xy位置
 * 可通过修改参数服务器中的 /point/pose来进行定点飞行
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
    Parameter::set("/line/angle/x", msg->data);
}
void Pixhawk::get_dis(const std_msgs::Float64::ConstPtr &msg)
{
    Parameter::set("/line/point/y", msg->data / 100.0);
}
void Pixhawk::get_flag_line(const std_msgs::Bool::ConstPtr &msg)
{
    Parameter::set("/line/flag", msg->data);
}
void Pixhawk::get_flag_door(const std_msgs::Bool::ConstPtr &msg)
{
    Parameter::set("/door/flag", msg->data);
}
void Pixhawk::get_point_door(const geometry_msgs::Point::ConstPtr &msg)
{
    Parameter::set<double>({{"/door/point/x", msg->x / 100.0},
                            {"/door/point/y", msg->y / 100.0},
                            {"/door/point/z", msg->z / 100.0}});
}
void Pixhawk::get_flag_land(const std_msgs::Bool::ConstPtr &msg)
{
    Parameter::set("/land/flag", msg->data);
}
void Pixhawk::get_point_land(const geometry_msgs::Point::ConstPtr &msg)
{
    Parameter::set<double>({{"/land/point/x", msg->x / 100.0},
                            {"/land/point/y", msg->y / 100.0}});
}
void Pixhawk::get_flag_QR(const std_msgs::Bool::ConstPtr &msg)
{
    Parameter::set("/QR/flag", msg->data);
}
void Pixhawk::get_info_QR(const std_msgs::String::ConstPtr &msg)
{
    std::stringstream ss(msg->data);
    double data[3] = {0, 0, 0};
    for (int i = 0; i < 3; i++)
    {
        if(ss>>data[i])
        {
        }
        else
        {
            break;
        }
    }
    Parameter::set<double>({{"/QR/info/order", data[0]},
                            {"/QR/info/high", data[1] / 100.0},
                            {"/QR/info/direction", data[2]}});
}
void Pixhawk::state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

void Pixhawk::readPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    double w, x, y, z, yaw;
    w = msg->pose.orientation.w;
    x = msg->pose.orientation.x;
    y = msg->pose.orientation.y;
    z = msg->pose.orientation.z;
    yaw = atan(4 * (w * x + y * z) / (1 - 2 * (y * y + z * z)));
    Parameter::set<double>({{"/state/pose/x", msg->pose.position.x},
                            {"/state/pose/y", msg->pose.position.y},
                            {"/state/pose/z", msg->pose.position.z},
                            {"/state/angle/z", yaw}});
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

    flag_land_sub = nh.subscribe<std_msgs::Bool>("/DetectLand/flag_land", 2, Pixhawk::get_flag_land);
    point_land_sub = nh.subscribe<geometry_msgs::Point>("/Land/center", 2, Pixhawk::get_point_land);

    flag_door_sub = nh.subscribe<std_msgs::Bool>("/Door/flag_door", 2, Pixhawk::get_flag_door);
    point_door_sub = nh.subscribe<geometry_msgs::Point>("/Door/center", 2, Pixhawk::get_point_door);

    flag_QR_sub = nh.subscribe<std_msgs::Bool>("/ScanQR/QR_inform", 2, Pixhawk::get_flag_QR);
    info_QR_sub = nh.subscribe<std_msgs::String>("/ScanQR/QR_flag", 2, Pixhawk::get_info_QR);

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
/*******************************以下无需修改**********************************************/
        if( mode == "land" &&
            current_state.mode != "AUTO.LAND" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_client.call(land_set_mode)&&
                land_set_mode.response.mode_sent)
            {
                ROS_INFO("Autoland enabled");
                time = 0;
            }
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
/*******************************以上无需修改**********************************************/
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
        if (mode == "key")
        {
            Parameter::set<double>({{"/control/speed/x", 0.0},
                                    {"/control/speed/y", 0.0},
                                    {"/control/speed/z", 0.0},
                                    {"/control/angle/z", 0.0}});
        }
        time++;
        ros::spinOnce();
        rate.sleep();
    }
    Parameter::set("/params/other/threadrosout", true);
}