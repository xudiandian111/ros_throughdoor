#include "Parameter.h"
#define RESET "\033[0m"
#define RED "\033[31m"
/**
 * Parameter类包含以下成员函数
 * 1.initParamServer（） 初始化函数服务器(需要调整的参数除外),每次起飞会运行该函数
 * 2.get()对ros::param::get()的二次封装，使用方法不变，返回值由bool变为参数的值
 * 3.getDouble() getBool()getInt(),直接返回相应类型参数的值，使用方法getDouble("paramname")
 * 4.set()对ros::param::set()的二次封装，使用方法不变，额外支持自增自减运算符使用方法set("paramname","++")
 * 5.setMore()实现多个参数的批量设定
 * 6.getMore()实现多个参数的批量获取
 * **/
Parameter::Parameter(void)
{
}

Parameter::~Parameter(void)
{
}

void Parameter::initParamServer()
{
    ros::param::set("turn_time", 0);
    ros::param::set("turn_left", 0);
    ros::param::set("turn_right", 0);
    ros::param::set("turn", 0);

    ros::param::set("straight_time", 0);
    ros::param::set("fly_time", 0);
    ros::param::set("unusual_dis_count", 0);

    ros::param::set("pre_high", 0.0);
    ros::param::set("aim_high", 0.7);

    ros::param::set("pre_dis", 0.0);
    ros::param::set("count_pid_dis", 0.0);
    ros::param::set("aim_dis", 0.0);

    ros::param::set("h_dis", 0.0);
    ros::param::set("h_pre_dis", 0.0);

    ros::param::set("pre_angle", 0.0);
    ros::param::set("pre_pid_angle", 0.0);
    ros::param::set("count_pid_angle", 0.0);

    ros::param::set("b_dif", 0.0);
    ros::param::set("b_dis", 0.0);

    ros::param::set("speed_x", 0.0);
    ros::param::set("speed_y", 0.0);
    ros::param::set("speed_z", 0.0);
    ros::param::set("angle_z", 0.0);

    ros::param::set("turn_left_flag", false);
    ros::param::set("turn_right_flag", false);
    ros::param::set("after_turn_flag", false);
    ros::param::set("is_cross", false);
    ros::param::set("cross_v_up_flag", false);
    ros::param::set("land_flag", false);

    ros::param::set("ardrone_speed", 0.3);
    ros::param::set("down_flag", false);

    /*Detection param*/
    ros::param::set("land_point_x", 0.0);
    ros::param::set("land_point_y", 0.0);
    ros::param::set("find_land", 0.0);
    ros::param::set("flag_line", 0.0);
    ros::param::set("aim_angle", 0.0);
    ros::param::set("aim_dis", 0.0);

    /*debug*/
    ros::param::set("pid_time", 0);

    /** 以下初始化为重构初始化 将会逐步取代上面的东西 
     * 分区规则 类
     * **/
    
    /* AutoPid */
}

bool Parameter::getBool(const std::string key)
{
    bool v;
    if (ros::param::get(key, v))
    {
        return v;
    }
    else
    {
        ROS_ERROR("%s is not exist!",key.c_str());
        return false;
    }
}
double Parameter::getDouble(const std::string key)
{
    double v;
    if(ros::param::get(key,v))
    {
        return v;
    }
    else
    {
        ROS_ERROR("%s is not exist!",key.c_str());
        return 0;
    }
}
int Parameter::getInt(const std::string key)
{
    int v;
    if (ros::param::get(key, v))
    {
        return v;
    }
    else
    {
        ROS_ERROR("%s is not exist!",key.c_str());
        return 0;
    }
}
std::string Parameter::getString(const std::string key)
{
    std::string v;
    if(ros::param::get(key,v))
    {
        return v;
    }
    else
    {
        ROS_ERROR("%s is not exist!", key.c_str());
        return 0;
    }
}
void Parameter::setInt(const std::string key, const std::string v)
{
    int value;
    if (ros::param::get(key, value))
    {
    
        if (v == "++")
        {
            value++;
//            ROS_ERROR("++%d", value);
            ros::param::set(key, value);
        }
        else if (v == "--")
        {
            value--;
            ros::param::set(key, value);
        }
        else
        {
            ROS_ERROR("%s:Unsupported formats %s", key.c_str(), v.c_str());
        }
    }
    else
    {
        ROS_ERROR("%s is not exist!",key.c_str());
    }
}

template <typename T>
void Parameter::setAll(std::string key, T value)
{
    std::map<std::string, std::string> map;
    if (key == "bool")
    {
        ros::param::get("/init/bool", map);
    }
    else if(key=="double")
    {
        ros::param::get("/init/double", map);
    }
    else if(key=="int")
    {
        ros::param::get("/init/int", map);
    }
    else 
    {
        return;
    }

    for (std::map<std::string, std::string>::iterator iter = map.begin(); iter != map.end(); iter++)
    {
        if (ros::param::has(iter->second))
        {
            ros::param::set(iter->second, value);
        }
        else
        {
            ROS_ERROR("init_setall_error: %s is not exist!", iter->second.c_str());
        }
    }
}
void Parameter::initParam()
{
    setAll("bool", false);
    setAll("double", 0.0);
    setAll("int", 0);
}
