#include "Parameter.h"
#define RESET "\033[0m"
#define RED "\033[31m"
/**
 * Parameter类包含以下成员函数
 * 1.get()对ros::param::get()的二次封装，使用方法不变,map类型变为传递参数的地址
 * 2.getDouble() getBool()getInt(),直接返回相应类型参数的值，使用方法getDouble("paramname")
 * 3.set()对ros::param::set()的二次封装，使用方法不变
 * **/
Parameter::Parameter(void)
{
}

Parameter::~Parameter(void)
{
}
void Parameter::initParam()
{
    setAll("bool", false);
    setAll("double", 0.0);
    setAll("int", 0);
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
void Parameter::debug()
{
    std::map<std::string, std::string> map;
    ros::param::get("/debug", map);
    for (std::map<std::string, std::string>::iterator iter = map.begin(); iter != map.end(); iter++)
    {
        double v;
        if(ros::param::get(iter->second, v))
        {
            ROS_INFO("%s is %lf", iter->second.c_str(), v);
        }
    }
}