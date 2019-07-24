#pragma once
#include <ros/ros.h>
//#include <std_msgs/String.h>
class Parameter
{
public:
    Parameter(void);
    ~Parameter(void);

public:
    /*init*/
    static void initParamServer();
    template <typename T>
    static bool get(const std::string key, T &value)
    {
        if(ros::param::get(key,value))
        {
            return true;
        }
        else
        {
            ROS_ERROR("get_error: %s is not exist!",key.c_str());
            value = 0;
            return false;
        }
    }
    static double getDouble(const std::string key);
    static int getInt(const std::string key);
    static bool getBool(const std::string key);
    static std::string getString(const std::string key);

    static void setInt(const std::string key, const std::string v);

    template <typename T>
    static void set(const std::string key, T value)
    {
        ros::param::set(key, value);
    }

    static void add(const std::string key);
    static void sub(const std::string key);
/*new*/
    static void initParam();
    template <typename T>
    static void set(std::map<std::string, T> map)
    {
        for (typename std::map<std::string, T>::iterator iter = map.begin(); iter != map.end(); iter++)
        {
            if (ros::param::has(iter->first))
            {
                ros::param::set(iter->first, iter->second);
            }
            else
            {
                ROS_ERROR("set_map_error: %s is not exist!", iter->first.c_str());
            }
        }
    }

    template <typename T>
    static void get(std::map<std::string, T *> map)
    {
        for (typename std::map<std::string, T *>::iterator iter = map.begin(); iter != map.end(); iter++)
        {
            T value;
            if (ros::param::get(iter->first, value))
            {
                *iter->second = value;
            }
            else
            {
                *iter->second = 0;
                ROS_ERROR("getMore: %s is not exist!", iter->first.c_str());
            }
        }
    }
private:
    template <typename T>
    static void setAll(std::string key, T value);
};