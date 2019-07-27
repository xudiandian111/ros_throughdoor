#include "Control.h"
Control::Control()
{
}
Control::~Control()
{
}
void Control::controlCheck()
{
    
}
void Control::controlMain(std::string mode)
{
    double cmd[2][3] = {0, 0, 0, 0, 0, 0};
    
    if(mode == "fly")
    {
        double pose_z, pose_x, pose_y,
            pose_x_error, pose_y_error, pose_z_error,
            point_x, point_y, point_z;
        Parameter::get<double>({{"/state/pose/x", &pose_x},
                                {"/state/pose/y", &pose_y},
                                {"/state/pose/z", &pose_z},
                                {"/point/pose/x", &point_x},
                                {"/point/pose/y", &point_y},
                                {"/point/pose/z", &point_z}});
        pose_x_error = point_x - pose_x;
        pose_y_error = point_y - pose_y;
        pose_z_error = point_z - pose_z;
        cmd[0][0] = pid.normalPid(pose_x_error, "pose_x");
        cmd[0][1] = pid.normalPid(pose_y_error, "pose_y");
        cmd[0][2] = pid.normalPid(pose_z_error, "pose_z");
    }
    else if(mode == "auto")
    {
        double pose_z, point_z, angle_z_error, pose_x_error, pose_y_error, pose_z_error;
        if (Parameter::getBool("/land/flag") == true)
        {
            Parameter::get<double>({{"/land/point/x", &pose_x_error},
                                    {"/land/point/y", &pose_y_error},
                                    {"/state/pose/z", &pose_z},
                                    {"/point/pose/z", &point_z},
                                    {"/line/angle/z", &angle_z_error}});
            pose_z_error = point_z - pose_z;
            cmd[0][0] = pid.normalPid(pose_x_error, "pose_x") / 100.0;
            cmd[0][1] = pid.normalPid(pose_y_error, "pose_y") / 100.0;
            cmd[0][2] = pid.normalPid(pose_z_error, "pose_z");
            cmd[1][2] = pid.normalPid(angle_z_error, "angle_z");
        }
        else if (Parameter::getBool("/door/flag") == true)
        {
            if(1)//竖直门
            {
                Parameter::get<double>({{"/door/point/y", &pose_y_error},
                                        {"/state/pose/z", &pose_z},
                                        {"/door/point/z", &point_z},
                                        {"/line/angle/z", &angle_z_error}});
                pose_z_error = point_z - pose_z;
                cmd[0][0] = 0;
                cmd[0][1] = pid.normalPid(pose_y_error, "pose_y")/100;
                cmd[0][2] = pid.normalPid(pose_z_error, "pose_z");
                cmd[1][2] = pid.normalPid(angle_z_error, "angle_z");
            }
            else if(Parameter::getBool("/QR/flag")==true)//竖直门
            {
                Parameter::get<double>({{"/QR/info/high", &point_z},
                                        {"/state/pose/z", &pose_z},
                                        {"/control/point/y", &pose_y_error},
                                        {"/control/angle/z", &angle_z_error}});
                if (Parameter::getBool("/QR/info/direction") == 1) //向上
                {
                    pose_z_error = point_z/100.0 - pose_z - 0.3;
                }
                else if (Parameter::getBool("/QR/info/direction") == 2) //向下
                {
                    pose_z_error = point_z / 100.0 + 0.4 - pose_z;
                }
                else if (Parameter::getBool("/QR/info/direction") == 3)
                {
                    pose_z_error = point_z / 100.0 + 0.4 - pose_z;
                }
                cmd[0][0] = 0;
                cmd[0][1] = pid.normalPid(pose_y_error, "pose_y") / 100;
                cmd[0][2] = pid.normalPid(pose_z_error, "pose_z");
                cmd[1][2] = pid.normalPid(angle_z_error, "angle_z");
            }
        }
        else
        {
            ROS_ERROR("line mode");
            Parameter::get<double>({{"/line/angle/z", &angle_z_error},
                                    {"/line/point/y", &pose_y_error},
                                    {"/state/pose/z", &pose_z},
                                    {"/point/pose/z", &point_z}});
            pose_z_error = point_z - pose_z;
            cmd[0][0] = 0;
            cmd[0][1] = pid.normalPid(pose_y_error, "pose_y") / 100.0;
            cmd[0][2] = pid.normalPid(pose_z_error, "pose_z");
            cmd[1][2] = pid.normalPid(angle_z_error, "angle_z");
        }
    }
    else if (mode == "key")
    {
        double pose_z_error, point_z, pose_z;
        Parameter::get<double>({{"/state/pose/z", &pose_z},
                                {"/point/pose/z", &point_z}});
        pose_z_error = point_z - pose_z;
        cmd[0][2] = pid.normalPid(pose_z_error, "pose_z");
    }
    if(mode == "key")
    {
        Parameter::set("/control/speed/z", cmd[0][2]);
    }
    else
    {
        Parameter::set<double>({{"/control/speed/x", cmd[0][0]},
                                {"/control/speed/y", cmd[0][1]},
                                {"/control/speed/z", cmd[0][2]},
                                {"/control/angle/z", cmd[1][2]}});
    }
#ifdef DEBUG
    Parameter::debug();
#endif
}
