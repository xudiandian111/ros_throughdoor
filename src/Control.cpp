#include "Control.h"
Control::Control()
{
}
Control::~Control()
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
        cmd[0][0] = pid.neuralPid(pose_x_error, "pose_x");
        cmd[0][1] = pid.neuralPid(pose_y_error, "pose_y");
        cmd[0][2] = pid.normalPid(pose_z_error, "pose_z");
    }
    else if(mode == "auto")
    {
        double pose_z, point_z, angle_z_error, pose_y_error, pose_z_error;
        Parameter::get<double>({{"/auto/angle/z", &angle_z_error},
                                {"/auto/point/y", &pose_y_error},
                                {"/state/pose/z", &pose_z},
                                {"/point/pose/z", &point_z}});
        pose_z_error = point_z - pose_z;
        cmd[0][0] = 0.3;
        cmd[0][1] = pid.neuralPid(pose_y_error, "pose_y") / 100.0;
        cmd[0][2] = pid.normalPid(pose_z_error, "pose_z");
        cmd[1][2] = pid.normalPid(angle_z_error, "angle_z");
    }
    if(mode!="key")
    {
        Parameter::set<double>({{"/control/speed/x", cmd[0][0]},
                                {"/control/speed/y", cmd[0][1]},
                                {"/control/speed/z", cmd[0][2]},
                                {"/control/angle/z", cmd[1][2]}});
    }
}
