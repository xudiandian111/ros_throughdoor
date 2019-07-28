#include "Control.h"
#define Debfug
Control::Control()
{
}
Control::~Control()
{
}
void Control::controlLine(double cmd[][3])
{
    double angle_z_error, pose_y_error, pose_z, point_z, pose_z_error;
    Parameter::get<double>({{"/line/angle/z", &angle_z_error},
                            {"/line/point/y", &pose_y_error},
                            {"/state/pose/z", &pose_z},
                            {"/point/pose/z", &point_z}});  
    pose_z_error = point_z - pose_z;
    cmd[0][0] = Parameter::getDouble("/state/speed/normal");
    cmd[0][1] = pid.normalPid(pose_y_error, "pose_y");
    cmd[0][2] = pid.normalPid(pose_z_error, "pose_z");
    cmd[1][2] = pid.normalPid(angle_z_error, "angle_z");
}
void Control::controlLand(double cmd[][3])
{
    double angle_z_error, pose_x_error, pose_y_error, pose_z, point_z, pose_z_error;
    Parameter::get<double>({{"/land/point/x", &pose_x_error},
                            {"/land/point/y", &pose_y_error},
                            {"/state/pose/z", &pose_z},
                            {"/point/pose/z", &point_z},
                            {"/line/angle/z", &angle_z_error}});
    pose_z_error = point_z - pose_z;
    if (pose_x_error < 1 && pose_y_error < 1)
    {
        Parameter::set("/state/mode", "land");
    }
    else
    {
        cmd[0][0] = pid.normalPid(pose_x_error, "pose_x");
        cmd[0][1] = pid.normalPid(pose_y_error, "pose_y");
        cmd[0][2] = pid.normalPid(pose_z_error, "pose_z");
        cmd[1][2] = pid.normalPid(angle_z_error, "angle_z");
    }
}
void Control::controlDoor(double cmd[][3])
{
    double angle_z_error, pose_y_error, pose_z, point_z, pose_z_error;
    if (Parameter::getBool("/QR/flag") == true) //水平门
    {
        Parameter::get<double>({{"/QR/info/high", &point_z},
                                {"/state/pose/z", &pose_z},
                                {"/control/point/y", &pose_y_error},
                                {"/control/angle/z", &angle_z_error}});
        //需要QR的坐标 飞到QR上空 然后前进
        //定义 /QR/info/point_z为门的中心高度
        if (Parameter::getBool("/QR/info/direction") == 1) //向上
        {
            if(Parameter::getBool("/QR/info/center")==true)
            {
                double pose_x, pose_y;
                Parameter::get<double>({{"/state/pose/x", &pose_x},
                                        {"/state/pose/y", &pose_y}});
                //进入特殊控制 设置任务 doorup 设置步骤1 记住当前xyz 坐标
                Parameter::set("/control/task/name", "doorup");
                Parameter::set("/control/task/step", 1);
                Parameter::set<double>({{"/control/task/x", pose_x},
                                        {"/control/task/y", pose_y},
                                        {"/control/task/z", point_z - 0.3}});
            }
            pose_z_error = point_z - pose_z - 0.3;
        }
        else if (Parameter::getBool("/QR/info/direction") == 2) //向下
        {
            if (Parameter::getBool("/QR/info/center") == true)
            {
                double pose_x, pose_y;
                Parameter::get<double>({{"/state/pose/x", &pose_x},
                                        {"/state/pose/y", &pose_y}});
                //进入特殊控制 设置任务 doorup 设置步骤1 记住当前xyz 坐标
                Parameter::set("/control/task/name", "doordown");
                Parameter::set("/control/task/step", 1);
                Parameter::set<double>({{"/control/task/x", pose_x},
                                        {"/control/task/y", pose_y},
                                        {"/control/task/z", point_z + 0.4}});
            }
            pose_z_error = point_z + 0.4 - pose_z;
        }
        // else if (Parameter::getBool("/QR/info/direction") == 3)
        // {
        //     pose_z_error = point_z + 0.4 - pose_z;
        // }
        // if(0)
        // {
        //     Parameter::set("/control/task/name", "door");
        // }
        cmd[0][0] = Parameter::getDouble("/state/speed/normal");
        cmd[0][1] = pid.normalPid(pose_y_error, "pose_y");
        cmd[0][2] = pid.normalPid(pose_z_error, "pose_z");
        cmd[1][2] = pid.normalPid(angle_z_error, "angle_z");
    }
    else  //竖直门
    {
        //目前使用的是线的y坐标 竖直门高度就行了
        Parameter::get<double>({{"/door/line/y", &pose_y_error},
                                {"/state/pose/z", &pose_z},
                                {"/door/point/z", &point_z},
                                {"/line/angle/z", &angle_z_error}});
        pose_z_error = point_z - pose_z;
        if (fabs(pose_z_error) < 0.05)
        {
            //高度误差小于5厘米开始前进
            //进入特殊控制（确保穿过门）
            cmd[0][0] = 0.3;
            Parameter::set("/control/task/last", "doorto");
            Parameter::set("/control/task/step", 1);
            Parameter::set("/control/task/z", point_z);
        }
        else
        {
            //误差过大 减慢速度
            cmd[0][0] = 0.1;
        }
        cmd[0][1] = pid.normalPid(pose_y_error, "pose_y");
        cmd[0][2] = pid.normalPid(pose_z_error, "pose_z");
        cmd[1][2] = pid.normalPid(angle_z_error, "angle_z");
    }
}
void Control::controlDoorUp(double cmd[][3], int step)
{
    //向上穿门完成
    double point_x, point_y, point_z,
        pose_x, pose_y, pose_z,
        angle_z_error, pose_y_error, pose_z_error, pose_x_error,
        dis, speed_x;

    Parameter::get<double>({{"/line/angle/z", &angle_z_error},
                            {"/line/point/y", &pose_y_error},
                            {"/state/pose/z", &pose_z},
                            {"/state/pose/x", &pose_x},
                            {"/state/pose/y", &pose_y},
                            {"/control/task/z", &point_z},
                            {"/control/task/x", &point_x},
                            {"/control/task/y", &point_y}});
    //此处得到的point是二维码的坐标 结算距离
    dis = sqrt((pose_x - point_x) * (pose_x - point_x) + (pose_y - point_y) * (pose_y - point_y));
    speed_x = Parameter::getDouble("/state/speed/normal");
    if (step == 1)
    {
        if (fabs(dis - 0.5) < 0.05)
        {
            //开始上升 目标高度变动
            Parameter::get("/QR/info/high", point_z);
            //记住当前的x y 坐标
            Parameter::set<double>({{"/control/task/z", point_z + 0.4},
                                    {"/control/task/x", pose_x},
                                    {"/control/task/y", pose_y}});
            //设置开始进入第二个步骤 开始控制x速度
            Parameter::set("/control/task/step", 2);
            pose_y_error = 0;
            pose_x_error = 0;
            speed_x = 0;
        }
        else
        {
            //正常的巡线 无需任何操作
        }
        
    }
    else if(step==2)
    {
        //当高度到达指定的值的时候
        if((pose_z - point_z) > 0)
        {
            //记住当前的x y坐标 开始正常巡线飞行
            Parameter::set<double>({{"/control/task/x", pose_x},
                                    {"/control/task/y", pose_y}});
            Parameter::set("/control/task/step", 3);
        }
        else
        {
            //进入上升阶段 控制x速度 y x重新解算
            pose_x_error = point_x - pose_x;
            speed_x = pid.normalPid(pose_x_error, "pose_x");
            pose_y_error = point_y - pose_y;
        }
    }
    else if(step==3)
    {
        //巡线飞行 无需干扰
        //距离大于0.5进入正常飞行模式
        if (dis > 0.5)
        {
            Parameter::set("/control/task/name", "auto");
            Parameter::set("/control/task/step", 0);
        }
    }
    pose_z_error = point_z - pose_z;
    cmd[0][0] = speed_x;
    cmd[0][1] = pid.normalPid(pose_y_error, "pose_y");
    cmd[0][2] = pid.normalPid(pose_z_error, "pose_z");
    cmd[1][2] = pid.normalPid(angle_z_error, "angle_z");
}
void Control::controlDoorDown(double cmd[][3], int step)
{
    //向下穿门完成
    double point_x, point_y, point_z,
        pose_x, pose_y, pose_z,
        angle_z_error, pose_y_error, pose_z_error, pose_x_error,
        dis, speed_x;

    Parameter::get<double>({{"/line/angle/z", &angle_z_error},
                            {"/line/point/y", &pose_y_error},
                            {"/state/pose/z", &pose_z},
                            {"/state/pose/x", &pose_x},
                            {"/state/pose/y", &pose_y},
                            {"/control/task/z", &point_z},
                            {"/control/task/x", &point_x},
                            {"/control/task/y", &point_y}});
    //此处得到的point是二维码的坐标 结算距离
    dis = sqrt((pose_x - point_x) * (pose_x - point_x) + (pose_y - point_y) * (pose_y - point_y));
    speed_x = Parameter::getDouble("/state/speed/normal");
    if (step == 1)
    {
        if (fabs(dis - 0.5) < 0.05)
        {
            //开始下降 目标高度变动
            Parameter::get("/QR/info/high", point_z);
            Parameter::set("/control/task/z", point_z - 0.3);
            //设置开始进入第二个步骤 开始控制x速度
            Parameter::set("/control/task/step", 2);
            pose_y_error = 0;
            pose_x_error = 0;
            speed_x = 0;
        }
        else
        {
            //正常的巡线 无需任何操作
        }
        
    }
    else if(step==2)
    {
        //当高度到达指定的值的时候
        if((pose_z - point_z) < 0)
        {
            //记住当前的x y坐标 开始正常巡线飞行
            Parameter::set<double>({{"/control/task/x", pose_x},
                                    {"/control/task/y", pose_y}});
            Parameter::set("/control/task/step", 3);
        }
        else
        {
            //下降阶段 控制x速度 y x重新解算
            //若门的信息可信 则使用门的信息 若不可信则盲飞
            if (Parameter::getDouble("/door/flag") == true)
            {
                Parameter::get<double>({{"/door/point/x", &pose_x_error},
                                        {"/door/point/y", &pose_y_error}});
            }
            else
            {
                pose_x_error = point_x - pose_x;
                speed_x = pid.normalPid(pose_x_error, "pose_x");
                pose_y_error = point_y - pose_y;
            }
        }
    }
    else if(step==3)
    {
        //巡线飞行 无需干扰
        //距离大于0.5进入正常飞行模式
        if (dis > 0.5)
        {
            Parameter::set("/control/task/name", "auto");
            Parameter::set("/control/task/step", 0);
        }
    }
    pose_z_error = point_z - pose_z;
    cmd[0][0] = speed_x;
    cmd[0][1] = pid.normalPid(pose_y_error, "pose_y");
    cmd[0][2] = pid.normalPid(pose_z_error, "pose_z");
    cmd[1][2] = pid.normalPid(angle_z_error, "angle_z");
}
void Control::controlDoorTo(double cmd[][3], int step)
{
    //竖直门的特殊控制
    double pose_y_error, pose_z, pose_x, pose_y, point_z, angle_z_error, pose_z_error;
    if (Parameter::getBool("/door/flag") == true)
    {
        Parameter::get<double>({{"/line/point/y", &pose_y_error},
                                {"/state/pose/z", &pose_z},
                                {"/door/point/z", &point_z},
                                {"/line/angle/z", &angle_z_error},
                                {"/state/pose/x", &pose_x},
                                {"/state/pose/y", &pose_y}});
        pose_z_error = point_z - pose_z;
        if (fabs(pose_z_error) < 0.05)
        {
            //高度误差小于5厘米开始前进
            //进入特殊控制（确保穿过门）
            cmd[0][0] = 0.3;
        }
        else
        {
            //误差过大 减慢速度
            cmd[0][0] = 0.2;
        }
        cmd[0][1] = pid.normalPid(pose_y_error, "pose_y");
        cmd[0][2] = pid.normalPid(pose_z_error, "pose_z");
        cmd[1][2] = pid.normalPid(angle_z_error, "angle_z");
        //每次记录xy坐标 为丢门之后做准备
        Parameter::set<double>({{"/control/task/x", pose_x},
                                {"/control/task/y", pose_y},
                                {"/control/task/z", pose_z}});
    }
    else
    {
        //此时门的位置已经lost！盲飞0.5米结束穿门
        double point_x, point_y, dis;
        Parameter::get<double>({{"/line/point/y", &pose_y_error},
                                {"/state/pose/z", &pose_z},
                                {"/control/task/z", &point_z},
                                {"/line/angle/z", &angle_z_error},
                                {"/state/pose/x", &pose_x},
                                {"/state/pose/y", &pose_y},
                                {"/control/task/x", &point_x},
                                {"/control/task/y", &point_y}});
        dis = sqrt((pose_x - point_x) * (pose_x - point_x) + (pose_y - point_y) * (pose_y - point_y));
        if (dis > 0.5)
        {
            //结束任务 进入巡线
            Parameter::set("/control/task/name", "auto");
            Parameter::set("/control/task/step", 0);
        }
        pose_z_error = point_z - pose_z;
        cmd[0][0] = Parameter::getDouble("/state/speed/normal");
        cmd[0][1] = pid.normalPid(pose_y_error, "pose_y");
        cmd[0][2] = pid.normalPid(pose_z_error, "pose_z");
        cmd[1][2] = pid.normalPid(angle_z_error, "angle_z");
    }
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
        std::string task;
        Parameter::get("/control/task/name", task);
        if (task != "auto") //意味着无人机进入穿门的特殊控制阶段
        {
            int step;
            Parameter::get("/control/task/step", step);
            if(task=="doorup")
            {
                controlDoorUp(cmd, step);
            }
            else if (task == "doordown")
            {
                controlDoorDown(cmd, step);
            }
            else if(task == "doorto")
            {
                controlDoorTo(cmd, step);
            }
        }
        else if (Parameter::getBool("/land/flag") == true)
        {
            controlLand(cmd);
            #ifdef Debug
                ROS_ERROR("land");
            #endif
        }
        else if (Parameter::getBool("/door/flag") == true || Parameter::getBool("/QR/flag") == true) //这玩意是只有竖直门才一直有
        {
            controlDoor(cmd);
        }
        else
        {
            controlLine(cmd);
            #ifdef Debug
                ROS_ERROR("line");
            #endif
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
#ifdef Debug
    Parameter::debug();
#endif
}
