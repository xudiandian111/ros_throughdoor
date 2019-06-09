/**
 * 包含对Bebop的控制层
 * 控制方案一：依赖完全图像
 * 控制方案二：依赖无人机罗盘信息
 * 涉及函数judgeAngle()和pidAngleData()
 * **/
#include "Control.h"

double Control::aim_angle = 17.9;
int Control::camera_angle = -48;
int Control::angle_change_count = 0;
double Control::judge_desire_angle = 17.9;

double Control::pre_high = 0;
double Control::aim_high = 0.7;

double Control::pre_dis = 0.0;
int Control::squar_count = 0;
double Control::pre_angle = 0.0;
double Control::special_pre_angle = 0;
double Control::turn_times = 10;
double Control::b_dif = 0;
double Control::b_dis=0;

Control::Control(void)
{
}
Control::~Control(void)
{
}
//此方法为根据实际场地进行测算 保留为参考函数
/*double Control::judgeAngle()
{
    //此函数内进行对传入角度的结算
    //原有控制代码未进行角度获取
    double current_angle = 180 / 3.135905 * Bebop::ardrone_z_attitude; //当前偏航角
    //全局坐标系转换 角度控制 
    if (current_angle < 0)
    {
        current_angle = current_angle + 360;
    }

    if (Detections::ready_for_turn == true)
    {
        switch (angle_change_count)
        {
        case 0:
            judge_desire_angle = 110.0;
            aim_high = 0.7;
            camera_angle = -50; //50
            break;
        case 1:
            judge_desire_angle = -175.5;
            aim_high = 1.3;
            camera_angle = -41;
            break;
        case 2:
            judge_desire_angle = -75.8;
            aim_high = 1.3;
            camera_angle = -60;
            break;
        case 3:
            judge_desire_angle = 9.0;
            aim_high = 1.3;
            camera_angle = -48; //49
            break;
        case 4:
            judge_desire_angle = 110.0;
            aim_high = 0.7;     //0.8
            camera_angle = -42; //39
            squar_count++;
            if (squar_count == 2)
            {
                camera_angle = -52;
            }
            Common::cross_time = 0;
            angle_change_count = 0;
            break;
        }
    }

    if (Detections::ready_for_cross == true)
    {
        judge_desire_angle = -164.0;
        aim_high = 0.7;
        camera_angle = -45;
        Common::land_wait_time = 0;
    }

    if (aim_angle != judge_desire_angle)
    {
        aim_angle = judge_desire_angle;
        angle_change_count++;
        Detections::turn_flag = false;
        Detections::ready_for_turn = false;
        Common::play_time = 0;
    }

    return judge_desire_angle;
}
*/

double Control::pidAngleData() /**角度控制**/
{
    double d_angle;
    double pid_angle;
    double erro_angle;
    double kp_rot = Parameter::angle_kp;
    double kd_rot = Parameter::angle_kd;
/*    double desire_angle;
    if (Parameter::turn_flag == true)
    {
        desire_angle = 90;
    }
    else
    {
        desire_angle = 0;
    }
        
    double desire_angle = 0;
    //double desire_angle = FindLines::angle;
    double ardrone_angle = 180 / 3.135905 * Bebop::ardrone_z_attitude;

    if (ardrone_angle < 0)
    {
        ardrone_angle = ardrone_angle + 360;
    }
*/
    erro_angle = 180 / 3.1415926 * Detections::aim_angle;

    if (erro_angle > 180) //坐标系角度转换
    {
        erro_angle = 360 - erro_angle;
    }
    if (erro_angle < -180)
    {
        erro_angle = 360 + erro_angle;
    }

    d_angle = erro_angle - Control::pre_angle;
    std::cout << erro_angle<<"___" <<Control::pre_angle<<"___"<< d_angle<< std::endl;
    //为防止出现偶尔的角度误判
    pid_angle = kp_rot * erro_angle + kd_rot * d_angle;
    if(fabs(d_angle)>80){
        pid_angle = pid_angle*0.4;
    }

    Control::pre_angle = erro_angle;
    
    return pid_angle;
}

double Control::pidDisData() //左右距离控制
{
    double d_dis = 0, pid_dis, kd_dis, kp_dis;
    kp_dis = Parameter::dis_kp;
    kd_dis = Parameter::dis_kd;
    if (Detections::flag_circle == true && Parameter::is_cross == false)
    {
        double D;
        double dis_kp = 0.18, dis_kd = 0.05;//0.18 0.05
        double p = Detections::circle_point.x - 320;
        p=-p;
        D=Control::b_dis;
        pid_dis=(dis_kp*p+dis_kd*D) / 100.0;
        while(fabs(pid_dis)<0.01&&fabs(pid_dis)>0)
        {
            pid_dis = pid_dis * 10;
            std::cout << "while";
        }
        Control::b_dis = p;
        std::cout << "dis" << pid_dis << "p" << dis_kp * p << "d" << dis_kd * D << std::endl;
        //pre_error_dis=aim_dis;
//        ROS_INFO_STREAM("pid_dis:  "<<pid_dis);
    }
    else
    {
        d_dis = Detections::aim_dis - pre_dis;
        pid_dis = kp_dis * Detections::aim_dis / 100.0 + kd_dis * d_dis / 100.0;
        if(pid_dis>0.02&&pid_dis<0.15)
        {
            pid_dis = 0.15;
        }
        else if(pid_dis<-0.02&&pid_dis>-0.15)
        {
            pid_dis = -0.15;
        }
        pre_dis = Detections::aim_dis;
        std::cout<<"dis"<<pid_dis<<"p"<<kp_dis * Detections::aim_dis/100.0<<"d"<<kd_dis * d_dis / 100.0<<std::endl;
    }
    if (pid_dis > 0.3)
    {
        pid_dis = 0.3;
    }
    if(pid_dis < -0.3)
    {
        pid_dis = -0.3;
    }
    return pid_dis;
}

double Control::pidHighData() //高度的PID控制
{
    double d_high = 0, pid_high, erro_high;
    double kd_high, kp_high;
    kp_high = Parameter::high_kp;
    kd_high = Parameter::high_kd;
    erro_high = aim_high - Bebop::ardrone_height;
    d_high = erro_high - pre_high;
    pid_high = kp_high * erro_high + kd_high * d_high;
    pre_high = erro_high;
    return pid_high;
}

double Control::pidSpecialAngle(double aim_angle, double real_angle)
{
    double d_angle;
    double pid_angle;
    double erro_angle;
    double kp_rot = Parameter::angle_kp;
    double kd_rot = Parameter::angle_kd;
    if (real_angle < 0)
    {
        real_angle = real_angle + 360;
    }
    erro_angle = aim_angle - real_angle;
    if (erro_angle > 180) //坐标系角度转换
    {
        erro_angle = 360 - erro_angle;
    }
    if (erro_angle < -180)
    {
        erro_angle = 360 + erro_angle;
    }
    d_angle = erro_angle - special_pre_angle;
    special_pre_angle = erro_angle;
    pid_angle = kp_rot * erro_angle + kd_rot * d_angle;
    
    return pid_angle;
}

double Control::getSpeedData(double angle, double dis) //速度选择器
{
    double speed;
    if (Detections::flag_circle == true && Parameter::is_cross == false)
    {
        double dif=(Detections::circle_point.y-270) / 70.0;
        dif = -dif;
        double kp = Parameter::speed_kp;
        double kd = Parameter::speed_kd;
        double d_dis = dif - Control::b_dif;
        speed = kp * dif +kd * d_dis;
        Control::b_dif = dif;
        if(speed == 0)
        {
            speed = 0.1;
        }
        while(fabs(speed)<0.1&&fabs(speed)>0)
        {
            speed = speed * 10;
            std::cout << "while";
            if(speed>0.2){
                speed = 0.2;
            }else if(speed<-0.2)
            {
                speed = -0.2;
            }
        }
        if(speed>0.4){
            speed = 0.4;
        }
        else if(speed<-0.4)
        {
            speed = -0.4;
        }
        std::cout << "speed:" << speed << "p:" << kp * dif << "d:" << kd * d_dis << std::endl;
    }
    else
    {
        if(fabs(angle)<0.1)
        {
            speed = Parameter::speed_straight;
        }
        else if(fabs(angle)<0.3)
        {
            speed = Parameter::speed_normal;
        }
        else if(fabs(angle)<0.5)
        {
            speed = Parameter::speed_low;
        }
        else
        {
            speed = Parameter::speed_turn;
        }
    }
    
    return speed;
}

void Control::controlGeneral(double control_parameter[][3])//常规控制
{
    /**
     * 需要的参数 1 飞机偏航角为0 飞机在跑道中央
     * 图像状态数据：直行 转向 降落 钻圈水平 钻圈竖直
     * 图像中需要的数据 1.目标角度或者直接角度差 2.左右理想距离 3.高度信息
     * 特殊控制 需要各种flag 高度信息 角度信息
     * 测试 90度右转
     * 测试 南为0度 东正西负
    **/
    double pid_angle,pid_dis,pid_high,get_speed;

    pid_angle = Control::pidAngleData();//角度控制
    pid_dis = Control::pidDisData();//左右控制
    pid_high = Control::pidHighData();//高度控制
    get_speed = Control::getSpeedData(pid_angle, pid_dis);//速度控制
#ifdef CONTROL
    std::cout << "pid_angle:" << pid_angle << "pid_dis" << pid_dis << "pid_high" << std::endl;
#endif
    control_parameter[0][0] = 0.35;
    control_parameter[0][2] = pid_high;
    control_parameter[0][1] = pid_dis;
    std::cout << "pid_angle:" << pid_angle << "aim_angle"<<Detections::aim_angle<<std::endl;
    control_parameter[1][2] = pid_angle;
}

//vertical and horizontal circle control.
void Control::controlVfindCircle(double control_parameter[][3])
{
    double pid_high, pid_dis, pid_speed;
    pid_speed = Control::getSpeedData(0, 0);
    pid_dis = Control::pidDisData();
    pid_high = Control::pidHighData();
    control_parameter[0][0] = pid_speed;
    control_parameter[0][1] = pid_dis;
    control_parameter[0][2] = pid_high;
}
void Control::controlVdownCircle(double control_parameter[][3])
{
    /**
     * 水平圈的向下钻圈
     * 控制思路
     * 到达上空识别出一部分圈 进行悬停降落
     * x轴速度为0
     * 在丢失圈的图像数据时根据丢失时的高度再降落0.3
     * 在与目标高度差距0.1时开始前进
     * 差距基本为0时停止钻圈 进行直行
     * 需要参数aim_high，flag
     * **/
    double pid_high, pid_angle, pid_dis, get_speed;
    Control::aim_high = 0.7;
    pid_high = Control::pidHighData();//高度控制
    control_parameter[0][2] = pid_high;
}

void Control::controlVupCircle(double control_parameter[][3])
{ 
    /**
     * 水平圈的向上钻圈
     * 控制思路
     * 到达上空识别出一部分圈 进行悬停降落
     * x轴速度为0
     * 在丢失圈的图像数据时根据丢失时的高度再降落0.3
     * 在与目标高度差距小于0.1时开始前进
     * 差距基本为0时停止钻圈 进行直行
     * 需要参数aim_high，flag
     * **/
    double pid_high, error_high, pid_angle, pid_dis, get_speed;
    Control::aim_high = 1.3;
    error_high = Control::aim_high - Bebop::ardrone_height;
    pid_angle = Control::pidAngleData();//角度控制
    pid_dis = Control::pidDisData();//左右控制
    pid_high = Control::pidHighData();//高度控制
    get_speed = Control::getSpeedData(pid_angle, pid_dis);//速度控制
    if (fabs(error_high) < 0.05)
    {
        control_parameter[0][0] = 0.08;
        Parameter::cross_v_up_flag = false;
    }
    control_parameter[0][2] = pid_high;
    std::cout << "error_high:" << error_high << std::endl;
}
double Control::getAimHigh(double high[30])
{
    /**
     * 最佳高度
     * 待测
     * **/

    if(Control::aim_high<1)
    {
        Control::aim_high = 1;
        Control::pidSpecialAngle(90,0);
    }
    else if(Control::aim_high>2)
    {
        Control::aim_high = 2;
    }
    
}
void Control::controlCross(double control_parameter[][3])
{
    /**
     * 竖直圈的钻圈控制
     * 暂时不使用
     * 需要参数flag,aim_high
     * high[]每组数据数
     * **/
    double pid_angle, pid_dis, pid_high, get_speed, error_high;
    Control::aim_high = 1.0;
    pid_angle = Control::pidAngleData();//角度控制
    pid_dis = Control::pidDisData();//左右控制
    pid_high = Control::pidHighData();//高度控制
    get_speed = Control::getSpeedData(pid_angle, pid_dis);//速度控制
    error_high = Control::aim_high - Bebop::ardrone_height;
}
void Control::specialControl(int kind, double control_parameter[][3])
{
    /**
     * 特殊控制
     * 指定角度和高度
     * 1 右转 大转向转两次
     * **/
    double pid_angle, pid_high;
//    Control::aim_high = 1;
        pid_angle = Control::pidSpecialAngle(90, 0);
        pid_high = Control::pidHighData();
        control_parameter[1][2] = pid_angle;
        control_parameter[0][2] = pid_high;
        control_parameter[0][1] = 0.35;
        Parameter::special_flag = false;
}
void Control::controlTurnLeft(double control_parameter[][3])
{
    double pid_angle, pid_dis, pid_high, get_speed, error_high;
    pid_angle = Control::pidAngleData();//角度控制
    pid_dis = Control::pidDisData();//左右控制
    pid_high = Control::pidHighData();//高度控制
    if (Parameter::turn_time < 2)
    {
        control_parameter[1][2] = Control::pidSpecialAngle(90, 0);
        control_parameter[0][1] = 0.35;
        control_parameter[0][0] = 0.45;
    }
    else if (Parameter::turn_time < 5)
    {
        control_parameter[0][0] = 0.3;
        control_parameter[1][2] = Control::pidSpecialAngle(10, 0);
        control_parameter[0][1] = 0.45;
    }
    else if(Parameter::turn_time < 6)
    {
        control_parameter[0][0] = 0.2;
        control_parameter[1][2] = pid_angle;
        control_parameter[0][1] = 0.4;
    }
    else
    {
        Parameter::turn_left_flag = false;
        control_parameter[0][0] = 0.3;
        control_parameter[1][2] = pid_angle;
        control_parameter[0][1] = 0.35;
        Parameter::turn_time = 0;
        Parameter::turn = 0;
    }
    
    Parameter::turn_time++;
}