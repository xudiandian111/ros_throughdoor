/**
 * 包含对Bebop的控制层
 * 控制方案一：依赖完全图像
 * 控制方案二：依赖无人机罗盘信息
 * 涉及函数judgeAngle()和pidAngleData()
 * dis超过0.35属于失控状态
 * 转向优化 左右转向时需要无人机偏左偏右
 * 目前拟设三种方案解决无人机转向
 * 第一特殊转向 目前测试左右转向会出现误判（图像出现误判）
 * 第二角度控制 图像提前告知左转右转 让无人机先偏离一定角度
 * 第三距离控制 图像提前告知左转右转 让无人机提前靠边行驶
 * 图像数据 提前告知下一个弯道左转还是右转
 * **/
#include "Control.h"

double Control::aim_angle = 17.9;
int Control::camera_angle = -48;
int Control::angle_change_count = 0;
double Control::judge_desire_angle = 17.9;

double Control::pre_high = 0;
double Control::aim_high = 0.7;

double Control::pre_dis = 0.0;
double Control::count_pid_dis = 0.0;
double Control::aim_dis = 0.0;

double Control::pre_angle = 0.0;
double Control::pre_pid_angle = 0.0;
double Control::count_pid_angle = 0.0;

double Control::special_pre_angle = 0;
double Control::turn_times = 10;
double Control::b_dif = 0;
double Control::b_dis=0;
queue<double> Control::angle_data;

Control::Control(void)
{
}
Control::~Control(void)
{
}

void Control::checkStatus() //新的方法下此方法将会被废除
{
    double erro_angle = 180 / 3.1415926 * Detections::aim_angle;
    if(erro_angle>30)
    {
        if(Parameter::turn_left>1)
        {
            Parameter::turn_left_flag = true;
            Parameter::turn_time = 0;
            Parameter::turn_left = 0;
            Control::aim_dis = 0;
        }
        else
        {
            Parameter::turn_left++;
            Control::aim_dis = 30;//待测
            ROS_ERROR("turn left%d", Parameter::turn_left);
        }
        
    }
    else if(erro_angle<-30)
    {
        if(Parameter::turn_right>1)
        {
            Parameter::turn_right_flag = true;
            Parameter::turn_time = 0;
            Parameter::turn_right = 0;
            Control::aim_dis = 0;
        }
        else
        {
            Control::aim_dis = -30;//待测
            Parameter::turn_right++;
            ROS_ERROR("turn right%d", Parameter::turn_right);
        }
    }
    else if(erro_angle<5&&erro_angle>-5)
    {
        Control::aim_dis = 0.0;
        Parameter::turn_left = 0;
        Parameter::turn_right = 0;
    }
}
double Control::getAngleData(double angle)
{
    Control::angle_data.push(angle);
    if (angle_data.size() < 5)
    {
        std::cout << "<5"<<std::endl;
        return angle;
    }
    else if (angle_data.size() > 5)
    {
        std::cout << "wrong"<<std::endl;
        return 0;
    }
    else
    {
        double angle[5];
        for (int i = 0; i < 5; i++)
        {
            angle[i]=angle_data.front();
            angle_data.push(angle[i]);
            angle_data.pop();
        }
        angle_data.pop();
        sort(angle, angle + 5);
        std::cout << "=5"<<std::endl;
        double r_angle = (angle[1] + angle[2] + angle[3]) / 3.0;
        return r_angle;
    }
    std::cout << "no!!"<<std::endl;
    
}
double Control::pidAngleData() /**角度控制**/
{
    double d_angle;
    double pid_angle;
    double erro_angle;
    double kp_rot = Parameter::angle_kp;
    double kd_rot = Parameter::angle_kd;
    double ki_rot = Parameter::angle_ki;

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
    Control::count_pid_angle = Control::count_pid_angle + d_angle;

    //为防止出现偶尔的角度误判
    if (fabs(d_angle) > 85)
    {
        pid_angle = pre_pid_angle;
        std::cout<<"nonononn" << d_angle << std::endl;
    }
    else
    {
        pid_angle = kp_rot * erro_angle + kd_rot * d_angle + ki_rot * Control::count_pid_angle;
        std::cout << "i angle:"<< Control::count_pid_angle * ki_rot<<std::endl;
        Control::pre_angle = erro_angle;
        Control::pre_pid_angle = pid_angle;
        
    }
    

    /**
    if(fabs(d_angle)>70){
        pid_angle = pre_pid_angle;
    }
    **/
    //std::cout <<"angle:"<< pid_angle;
    //pid_angle = Control::getAngleData(pid_angle);
    //std::cout << "after" << pid_angle << "over" << std::endl;
    return pid_angle;
}

double Control::pidDisData() //左右距离控制
{
    double d_dis = 0, pid_dis, kd_dis, kp_dis, ki_dis;
    kp_dis = Parameter::dis_kp;
    kd_dis = Parameter::dis_kd;
    ki_dis = Parameter::dis_ki;
    if (Detections::flag_circle == true && Parameter::is_cross == false)
    {
        double D;
        double dis_kp = 0.18, dis_kd = 0.05;//0.18 0.05
        double p = Detections::circle_point.x - 320;
        p=-p;
        D=Control::b_dis;
        pid_dis=(dis_kp*p+dis_kd*D) / 100.0;
        while(fabs(pid_dis)<0.001&&fabs(pid_dis)>0)
        {
            pid_dis = pid_dis * 10;
        }
        Control::b_dis = p;
//        pre_error_dis=aim_dis;
//        
//        ROS_INFO_STREAM("pid_dis:  "<<pid_dis);
    }
    else
    {
        d_dis = Detections::aim_dis - pre_dis;
        Control::count_pid_dis = d_dis + Control::count_pid_dis;
        std::cout<<"i dis" << Control::count_pid_dis * ki_dis<<std::endl;
        pid_dis = kp_dis * Detections::aim_dis / 100.0 + kd_dis * d_dis / 100.0 + Control::count_pid_dis * ki_dis/100.0;
        /**
        if (pid_dis > 0.02 && pid_dis < 0.1)
        {
            pid_dis = 0.1;
        }
        else if (pid_dis < -0.02 && pid_dis > -0.1)
        {
            pid_dis = -0.1;
        }
        **/
        pre_dis = Detections::aim_dis;
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

double Control::getSpeedData(double angle, double dis) //速度选择器
{
    double speed;
    if (Detections::flag_circle == true && Parameter::is_cross == false)
    {
        double dif = (Detections::circle_point.y - 270) / 70.0;
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
        while (fabs(speed) < 0.1 && fabs(speed) > 0)
        {
            speed = speed * 10;
            if (speed > 0.2)
            {
                speed = 0.2;
            }
            else if (speed < -0.2)
            {
                speed = -0.2;
            }
        }
        if (speed > 0.4)
        {
            speed = 0.4;
        }
        else if (speed < -0.4)
        {
            speed = -0.4;
        }
    }
    else
    {
        if (fabs(angle) < 0.1 && fabs(dis) < 0.1)
        {
            speed = Parameter::speed_straight;
        }
        else if (fabs(angle) < 0.2 && fabs(dis) < 0.2)
        {
            speed = Parameter::speed_normal;
        }
        else if (fabs(angle) < 0.3 && fabs(dis) < 0.3)
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
     * 存在的问题 转向时结算的dis显然太小
    **/
    double pid_angle,pid_dis,pid_high,get_speed;

    pid_angle = Control::pidAngleData();//角度控制
    pid_dis = Control::pidDisData();//左右控制
    pid_high = Control::pidHighData();//高度控制
    get_speed = Control::getSpeedData(pid_angle, pid_dis);//速度控制
    /* if (pid_angle > 0.3 && pid_dis < 0.1)
    {
        pid_dis = 0.1;
    }
    else if (pid_angle < -0.3 && pid_dis > -0.1)
    {
        pid_dis = -0.1;
    }
    */
    /* if(pid_dis>0.3||pid_dis<-0.3)
    {
        control_parameter[0][0] = 0.1;
    }
    else
    {
        control_parameter[0][0] = get_speed;
        control_parameter[0][2] = pid_high;
        control_parameter[1][2] = pid_angle;
    }
    */

    control_parameter[0][0] = get_speed;
    control_parameter[0][2] = pid_high;
    control_parameter[1][2] = pid_angle;
    if(fabs(pid_angle)>0.2)
    {
        control_parameter[0][1] = pid_dis+(pid_angle*0.165);
    }
    else
    {
        control_parameter[0][1] = pid_dis;
    }

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
