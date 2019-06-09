#include "Parameter.h"
/**
 * 包含控制的常用参数
 * **/
double    Parameter::speed_auto_x = 0.2; //0.08
double    Parameter::speed_land_x = 0.1; //0.08

int Parameter::cv_a = 75; //46
bool Parameter::cv_heng = false;
bool Parameter::cv_shu = true;

double Parameter::angle_kp = 0.009;    //0.018
double Parameter::angle_kd = 0.001;    //0,004

double Parameter::dis_kp = 0.18; //0.05 0.18 0.05
double Parameter::dis_kd = 0.05;  //2.0

double Parameter::high_kp = 0.23; //0.23
double Parameter::high_kd = 0.4;   //0.40

double Parameter::speed_kp = 0.35;
double Parameter::speed_kd = 0.06;

//速度项采用分级速度对 转弯 直行 钻圈分别进行控制

double Parameter::speed_straight = 0.3;    //直行速度
double Parameter::speed_normal = 0.3;     //正常速度
double Parameter::speed_low = 0.2;      //慢速行驶
double Parameter::speed_turn = 0.1;     //转向速度

//以下为测试标志部分
bool Parameter::turn_flag = false; //转向测试
bool Parameter::cross_v_up_flag = false;
bool Parameter::cross_v_down_flag = false;
bool Parameter::land_flag = false;
bool Parameter::cross_state = false;
bool Parameter::special_flag = false;
bool Parameter::is_cross = false;
int Parameter::fly_time = 0;
bool Parameter::turn_left_flag = false;
int Parameter::turn_time = 0;
int Parameter::turn = 0;

Parameter::Parameter(void)
{
}
Parameter::~Parameter(void)
{
}
void Parameter::initParameter()
{
    Parameter::turn_flag = false;
    Parameter::cross_v_up_flag = false;
    Parameter::cross_v_down_flag = false;
    Parameter::land_flag = false;
    Parameter::cross_state = false;
    Parameter::special_flag = false;
}
