#pragma once
class Parameter
{
public:
    Parameter(void);
    ~Parameter(void);

public:
    /**bebop_maincontrol**/
    static double speed_land_x;
    static double speed_auto_x;
    /*Control PID*/
    static double angle_kp;
    static double angle_kd;
    static double angle_ki;
    static double dis_kp;
    static double dis_kd;
    static double dis_ki;
    static double high_kp;
    static double high_kd;
    static double speed_kp;
    static double speed_kd;
    /*speed control*/
    static double speed_straight;
    static double speed_normal;
    static double speed_low;    
    static double speed_turn;

    /*test flag*/
    static bool turn_flag;
    static bool cross_v_up_flag;
    static bool cross_v_down_flag;
    static bool land_flag;
    static bool cross_state;
    static bool special_flag;
    /*init*/
    static void initParameter();
    /*cross*/
    static bool is_cross;
    static int fly_time;
    static int turn_time;
    static int turn;
    /*flag*/
    static bool turn_left_flag;
    static bool turn_right_flag;
    static int turn_left;
    static int turn_right;
};