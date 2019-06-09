//ROS Lib
#pragma once
#include "Bebop.h"
#include "Detections.h"
#include "Common.h"
#include "Parameter.h"
#include "NWPU.h"

class Control
{
public:
    Control(void);
    ~Control(void);

public:
    double judgeAngle();

    double pidAngleData();
    double pidDisData();
    double pidHighData();
    double pidSpecialAngle(double aim_angle, double real_angle);

    double getSpeedData(double angle, double dis);
    double getAimHigh(double high[30]);

    void controlGeneral(double control_parameter[][3]);
    void controlVupCircle(double control_parameter[][3]);
    void controlVfindCircle(double control_parameter[][3]);
    void controlVdownCircle(double control_parameter[][3]);
    void controlCross(double control_parameter[][3]);
    void controlTurnLeft(double control_parameter[][3]);
    void specialControl(int kind, double control_parameter[][3]);

    static double pre_angle;
    static double aim_angle;
    static int camera_angle;
    static int angle_change_count;
    static double judge_desire_angle;

    static double pre_high;
    static double aim_high;

    static double pre_dis;
    static int squar_count;

    static double b_dis;
    static double b_dif;

    static double special_pre_angle;
    static double turn_times;
};