//ROS Lib
#pragma once
#include "Bebop.h"
#include "Detections.h"
#include "Common.h"
#include "Parameter.h"
#include "NWPU.h"
#include <queue>
#include <algorithm>

class Control
{
public:
    Control(void);
    ~Control(void);

public:
    double getSpeedData(double angle, double dis);
    double getAngleData(double angle);

    void checkStatus();

    void controlGeneral(double control_parameter[][3]);

    void controlVupCircle(double control_parameter[][3]);
    void controlVfindCircle(double control_parameter[][3]);
    void controlVdownCircle(double control_parameter[][3]);

    void controlCross(double control_parameter[][3]);

    static double pre_angle;
    static double aim_angle;
    static double pre_pid_angle;

    static int camera_angle;
    static int angle_change_count;
    static double judge_desire_angle;

    static double pre_high;
    static double aim_high;

    static double pre_dis;
    static double aim_dis;

    static double b_dis;
    static double b_dif;

    static double special_pre_angle;
    static double turn_times;

    static double count_pid_dis;
    static double count_pid_angle;

    static queue<double> angle_data;
private:
    double pidAngleData();
    double pidDisData();
    double pidHighData();
};