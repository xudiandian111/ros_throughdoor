#pragma once
#include "Pid.h"
class Control
{
private:
    Pid pid;
    void controlLine(double cmd[][3]);
    void controlDoor(double cmd[][3]);
    void controlLand(double cmd[][3]);
    void controlDoorUp(double cmd[][3], int step);
    void controlDoorDown(double cmd[][3], int step);
    void controlDoorTo(double cmd[][3], int step);
public:
    Control();
    ~Control();
    void controlMain(std::string mode);
};
