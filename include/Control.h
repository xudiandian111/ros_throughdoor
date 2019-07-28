#pragma once
#include "Pid.h"
class Control
{
private:
    Pid pid;

public:
    Control();
    ~Control();
    void controlMain(std::string mode);
    void controlLine(double cmd[][3]);
    void controlDoor(double cmd[][3]);
    void controlLand(double cmd[][3]);
    void controlDoorUp(double cmd[][3], int step);
};
