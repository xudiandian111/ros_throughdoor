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
    void controlCheck();
};
