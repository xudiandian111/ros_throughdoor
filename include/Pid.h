#pragma once
#include <cmath>
#include <iostream>
#include "Parameter.h"
class Pid
{
private:
    void pidRules(double zk, double uk, double x[3], std::string key);
public:
    Pid();
    ~Pid();
    double neuralPid(double error, std::string v);
    double normalPid(double error, std::string v);
};
