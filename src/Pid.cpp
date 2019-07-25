#include "Pid.h"
/*似乎自适应PID在实物上表现并不是太好 在仿真平台上表现良好*/
Pid::Pid()
{

}
Pid::~Pid()
{

}
/*神经网络参数自整定PID控制器，规则*/
void Pid::pidRules(double zk, double uk, double x[3], std::string key)
{
    double wi, wp, wd, ki, kp, kd;
    Parameter::get<double>({{key + "wi", &wi},
                            {key + "wp", &wp},
                            {key + "wd", &wd},
                            {key + "ki", &ki},
                            {key + "kp", &kp},
                            {key + "kd", &kd}});
    wi = wi + ki * zk * uk * x[0];
    wp = wp + kp * zk * uk * x[1];
    wd = wd + kd * zk * uk * x[2];
    Parameter::set<double>({{key + "wi", wi},
                            {key + "wp", wp},
                            {key + "wd", wd}});
}

/**
 * 神经网络参数自整定PID控制器，以位置型方式实现
 * kcoef 神经元输出比例
 * dcoef 死区比例
 * wi wp wd 学习速度
 * vMax vMin 输出限幅
 * deadband 死区
 * **/
double Pid::neuralPid(double error, std::string v)
{
    double x[3], w[3], lasterror, result, sum,
        deadband, dcoef, vMax, vMin, kcoef,
        wi, wp, wd, sumerror;
    std::string key = "/autopid/" + v + "/";
    Parameter::get<double>({{key + "lasterror", &lasterror},
                            {key + "sumerror", &sumerror},
                            {key + "dcoef", &dcoef},
                            {key + "kcoef", &kcoef},
                            {key + "wi", &wi},
                            {key + "wp", &wp},
                            {key + "wd", &wd},
                            {key + "vMax", &vMax},
                            {key + "vMin", &vMin}});
    deadband = (vMax - vMin) * dcoef;
    sumerror = sumerror + error;
    if (fabs(error) > deadband)
    {
        x[0] = sumerror;
        x[1] = error;
        x[2] = error - lasterror;
        sum = fabs(wi) + fabs(wp) + fabs(wd);
        w[0] = wi / sum;
        w[1] = wp / sum;
        w[2] = wd / sum;
        result = (w[0] * x[0] + w[1] * x[1] + w[2] * x[2]) * kcoef;
    }
    else
    {
        result = 0;
    }
    if (result > vMax)
    {
        result = vMax;
    }
    else if(result<vMin)
    {
        result = vMin;
    }
    pidRules(error, result, x, key);
    Parameter::set<double>({{key + "lasterror", error},
                            {key + "sumerror", sumerror}});
    return result;
}
/**
 * 常规的位置型PID
 * **/
double Pid::normalPid(double error, std::string v)
{
    std::string key = "/normalpid/" + v + "/";
    double lasterror, kp, ki, kd, d_error, result, sumerror,
            vMax,vMin;
    Parameter::get<double>({{key + "lasterror", &lasterror},
                            {key + "sumerror", &sumerror},
                            {key + "kp", &kp},
                            {key + "ki", &ki},
                            {key + "kd", &kd},
                            {key + "vMax", &vMax},
                            {key + "vMin", &vMin}});
    d_error = error - lasterror;
    sumerror = error + sumerror;
    result = kp * error + kd * d_error;
    Parameter::set<double>({{key + "lasterror", error},
                            {key + "sumerror", sumerror}});
    if (result > vMax)
    {
        result = 0.8;
    }
    else if (result < vMin)
    {
        result = -0.8;
    }
    return result;
}
