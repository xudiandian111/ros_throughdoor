#include "KeyControl.h"
#define keytest
KeyControl::KeyControl()
{
}
KeyControl::~KeyControl()
{
}
void KeyControl::controlMain(std::string key)
{
   if (Parameter::getString("/state/mode/current") == "fly")
    {
        Parameter::set("/state/mode/current", "key");
        ROS_WARN("mode key");
        return;
    }
    double cmd[2][3] = {0, 0, 0, 0, 0, 0};
    if (key == "up")
    {
        cmd[0][2] = 0.3;
        ROS_INFO("key:%s", key.c_str());
    }
    else if(key=="down")
    {
        cmd[0][2] = -0.3;
    }
    else if(key=="forward")
    {
        cmd[0][0] = 0.3;
    }
    else if(key=="backward")
    {
        cmd[0][0] = -0.3;
    }
    else if(key=="left")
    {
        cmd[0][1] = 0.3;
    }
    else if(key=="right")
    {
        cmd[0][1] = -0.3;
    }
    else if(key=="ccw")
    {
        cmd[1][2] = 0.3;
    }
    else if(key=="cw")
    {
        cmd[1][2] = -0.3;
    }
    else
    {
    }
    Parameter::set<double>({{"/control/speed/x", cmd[0][0]},
                            {"/control/speed/y", cmd[0][1]},
                            {"/control/angle/x", cmd[1][0]},
                            {"/control/angle/y", cmd[1][1]},
                            {"/control/angle/z", cmd[1][2]}});
    ROS_INFO("key:%s", key.c_str());
}
void KeyControl::keyMain()
{
    /************必须开小写键盘！！！*************************************/
    char in;
    struct termios new_settings;
    struct termios stored_settings;
    tcgetattr(0,&stored_settings);
    new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(0,&stored_settings);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0,TCSANOW,&new_settings);
    char value;
    while (1)
    {
        value = getchar();
        std::string mode = Parameter::getString("/state/mode/current");

        if (Parameter::getBool("/params/other/threadrosout") == true)
        {
            break;
        }
        boost::this_thread::interruption_point();
        switch (value)
        {
            case 'b': //auto_control
#ifdef keytest
                if(mode == "fly")
                {
                    ROS_WARN("mode auto");
                    Parameter::initParam();
                }
                else if (mode == "auto")
                {
                    ROS_WARN("mode wait");
                }
#endif
                if (mode == "fly")
                {
                    Parameter::set("/state/mode/current", "auto");
                }
                else
                {
                    Parameter::set("/state/mode/current", "fly");
                    Parameter::set<double>({{"/point/pose/x", Parameter::getDouble("/state/pose/x")},
                                            {"/point/pose/y", Parameter::getDouble("/state/pose/y")}});

                }
                break;
            case 'y':
                Parameter::set("/state/mode/current", "land");
                Parameter::set<double>({{"/point/pose/x", Parameter::getDouble("/state/pose/x")},
                                        {"/point/pose/y", Parameter::getDouble("/state/pose/y")}});
                ROS_WARN("mode land");
                break;
            case 't':
                if(mode == "land")
                {
                    Parameter::set("/state/mode/current", "fly");
                    Parameter::initParam();
                    ROS_WARN("mode takeoff");
                }
                else
                {
                    ROS_ERROR("ERROR:You are not in land mode");
                }
                break;
            case 'w':
                controlMain("forward");
                break;
            case 's':
                controlMain("backward");
                break;
            case 'a':
                controlMain("left");
                break;
            case 'd':
                controlMain("right");
                break;
            case 'i':
                controlMain("up");
                break;
            case 'k':
                controlMain("down");
                break;
            case 'j':
                controlMain("ccw");
                break;
            case 'l':
                controlMain("cw");
                break;
            default:
                break;
        }
    }
}