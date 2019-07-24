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
                            {"/control/speed/z", cmd[0][2]},
                            {"/control/angle/x", cmd[1][0]},
                            {"/control/angle/y", cmd[1][1]},
                            {"/control/angle/z", cmd[1][2]}});
    ROS_INFO("key:%s", key.c_str());
}
void KeyControl::keyMain()
{
    /************必须开小写键盘！！！*************************************/
    int kfd = 0;
    char c;
    struct termios cooked, raw;
    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;
    int num;
    int y;
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
    while (1)
    {
        std::string mode = Parameter::getString("/state/mode/current");

        if (Parameter::getBool("/params/other/threadrosout") == true)
        {
            break;
        }
        boost::this_thread::interruption_point();
        int num;
        num = poll(&ufd, 1, 250);
        if (num < 0)
        {
            perror("poll():");
            continue;
            return;
        }
        else if (num > 0)
        {
            if (read(kfd, &c, 1) < 0)
            {
                perror("read():");
                continue;
                return;
            }
        }
        else
        {
            continue;
        }
        switch (c)
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
                Parameter::set("/state/mode/current", "fly");
                Parameter::initParam();
                ROS_WARN("mode takeoff");
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