#include "Pixhawk.h"
#include "KeyControl.h"
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

/**
 * 所有类依赖Parameter进行通信
 * Parameter实现对ros的一些儿函数的封装
 * **/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "offb_node");
    Pixhawk pixhawk;
    KeyControl keycontrol;
    boost::thread rosway(boost::bind(&Pixhawk::mavrosMain, &pixhawk));
    boost::thread keyway(boost::bind(&KeyControl::keyMain, &keycontrol));
    keyway.join();
    rosway.join();
    return 0;
}