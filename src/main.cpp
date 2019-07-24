//当前已完成测试部分
//bebop控制良好 转向测试良好 
//待测试的部分 摄像头的转向
#include "Bebop.h"
#include "Common.h"
#include "Control.h"
#include "Detections.h"

#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_takeoff");
    Bebop useBebop;
    Detections useDetections;

    boost::thread rosway(boost::bind(&Bebop::rosVersion, &useBebop));
//    boost::thread camway(boost::bind(&Detections::cameraUse, &useDetections));
    boost::thread keyway(boost::bind(&Bebop::keyVersion, &useBebop));
    rosway.join();
    keyway.join();
//    camway.join();

    return 0;
}
