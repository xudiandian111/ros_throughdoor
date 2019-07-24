//对外部环境监测的一些神奇操作
#pragma once
//my lib
#include "FindLines.h"
#include "Parameter.h"
#include "FindLand.hpp"
//C++ lib
#include <iostream>
#include <vector>
#include <fstream>
#include <iostream>
#include <stack>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

class Detections
{
public:
	Detections(void);
	~Detections(void);
public:
    void imagePcs(const sensor_msgs::ImageConstPtr &cam_image);
};