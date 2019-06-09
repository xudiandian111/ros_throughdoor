//包含对外部路径检测的操作
#include "Detections.h"
#include <ros/ros.h>
FindLines lineans;
FindCircle circleans;
bool Detections::flag_line = false;
bool Detections::flag_circle = false;

double Detections::aim_angle = 0;
double Detections::aim_dis = 0;
Point Detections::circle_point;

Detections::Detections(void)
{

}
Detections::~Detections(void)
{

}
void Detections::imagePcs(const sensor_msgs::ImageConstPtr& cam_image)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat cv_image,imgforlines,imgforcircle;
    
    try
    {
        cv_ptr = cv_bridge::toCvCopy(cam_image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception:%s", e.what());
        return;
    }
    
    cv_image = cv_ptr->image;
    resize(cv_image,cv_image,Size(640,480),0,0,INTER_LINEAR);
    cv_image.copyTo(imgforlines);
    cv_image.copyTo(imgforcircle);
    // namedWindow("imgforlines",1);
    // imshow("imgforlines",imgforlines);
    // waitKey(10);

	lineans.lineMain(imgforlines,imgforlines);
    Detections::flag_line = lineans.Flag();
    Detections::aim_angle = lineans.Angle();
    Detections::aim_dis = lineans.Dis();
    // if(!flag_line) ROS_INFO("no lines,no lines!!!");
    // else{
    //    ROS_INFO("lines!!!"); 
    // }
    // circleans.circleMain(imgforcircle);

    // flag_circle=circleans.getFlag();
    //aim_height=flag_circle?(0.7+circleans.getHeight()/100):0.7;

    circleans.circleMain(imgforcircle);
    Detections::flag_circle=circleans.getFlag();
    Detections::circle_point=circleans.getPoint();
}