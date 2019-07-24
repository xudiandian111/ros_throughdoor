//作为对图像的对接函数
#include "Detections.h"
#include <ros/ros.h>
FindLines lineans;
FindLand land;
/**
 * 待填充参数 
 * bool find_h 是否发现降落点  
 * double h_dis 与降落点的直线距离
 * **/

Detections::Detections(void)
{

}
Detections::~Detections(void)
{

}
void Detections::imagePcs(const sensor_msgs::ImageConstPtr& cam_image)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat cv_image,imgforlines,imgforcircle,imgforland;
    
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
    cv_image.copyTo(imgforland);

    lineans.lineMain(imgforlines, imgforlines);
    land.landMain(imgforland);

    Point land_point;
    double land_point_x, land_point_y, find_land, 
           flag_line, aim_angle, aim_dis;

    land_point = land.getCenter();
    land_point_x = land_point.x;
    land_point_y = land_point.y;
    find_land = land.getFindH();
    Parameter::set("land_point_x", land_point_x);
    Parameter::set("land_point_y", land_point_y);
    Parameter::set("find_land", find_land);

    flag_line = lineans.Flag();
    aim_angle = lineans.Angle();
    aim_dis = lineans.Dis();

    Parameter::set("flag_line", flag_line);
    Parameter::set("aim_angle", aim_angle);
    Parameter::set("aim_dis", aim_dis);
}