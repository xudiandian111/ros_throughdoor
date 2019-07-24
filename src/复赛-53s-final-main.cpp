/******************Zhang_Zh_Taiwan***************/
/*****此版本为实验室的最终决赛版本时间为53s左右，寻线和转弯都是罗盘角度的控制效果比较好**/
//ROS Lib
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <bebop_msgs/Ardrone3PilotingStateAttitudeChanged.h>
#include<bebop_msgs/Ardrone3PilotingStatePositionChanged.h>
#include<bebop_msgs/Ardrone3PilotingStateAltitudeChanged.h>
#include<bebop_msgs/Ardrone3PilotingStateGpsLocationChanged.h>
#include<bebop_msgs/Ardrone3PilotingStateSpeedChanged.h>

//C++ Standard Lib
#include <vector>
#include <fstream>
#include <iostream>
#include <stack>


//OpenCV Lib
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//my lib
#include "ccc/FindQR.h"
#include "ccc/FindLines.h"
#include "ccc/Linezxz.h"

//ubuntu lib
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <signal.h>
#include <boost/thread/thread.hpp>
#include <sys/poll.h>
#include <termios.h>
#define keytest
#define front 0
#define back 1
#define left 2
#define right 3

std_msgs::Empty emp_msg;

ros::Publisher pub_takeoff;
ros::Publisher pub_land;
ros::Publisher pub_control;
ros::Publisher pub_eland;
ros::Publisher auto_control;
ros::Publisher camera_control;
image_transport::Subscriber 	image_sub;
ros::Subscriber angle;
ros::Subscriber height;
ros::Subscriber speed;

/*********fly command**************/
geometry_msgs::Twist fly_up;
geometry_msgs::Twist fly_down;
geometry_msgs::Twist fly_left;
geometry_msgs::Twist fly_right;
geometry_msgs::Twist fly_forward;
geometry_msgs::Twist fly_backward;
geometry_msgs::Twist fly_turn_right;
geometry_msgs::Twist fly_turn_left;
geometry_msgs::Twist fly_pause;
geometry_msgs::Twist change_camera;
/**********for safe exit************/
bool threadrosout	=	false;
bool threadkeyout	=	false;

/*****************************************************/
int             a=75;//46

/***********QR control**************/
int 		QRtime = 100;
FindQR		QRans;
stack<string>	q;
vector<string>  road;
const string	EEE = "east";
const string	WWW = "west";
const string	NNN = "north";
const string	SSS = "south";

int 		play_time=0;
/***********FindLines****************/
FindLines	Lineans;
/***********CutBalloons**************/
void cam(int& posi);
Point center(Rect rec);
void color(Mat& img);
void on_mouse(int EVENT, int x, int y,int flag,void* userdata);
const unsigned char FORE_GROUD = 255;
int thresh = 70;
int posi = 0;
//#define COLOR 255
#define WKD 666
/***************turn******************/
void TurnTime(cv::Mat& src) ;//转弯图像
bool ready_for_turn = false ;//识别到转弯点返回为true
bool turn_flag=true;//开启转弯识别函数
/************land********************/
void landTime(cv::Mat& src);//降落图像
bool ready_for_land = false ;//识别到降落点返回为true
bool land_flag=false;//开启关闭降落识别函数
/***********cross********************/
void crossTime(cv::Mat& src);//十字路口图像
bool ready_for_cross = false ;//识别到转弯点返回为true
bool cross_flag=false;//开启关闭十字路口识别函数
/**********autocontrol***************/
double ardrone_z_attitude = 0.0f;
double ardrone_height=0.0f;
double ardrone_height_2=0.0f;
double ardrone_height_3=0.0f;
double ardrone_speed=0.0f;
float ardrone_speed2=0.0f;
/**********auto controll*************/
#define MAX_IBVS_NUM 500
bool 			autoswitch;
geometry_msgs::Twist 	auto_command;
int                     angle_change_count=0;
int 			control_time=0;
double 			judge_desire_angle=17.9;
int               	flag=false;
bool                    heng=false;
bool                    shu=true;
double                  aim_angle=17.9;
int                     cross_time=0;
double                  aim_high=0.8;
int                     fly_time=0;
int                     land_time=0;
int                     squar_count=0;
bool                    drop_flag=false;
int                     land_wait_time=0;
int                     camera_angle=-48;
bool                    down_flag=false;
static int statictime=0;

void control_main();
double pid_dis_data();  //距离解算
double pid_angle_data(); //角度解算
double judge_angle();
double pid_high_data();  //高度解算
void camera_angle_control(); 

void set_timer()  //系统
{
        struct itimerval itv;
        itv.it_value.tv_sec = 0;
        itv.it_value.tv_usec = 50000;


        itv.it_interval.tv_sec = 0;
        itv.it_interval.tv_usec = 50000;

        setitimer(ITIMER_REAL, &itv, NULL);
}
void readangle(const bebop_msgs::Ardrone3PilotingStateAttitudeChanged::ConstPtr& msg)//系统
{
        ardrone_z_attitude = msg->yaw;
        //ROS_INFO("normaltest:yaw : %f", msg->yaw);
}
void readheight(const bebop_msgs::Ardrone3PilotingStateAltitudeChanged::ConstPtr& msg)//系统
{
        ardrone_height =msg->altitude;
        if(aim_high==0.7&&ardrone_height>0.8) ROS_INFO("normaltest::altitude1: %f",msg->altitude);//gaodu
        if(aim_high==1.3&&ardrone_height>1.35) ROS_INFO("normaltest::altitude1: %f",msg->altitude);//gaodu

}
void readspeed(const bebop_msgs::Ardrone3PilotingStateSpeedChanged::ConstPtr& msg)//系统
{
        ardrone_speed2 =msg->speedX;
        //ROS_INFO("ardrone_speed: %f",msg->speedX);//gaodu
}
void sigalrm_handler(int sig)//系统
{
        play_time++;
        QRtime++;
        control_time++;
	    fly_time++;
        cross_time++;
        land_time++;
        land_wait_time++;
        //std::cout<<play_time<<std::endl;
}
void initFlyParam()//初始化
{
    autoswitch=false;
	fly_up.linear.x = 0;
	fly_up.linear.y = 0;
	fly_up.linear.z = 0.5;
	fly_up.angular.x = 0;
	fly_up.angular.y = 0;
	fly_up.angular.z = 0;

	fly_down.linear.x = 0;
	fly_down.linear.y = 0;
	fly_down.linear.z = -0.3;
	fly_down.angular.x = 0;
	fly_down.angular.y = 0;
	fly_down.angular.z = 0;

	fly_left.linear.x = 0;
    fly_left.linear.y = 0.2;
	fly_left.linear.z = 0;
	fly_left.angular.x = 0;
	fly_left.angular.y = 0;
	fly_left.angular.z = 0;

	fly_right.linear.x = 0;
    fly_right.linear.y = -0.2;
	fly_right.linear.z = 0;
	fly_right.angular.x = 0;
	fly_right.angular.y = 0;
	fly_right.angular.z = 0;

    fly_forward.linear.x = 0.2;
	fly_forward.linear.y = 0;
	fly_forward.linear.z = 0;
	fly_forward.angular.x = 0;
	fly_forward.angular.y = 0;
	fly_forward.angular.z = 0;

    fly_backward.linear.x = -0.2;
	fly_backward.linear.y = 0;
	fly_backward.linear.z = 0;
	fly_backward.angular.x = 0;
	fly_backward.angular.y = 0;
	fly_backward.angular.z = 0;

	fly_turn_right.linear.x = 0;
	fly_turn_right.linear.y = 0;
	fly_turn_right.linear.z = 0;
	fly_turn_right.angular.x = 0;
	fly_turn_right.angular.y = 0;
	fly_turn_right.angular.z = -1.0;

	fly_turn_left.linear.x = 0;
	fly_turn_left.linear.y = 0;
	fly_turn_left.linear.z = 0;
	fly_turn_left.angular.x = 0;
	fly_turn_left.angular.y = 0;
	fly_turn_left.angular.z = 1.0;


	fly_pause.linear.x = 0;
	fly_pause.linear.y = 0;
	fly_pause.linear.z = 0;
	fly_pause.angular.x = 0;
	fly_pause.angular.y = 0;
	fly_pause.angular.z = 0;


}

void imageRcognition(const sensor_msgs::ImageConstPtr& cam_image)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat imgforQR,cv_image,imgforLines,imgforLinesII,imgforlandr,imgforlandh;
    cv::Mat imgforTurn,imgforCross,imgforLand ;
    IplImage pFrame,pFrameII;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(cam_image, sensor_msgs::image_encodings::BGR8);
    }

    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception:%s", e.what());
        return;
    }
    //640*360
    cv_image = cv_ptr->image;
    resize(cv_image,cv_image,Size(640,480),0,0,INTER_LINEAR);
    cv_image.copyTo(imgforQR);
    cv_image.copyTo(imgforLines);
    cv_image.copyTo(imgforLinesII);
    cv_image.copyTo(imgforlandr);
    cv_image.copyTo(imgforlandr);
    cv_image.copyTo(imgforTurn);
    cv_image.copyTo(imgforCross);
    cv_image.copyTo(imgforLand);
   // landversion(imgforlandr,imgforlandh);
    //cv::pow(imgforLines,4.0,imgforLines);
    pFrame   = IplImage(imgforLines);
    pFrameII = IplImage(imgforLinesII);
    //cvShowImage("video8", &pFrame);
    Lineans.findmain(&pFrame,&pFrameII);// find line

    //when to turn
    if(turn_flag==true)TurnTime(imgforTurn);
    //when to cross
   if(cross_flag==true)
   {
       crossTime(imgforCross) ;
       //cout<<"              CROSS            "<<endl;
   }
    //when to land
    if(land_flag==true)
    {
        landTime(imgforLand) ;
        //cout<<"              LAND            "<<endl;
    }
    cv::waitKey(1);
}
void camerause()
{
    cam(posi);
}
void cam(int& posi)
{
    VideoCapture video(1);
    //判断如果video是否可以打开
    if (!video.isOpened())
        return ;

    //用于保存当前帧的图片
    Mat currentBGRFrame ;
    Mat previousBGRFrame;
    //用来保存上一帧和当前帧的灰度图片
    Mat previousGrayFrame;
    Mat currentGaryFrame;

    //用来保存帧差
    Mat frameDifference;//CV_16SC1

                        //用来保存帧差的绝对值
    Mat absFrameDifferece;

    //用来显示前景
    Mat segmentation;

    //显示原视频
    namedWindow("video", 1);

    //显示前景
    namedWindow("segmentation", 1);
    createTrackbar("阈值:", "segmentation", &thresh, FORE_GROUD, NULL);

    //帧数
    int numberFrame = 0;

    //形态学处理用到的算子
    Mat morphologyKernel = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));

    for (;;)
    {
        video >> currentBGRFrame;
        //判断当前帧是否存在
        if (!currentBGRFrame.data)
            break;

        numberFrame++;

#ifdef COLOR
        //捕捉第50帧图像
        if(numberFrame==50)
            color(currentBGRFrame);
#endif
        if (numberFrame == 1)
        {
            //颜色空间的转换
            cvtColor(currentBGRFrame, currentGaryFrame, COLOR_BGR2GRAY);
            //保存当前帧的灰度图、三通道图
            previousGrayFrame = currentGaryFrame.clone();
            previousBGRFrame = currentBGRFrame.clone();
            //imshow("video", currentBGRFrame);
            continue;
        }
        else
        {
            //颜色空间的转换
            cvtColor(currentBGRFrame, currentGaryFrame, COLOR_BGR2GRAY);

            //src1-src2
            subtract(currentGaryFrame, previousGrayFrame, frameDifference, Mat(), CV_16SC1);

            //取绝对值
            absFrameDifferece = abs(frameDifference);

            //位深的改变
            absFrameDifferece.convertTo(absFrameDifferece, CV_8UC1, 1, 0);

            //阈值处理
            threshold(absFrameDifferece, segmentation, double(thresh), double(FORE_GROUD), THRESH_BINARY);

            //中值滤波
            medianBlur(segmentation, segmentation, 3);

            //形态学处理(开闭运算)
            //morphologyEx(segmentation,segmentation,MORPH_OPEN,morphologyKernel,Point(-1,-1),1,BORDER_REPLICATE);
            morphologyEx(segmentation, segmentation, MORPH_CLOSE, morphologyKernel, Point(-1, -1), 1, BORDER_REPLICATE);

            //显示二值化图片
            imshow("segmentation", segmentation);

            //找边界
            vector< vector<Point> > contours;
            vector<Vec4i> hierarchy;
            findContours(segmentation, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));//CV_RETR_TREE
            vector< vector<Point> > contours_poly(contours.size());

            //存储运动物体
            vector<Rect> boundRect;
            boundRect.clear();
            //画出运动物体
            //对视频中出现的运动物体，进行初次的筛选
            // static int cnt = 0;
            static int BALL[5] = { 0,1,2,3,4 };
            for (unsigned int index = 0;index < contours.size();index++)
            {
                approxPolyDP(Mat(contours[index]), contours_poly[index], 3, true);
                Rect rect = boundingRect(Mat(contours_poly[index]));
                Rect rec;
                if (rect.area() < 50)
                    rec = rect;
                //judge yellow
                Point mi ;
                mi = center(rec);
                int  mi_b = previousBGRFrame.at<Vec3b>(mi.y,mi.x)[0];
                int  mi_g = previousBGRFrame.at<Vec3b>(mi.y,mi.x)[1];
                int  mi_r = previousBGRFrame.at<Vec3b>(mi.y,mi.x)[2];

                if (mi_b<120&& mi_g<170&&mi_r<130&&(mi_g-mi_b)>80)
                {
                    Point centre;
                    centre = center(rec);
                    if (rec.area() < 25)
                    {
                        rectangle(currentBGRFrame, rect, Scalar(0, 255, 255), 2);
                #ifdef WKD
                        if (centre.x < 320 && centre.y < 240)
                        {
                            posi = -BALL[1];
                            //BALL[1] = 0;
                            cout << posi << " BALLOON IS YEEEEEELLOOOOOOW AT ( " << centre.x << " , " << centre.y << " )" << endl;
                            //return;
                        }
                        if (centre.x >= 320 && centre.y < 240)
                        {
                            posi = -BALL[2];
                            //BALL[2] = 0;
                            cout << posi << " BALLOON IS YEEEEEELLOOOOOOW AT ( " << centre.x << " , " << centre.y << " )" << endl;
                            //return;
                        }

                        if (centre.x < 320 && centre.y >= 240)
                        {
                            posi = -BALL[3];
                            //BALL[3] = 0;
                            cout << posi << " BALLOON IS YEEEEEELLOOOOOOW AT ( " << centre.x << " , " << centre.y << " )" << endl;
                            //return;
                        }
                        if (centre.x >= 320 && centre.y >= 240)
                        {
                            posi = -BALL[4];
                            //BALL[4] = 0;
                            cout << posi << " BALLOON IS YEEEEEELLOOOOOOW AT ( " << centre.x << " , " << centre.y << " )" << endl;
                            //return;
                        }
                    #endif

                    }
                }

                //judge red
                Mat ROI_RED = currentBGRFrame(rec);
                Scalar mean_val = mean(ROI_RED);
                double mean_b = mean_val[0];
                double mean_g = mean_val[1];
                double mean_r = mean_val[2];
                //cout << currentBGRFrame.rows << "  " << currentBGRFrame.cols << endl;

                if (mean_b <65 && mean_g<65 && mean_r>100)
                {
                    Point centre;
                    centre = center(rec);
                    if (rec.area() < 25)
                    {
                        rectangle(currentBGRFrame, rect, Scalar(0, 255, 255), 2);
                #ifdef WKD
                        if (centre.x < 320 && centre.y < 240)
                        {
                            posi = BALL[1];
                            //BALL[1] = 0;
                            cout << posi << " BALLOON AT ( " << centre.x << " , " << centre.y << " )" << endl;
                            //return;
                        }
                        if (centre.x >= 320 && centre.y < 240)
                        {
                            posi = BALL[2];
                            //BALL[2] = 0;
                            cout << posi << " BALLOON AT ( " << centre.x << " , " << centre.y << " )" << endl;
                            //return;
                        }

                        if (centre.x < 320 && centre.y >= 240)
                        {
                            posi = BALL[3];
                            //BALL[3] = 0;
                            cout << posi << " BALLOON AT ( " << centre.x << " , " << centre.y << " )" << endl;
                            //return;
                        }
                        if (centre.x >= 320 && centre.y >= 240)
                        {
                            posi = BALL[4];
                            //BALL[4] = 0;
                            cout << posi << " BALLOON AT ( " << centre.x << " , " << centre.y << " )" << endl;
                            //return;
                        }
                #endif

                    }
                }
            }

            //显示原视频
            imshow("video", currentBGRFrame);

            //保存当前帧的灰度图、三通道图
            previousGrayFrame = currentGaryFrame.clone();
            previousBGRFrame = currentBGRFrame.clone();
        }

        if (waitKey(1) == 'q')
            break;

    }

}
Point center(Rect rec)
{
    Point cen;
    cen.x = rec.x + cvRound(rec.width / 2.0);
    cen.y = rec.y + cvRound(rec.height / 2.0);
    return cen;
}
void color(Mat& img)
{
    namedWindow("【display】");
    Mat src = img;
    setMouseCallback("【display】", on_mouse, &src);
    //while(1)
        //imshow("【display】", src);
}
void on_mouse(int EVENT, int x, int y,int flag,void* userdata)
{
    Mat hh;
    hh = *(Mat*)userdata;
    Point p(x, y);
    switch (EVENT)
    {
    case EVENT_LBUTTONDOWN:
    {

        printf("b=%d\t", hh.at<Vec3b>(p)[0]);
        printf("g=%d\t", hh.at<Vec3b>(p)[1]);
        printf("r=%d\n", hh.at<Vec3b>(p)[2]);
        circle(hh, p, 2, Scalar(255), 3);
    }
    break;
    
    }
}
void camera_angle_control()//相机角度控制
{
    change_camera.angular.y=camera_angle;
    change_camera.angular.z=0;
    camera_control.publish(change_camera);
}
void TurnTime(cv::Mat& src)//转弯检测
{
    cv::Mat srcImg = src.clone() ;
    cv::Mat grayImg1,grayImg2 ;
    cv::cvtColor(srcImg,grayImg1,CV_BGR2GRAY) ;
    cv::threshold(grayImg1,grayImg1,a,255.0,CV_THRESH_BINARY_INV) ;//28
    grayImg2 = grayImg1.clone() ;
    cv::Mat roi = grayImg1(cv::Range(grayImg1.rows-420,grayImg1.rows-400),cv::Range(grayImg1.cols/2-100,grayImg1.cols/2+100)) ;
    cv::Mat roi2 = grayImg2(cv::Range(grayImg2.rows-360,grayImg2.rows-340),cv::Range(grayImg2.cols/2+150,grayImg2.cols/2+200)) ;
    rectangle(grayImg1,cv::Point(grayImg1.cols/2-100,grayImg1.rows-420),cv::Point(grayImg1.cols/2+100,grayImg1.rows-400),cv::Scalar(255,255,255),1) ;
    rectangle(grayImg1,cv::Point(grayImg1.cols/2+150,grayImg1.rows-360),cv::Point(grayImg1.cols/2+200,grayImg1.rows-340),cv::Scalar(255,255,255),1) ;
    cv::imshow("Turn",grayImg1) ;
    cv::waitKey(1) ;
    int white1 = cv::countNonZero(roi) ;
    int white2 = cv::countNonZero(roi2) ;
    if(white1<500&&white2>500)
    {
        ready_for_turn = true ;
        std::cout<<"ready to turn"<<std::endl ;
    }
    else
    {
        ready_for_turn = false ;
        //std::cout<<"go straight"<<std::endl ;
    }
}
void crossTime(cv::Mat& src)//十字路口检测
{
    cv::Mat srcImg = src.clone() ;
    cv::Mat grayImg1,grayImg2 ;
    cv::cvtColor(srcImg,grayImg1,CV_BGR2GRAY) ;
    cv::threshold(grayImg1,grayImg1,a,255.0,CV_THRESH_BINARY_INV) ;//30
    grayImg2 = grayImg1.clone() ;
    cv::Mat roi = grayImg1(cv::Range(grayImg1.rows-420,grayImg1.rows-400),cv::Range(grayImg1.cols/2-100,grayImg1.cols/2+100)) ;
    cv::Mat roi2 = grayImg2(cv::Range(grayImg2.rows-360,grayImg2.rows-340),cv::Range(grayImg2.cols/2+150,grayImg2.cols/2+200)) ;
    rectangle(grayImg1,cv::Point(grayImg1.cols/2-100,grayImg1.rows-420),cv::Point(grayImg1.cols/2+100,grayImg1.rows-400),cv::Scalar(255,255,255),1) ;
    rectangle(grayImg1,cv::Point(grayImg1.cols/2+150,grayImg1.rows-360),cv::Point(grayImg1.cols/2+200,grayImg1.rows-340),cv::Scalar(255,255,255),1) ;
    //cv::imshow("Cross",grayImg1) ;
    //cv::waitKey(1) ;
    int white1 = cv::countNonZero(roi) ;
    int white2 = cv::countNonZero(roi2) ;
    //cout<<"white1 "<<white1<<"    white2 "<<white2<<endl ;
    if(white1>500&&white2>500)
    {
        ready_for_cross = true ;
        //std::cout<<"ready to cross"<<std::endl ;
    }
    else
    {
        ready_for_cross = false ;
        //std::cout<<"go straight"<<std::endl ;
    }
}
void landTime(cv::Mat& src)//降落检测
{
    cv::Mat srcImg = src.clone() ;
    cv::Mat grayImg1,grayImg2 ;
    cv::cvtColor(srcImg,grayImg1,CV_BGR2GRAY) ;
    cv::threshold(grayImg1,grayImg1,a,255.0,CV_THRESH_BINARY_INV) ;//30
    grayImg2 = grayImg1.clone() ;
    cv::Mat roi = grayImg1(cv::Range(grayImg1.rows-220,grayImg1.rows-200),cv::Range(grayImg1.cols/2-200,grayImg1.cols/2+200)) ;
    //cv::Mat roi2 = grayImg2(cv::Range(grayImg2.rows-180,grayImg2.rows-160),cv::Range(grayImg2.cols/2-50,grayImg2.cols/2+50)) ;
    rectangle(grayImg1,cv::Point(grayImg1.cols/2-200,grayImg1.rows-220),cv::Point(grayImg1.cols/2+200,grayImg1.rows-200),cv::Scalar(255,255,255),1) ;
    //rectangle(grayImg1,cv::Point(grayImg1.cols/2-50,grayImg1.rows-180),cv::Point(grayImg1.cols/2+50,grayImg1.rows-160),cv::Scalar(255,255,255),1) ;
   // cv::imshow("land",grayImg1) ;
    //cv::waitKey(1) ;
    int white1 = cv::countNonZero(roi) ;
    //int white2 = cv::countNonZero(roi2) ;
    if(white1<1000)
    {
        ready_for_land = true ;
        //std::cout<<"ready to land"<<std::endl ;
    }
    else
    {
        ready_for_land = false ;
        //std::cout<<"go straight"<<std::endl ;
    }
}

void rosversion()
{
    ros::NodeHandle node;
    ros::Rate loop_rate(20);
    image_transport::ImageTransport it(node);
    /************publish init*******************************/
    pub_takeoff = node.advertise<std_msgs::Empty>("/bebop/takeoff", 1);
    pub_land    = node.advertise<std_msgs::Empty>("/bebop/land", 1);
    pub_control = node.advertise<geometry_msgs::Twist>("/bebop/cmd_vel", 1);
    auto_control = node.advertise<geometry_msgs::Twist>("/bebop/cmd_vel", 1);
    pub_eland = node.advertise<std_msgs::Empty>("/bebop/reset", 1);
    image_sub   = it.subscribe("/bebop/image_raw", 10, imageRcognition);
    angle =node.subscribe("/bebop/states/ardrone3/PilotingState/AttitudeChanged",1,readangle);
    height=node.subscribe("/bebop/states/ardrone3/PilotingState/AltitudeChanged",1,readheight);//无人机高度变化。海拔高度为起飞点以上的海拔高度。
    speed=node.subscribe("/bebop/states/ardrone3/PilotingState/SpeedChanged",1,readspeed);
    camera_control=node.advertise<geometry_msgs::Twist>("/bebop/camera_control",1);
    initFlyParam();
    /***************timer***********************************/
    signal(SIGALRM, sigalrm_handler);//for timer
    set_timer();

    ROS_ERROR("init ok! Z_Z,good luck!\n");
    while (ros::ok())
    {
        camera_angle_control();//bebop摄像头转动函数
        //ROS_INFO("now angle: %lf\n",180/3.135905*ardrone_z_attitude);
        if(autoswitch)
        {
            control_main();
            //ROS_ERROR("fly_time :                %d",fly_time/20);
        }
        if(down_flag) pub_land.publish(emp_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    threadrosout=true;
}

double pid_dis_data()/****距离控制****/
{
    static double pre_dis=0;
    double d_dis=0,pid_dis;
    double kd_dis,kp_dis;
    kp_dis=0.080;//0.05
    kd_dis=2.40;//2.0
    d_dis=Lineans.dis-pre_dis;
    pid_dis=kp_dis*Lineans.dis/100.0+kd_dis*d_dis/100.0;
    pre_dis=Lineans.dis;
   // ROS_ERROR("PID_DIS :%lf\n",pid_dis);
    return pid_dis;
}
double pid_high_data()/*****高度控制******/
{
    static double pre_high=0;
    double d_high=0,pid_high,erro_high;
    double kd_high,kp_high;
    kp_high=2.0;//0.23
    kd_high=0;//0.40
    erro_high=aim_high-ardrone_height;
    d_high=erro_high-pre_high;
    pid_high=kp_high*erro_high+kd_high*d_high;
    pre_high=erro_high;
    //ROS_ERROR("PID_high :%lf\n",pid_high);
    return pid_high;
}
double judge_angle()/****得到转角****/
{
    double current_angle=180/3.135905*ardrone_z_attitude;//当前的角度
    if(current_angle<0) current_angle+=360;
    if(ready_for_turn==true)
    {
        switch (angle_change_count)
        {
        case 0:
            judge_desire_angle=110.0;
            aim_high=0.7;
            camera_angle=-50;//50
            break;
        case 1:
            judge_desire_angle=-175.5;
            aim_high=1.3;
            camera_angle=-41;
            break;
        case 2:
            judge_desire_angle=-75.8;
            aim_high=1.3;
            camera_angle=-60;
            break;
        case 3:
            judge_desire_angle=9.0;
            aim_high=1.3;
            camera_angle=-48;//49
            break;
        case 4:
            judge_desire_angle=110.0;
            aim_high=0.7;//0.8
            camera_angle=-42;//39
            squar_count++;
            if(squar_count==2)camera_angle=-52;
            cross_time=0;
            //ROS_ERROR("HHHHHHHHHHHHHHHHHH:%d\n",squar_count);
            angle_change_count=0;
            break;
        }
    }
    if(ready_for_cross==true)
    {
        judge_desire_angle=-164.0;
        aim_high=0.7;
        camera_angle=-45;
        land_wait_time=0;
    }
    if(aim_angle!=judge_desire_angle)
    {
        aim_angle=judge_desire_angle;
        angle_change_count++;
        turn_flag=false;
        ready_for_turn=false;
        //ROS_INFO("JI SHU 111111111111111111111111111\n");
        play_time=0;
    }
    //ROS_ERROR("get the angle: %lf\n",judge_desire_angle);
    return judge_desire_angle;
}
double pid_angle_data()/*****角度控制*****/
{
    static double pre_angle=0;
    double d_angle;
    double pid_angle;
    double erro_angle;
    double kp_rot=0.0101;//0.018
    double kd_rot=0.0;//0.004
    double desire_angle= judge_angle();
    double ardrone_angle=180/3.135905*ardrone_z_attitude;
    if(ardrone_angle<0) ardrone_angle=ardrone_angle+360;
    erro_angle=desire_angle-ardrone_angle;
    if(erro_angle>180)//坐标系角度转换
    {
        erro_angle-=360;
    }
    if(erro_angle<-180)
    {
        erro_angle+=360;
    }
    d_angle=erro_angle-pre_angle;
    pid_angle=kp_rot*erro_angle+kd_rot*d_angle;
    pre_angle=erro_angle;
   // ROS_INFO("pid_angle :%lf\n",pid_angle);
    return pid_angle;
}
void control_main()/****寻线控制****/
{
    /********  angle_control 获取pid调节后的角度 **********/
    double pid_angle; 
    pid_angle=pid_angle_data();
    /********  dis_control-ardrone_height  **********/
    double pid_dis;
    pid_dis=pid_dis_data();
    /********  height_control  ***********/
    double pid_high;
    pid_high=pid_high_data();
    double angle_erro_control=fabs(180/3.135905*ardrone_z_attitude)-fabs(judge_desire_angle);
    //double dis_erro_control=fabs(Lineans.dis);
    
    if(fabs(angle_erro_control)>30)
    {
        //if(squar_count==0) control_time=0;
        //if(squar_count==1)
        control_time=0;
        cross_time=0;
        auto_command.linear.x = 0.0;
        auto_command.linear.y = 0;
        auto_command.linear.z = pid_high;
        auto_command.angular.x = 0;
        auto_command.angular.y = 0;
        auto_command.angular.z = -pid_angle;
        auto_control.publish(auto_command);
    }
    
    if(fabs(angle_erro_control)<30)
    {
    
        if(squar_count==2&&cross_time>30)//1
        {
            cross_flag=true;
        }
        
        if(ready_for_land==false)
        {
            if(ready_for_cross==false&&cross_flag==true) land_flag=true;
            auto_command.linear.x = 0.20;//0.08
            auto_command.linear.y = pid_dis;
            auto_command.linear.z = pid_high;
            auto_command.angular.x = 0;
            auto_command.angular.y = 0;
            auto_command.angular.z = -pid_angle;
            auto_control.publish(auto_command);
            land_time=0;
        }
        
        //ROS_ERROR("control_time:  %d\n",control_time);
        if(play_time>60&&land_flag==false)//2
        {
            turn_flag=true;
            play_time=0;
           // ROS_INFO("kaiqi22222222222222222222222222\n");
        }
        
        if(angle_change_count==2&&control_time>50)//2
        {
           aim_high=0.8;
           camera_angle=-40;
           drop_flag=true;
        }
        else
        {
        drop_flag=false;
        }
        
        if(drop_flag==true&&(ardrone_height-0.8)>0.2)//???
        {
            auto_command.linear.x = -0.20;//0.08
            auto_command.linear.y = pid_dis;
            auto_command.linear.z = pid_high;
            auto_command.angular.x = 0;
            auto_command.angular.y = 0;
            auto_command.angular.z = -pid_angle;
            auto_control.publish(auto_command);
           // ROS_ERROR("DDDDDDDDDDDDDDDDDDDDDDDDDDDD\n");
        }
        if(ready_for_land==true&&land_wait_time>60)//4
        {
            auto_command.linear.x = -0.1;//0.08
            auto_command.linear.y = pid_dis;
            auto_command.linear.z = 0;
            auto_command.angular.x = 0;
            auto_command.angular.y = 0;
            auto_command.angular.z = -pid_angle;
            auto_control.publish(auto_command);
            autoswitch=false;
            if(autoswitch==false) //autoswith???
            {
                pub_land.publish(emp_msg);
                down_flag=true;
                ROS_ERROR("fly_time :                %d",fly_time/20);
                ROS_ERROR("-------------------DOWN-------------------\n");
            }
        }
    }
}

void keyversion()
{
/************必须开小写键盘！！！*************************************/
    int kfd = 0;
	char c;
	struct termios cooked, raw;

	struct pollfd ufd;
    	ufd.fd = kfd;
    	ufd.events = POLLIN;
	int num;
	tcgetattr(kfd, &cooked);
 	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &=~ (ICANON | ECHO);
        raw.c_cc[VEOL] = 1;
        raw.c_cc[VEOF] = 2;
        tcsetattr(kfd, TCSANOW, &raw);

    for(;;)
   	{
		if(threadrosout==true)break;
        	boost::this_thread::interruption_point();

       	 	// get the next event from the keyboard
        int num;
        num = poll(&ufd, 1, 250);
		//ROS_INFO("num : %d",num);

        	if (num< 0)
        	{
            		perror("poll():");
			//continue;
            		//return;
        	}
        	else if(num > 0)
        	{
            		if(read(kfd, &c, 1) < 0)
            		{
                		perror("read():");
				//continue;
                		//return;
            		}
        	}
        	else
        	{
  	          continue;
       	 	}

        	switch(c)
       	 	{
           case 't'://take off
#ifdef keytest
			ROS_INFO("key command:take off\n");
#endif

			pub_control.publish(fly_pause);
			pub_takeoff.publish(emp_msg);
			break;
		case 'y'://land
#ifdef keytest
			ROS_INFO("key command:land\n");
#endif
                        autoswitch=false;
			pub_land.publish(emp_msg);
			break;
           	case 'w':
#ifdef keytest
			ROS_INFO("key command:flay_up\n");
#endif
			pub_control.publish(fly_up);
               		break;
            	case 's':
#ifdef keytest
			ROS_INFO("key command:flay_down\n");
#endif
			pub_control.publish(fly_down);
                	break;
            	case 'a':
#ifdef keytest
			ROS_INFO("key command:flay_turn_left\n");
#endif
			pub_control.publish(fly_turn_left);
                	break;
            	case 'd':
#ifdef keytest
			ROS_INFO("key command:flay_turn_right\n");
#endif
			pub_control.publish(fly_turn_right);
                	break;
            	case 'i':
#ifdef keytest
			ROS_INFO("key command:flay_forward\n");
#endif
			pub_control.publish(fly_forward);
                	break;
            	case 'k':
#ifdef keytest
			ROS_INFO("key command:flay_backward\n");
#endif
			pub_control.publish(fly_backward);
                	break;
            	case 'j':
#ifdef keytest
			ROS_INFO("key command:flay_left\n");
#endif
			pub_control.publish(fly_left);
                	break;
            	case 'l':
#ifdef keytest
			ROS_INFO("key command:flay_right\n");
#endif
			pub_control.publish(fly_right);
                	break;
            	case ' ':
#ifdef keytest
			ROS_INFO("key command:test kongge\n");
#endif
			pub_control.publish(fly_pause);
                    break;
            case 'z':
#ifdef keytest
        ROS_INFO("key command:test eland\n");
#endif
        pub_eland.publish(emp_msg);
                break;
            case 'b':
#ifdef keytest
            ROS_INFO("key command:now auto controll\n");
#endif
            statictime=QRtime;
            if(autoswitch==false)
            {
                autoswitch = true;
                fly_time=0;
            }
            else
            {
                autoswitch = false;
                pub_control.publish(fly_pause);
            }
            break;
        	}

        }
    }
int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_takeoff");
	boost::thread rosway(&rosversion);
   	boost::thread camway(&camerause);
	boost::thread keyway(&keyversion);
	rosway.join();
	keyway.join();
    	camway.join();//
	return 0;
}//main

