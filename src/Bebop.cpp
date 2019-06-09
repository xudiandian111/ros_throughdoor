//包含对BeBop信息读取和控制的操作
//control和controlMain两个函数需要进行合并　Ｍａｉｎ中函数封装到Ｃｏｎｔｒｏｌ类中
#include "Bebop.h"

#define keytest

Control getControl;
Detections getDetections;
Common getCommon;

bool Bebop::down_flag = false;
bool Bebop::drop_flag = false;
/*********autoswitch**************/
bool Bebop::autoswitch = false;
double Bebop::ardrone_speed = 0.0f;
double Bebop::ardrone_height = 0.0f;
double Bebop::ardrone_z_attitude = 0.0f;
int Bebop::statictime = 0;
bool Bebop::threadrosout = false;
bool Bebop::threadkeyout = false;

Bebop::Bebop(void)
{

}

Bebop::~Bebop(void)
{
}

void Bebop::readHeight(const bebop_msgs::Ardrone3PilotingStateAltitudeChanged::ConstPtr &msg) //高度变化
{
    ardrone_height = 0.0f;
    ardrone_height = msg->altitude;
}

void Bebop::readAngle(const bebop_msgs::Ardrone3PilotingStateAttitudeChanged::ConstPtr &msg) //航向角
{
    ardrone_z_attitude = 0.0f;
    ardrone_z_attitude = msg->yaw;
}

void Bebop::readSpeed(const bebop_msgs::Ardrone3PilotingStateSpeedChanged::ConstPtr &msg)//速度读取
{
    ardrone_speed = 0.0f;
    ardrone_speed = msg->speedX;
}

void Bebop::initFlyParam() //参数初始化
{
    autoswitch = false;
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

void Bebop::cameraAngleControl() //相机角度控制
{
    change_camera.angular.y = -90;
    change_camera.angular.z = 0;
    camera_control.publish(change_camera);
}

void Bebop::generalControl() //常规控制
{
    double control_parameter[2][3]={0,0,0,0,0,0};
    /*
    速度数组　可以接受值的范围[-1~1]
    control_parameter[0][0~2]代表无人机平动x y z轴的速度
    control_parameter[1][0~2]代表无人机旋转x y z轴的速度，通常只用ｚ轴
    */
    double erro_angle = 180 / 3.1415926 * Detections::aim_angle;
    if(erro_angle>25)
    {
        if(Parameter::turn>2)
        {
            Parameter::turn_left_flag = true;
            Parameter::turn_time = 0;
            Parameter::turn = 0;
        }
        else
        {
            Parameter::turn++;
            ROS_ERROR("%d", Parameter::turn);
        }
        
    }
    if(erro_angle<5)
    {
        Parameter::turn = 0;
    }

    if (Detections::flag_circle == true && Parameter::is_cross == false)
    {
        if((fabs(Detections::circle_point.x-320)<20) && (fabs(Detections::circle_point.y-270)<20))
        {
            ROS_ERROR("land !!!");
            getControl.controlVdownCircle(control_parameter);
            Parameter::is_cross = true;
            Parameter::fly_time = 0;
        }
        else
        {
            ROS_ERROR("circle");
            getControl.controlVfindCircle(control_parameter);
        }
    }
    else if(Parameter::special_flag==true)
    {
        getControl.specialControl(1, control_parameter);
    }
    else if (Parameter::turn_left_flag == true)
    {
        getControl.controlTurnLeft(control_parameter);
        ROS_ERROR("turn_left%lf,%d",erro_angle,Parameter::turn_time);
    }
    else if(Parameter::special_flag == true)
    {
        getControl.specialControl(1, control_parameter);//特殊控制 待调试 盲转向
        ROS_INFO("angle:%lf\n", ardrone_z_attitude);
    }
    else
    {
        getControl.controlGeneral(control_parameter);
        if (Detections::flag_circle == true)
        {
            Parameter::fly_time++;
        }
        else if (Detections::flag_circle == false && Parameter::fly_time > 5)
        {
            Parameter::is_cross = false;
            Parameter::fly_time = 0;
            Control::aim_high = 1.0;
        }
    }
    if(Parameter::land_flag == true)
    {
        pub_land.publish(emp_msg);
    }
    else
    {
        auto_command.linear.x = control_parameter[0][0];
        auto_command.linear.y = control_parameter[0][1];
        auto_command.linear.z = control_parameter[0][2];
        auto_command.angular.x = control_parameter[1][0];
        auto_command.angular.y = control_parameter[1][1];
        auto_command.angular.z = control_parameter[1][2];
        auto_control.publish(auto_command);
    }
    
#ifdef TEST
    ROS_INFO("linear.x:%lf,linear.y:%lf,linear.z:%lf,angular.z:%lf/n", control_parameter[0][0], control_parameter[0][1], control_parameter[0][2], control_parameter[1][2]);
#endif

}

void Bebop::rosVersion()
{
    ros::NodeHandle node;
    ros::Rate loop_rate(20);
    image_transport::ImageTransport it(node);
    /************publish init*******************************/
    pub_takeoff = node.advertise<std_msgs::Empty>("/bebop/takeoff", 1);
    pub_land = node.advertise<std_msgs::Empty>("/bebop/land", 1);
    pub_control = node.advertise<geometry_msgs::Twist>("/bebop/cmd_vel", 1);
    auto_control = node.advertise<geometry_msgs::Twist>("/bebop/cmd_vel", 1);
    pub_eland = node.advertise<std_msgs::Empty>("/bebop/reset", 1);
    image_sub = it.subscribe("/bebop/image_raw", 10, &Detections::imagePcs, &getDetections);
    angle = node.subscribe("/bebop/states/ardrone3/PilotingState/AttitudeChanged", 1, Bebop::readAngle);
    height = node.subscribe("/bebop/states/ardrone3/PilotingState/AltitudeChanged", 1, Bebop::readHeight); //无人机高度变化。海拔高度为起飞点以上的海拔高度。
    speed = node.subscribe("/bebop/states/ardrone3/PilotingState/SpeedChanged", 1, Bebop::readSpeed);
    camera_control = node.advertise<geometry_msgs::Twist>("/bebop/camera_control", 1);
    Bebop::initFlyParam();
    getCommon.set_timer();
    ROS_INFO("init ok! 't' take off, 'y' land, 'z' stop\n");
    while (ros::ok())
    {
        Bebop::cameraAngleControl(); //bebop摄像头转动函数
        if (autoswitch)
        {

            //Bebop::controlMain();
            Bebop::generalControl();
        }
        if (down_flag)
        {
            pub_land.publish(emp_msg);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    threadrosout = true;
}



void Bebop::keyVersion()
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

    for (;;)
    {
        if (threadrosout == true)
            break;
        boost::this_thread::interruption_point();

        // get the next event from the keyboard
        int num;
        num = poll(&ufd, 1, 250);
        //ROS_INFO("num : %d",num);

        if (num < 0)
        {
            perror("poll():");
            //continue;
            //return;
        }
        else if (num > 0)
        {
            if (read(kfd, &c, 1) < 0)
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

        switch (c)
        {
        case 't': //take off
#ifdef keytest
            ROS_INFO("key command:take off\n");
#endif
            pub_control.publish(fly_pause);
            pub_takeoff.publish(emp_msg);
            break;
        case 'y': //land
#ifdef keytest
            ROS_INFO("key command:land\n");
#endif
            autoswitch = false;
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
        case ' '://悬停
#ifdef keytest
            ROS_INFO("key command:test kongge\n");
#endif
            pub_control.publish(fly_pause);
            break;
        case 'z'://紧急暂停
#ifdef keytest
            ROS_INFO("key command:test eland\n");
#endif
            pub_eland.publish(emp_msg);
            break;
            case '=':
              y+=1;
              change_camera.angular.y=y;
              change_camera.angular.z=0;
              camera_control.publish(change_camera);
        	  break;
            case '-':
              y-=1;
              change_camera.angular.y=y;
              change_camera.angular.z=0;
              camera_control.publish(change_camera);
        	  break;
        case 'm': //测试
#ifdef keytest
            if (Parameter::special_flag == false)
            {
                ROS_INFO("key command:cross\n");
            }
            else
            {
                ROS_INFO("key command:gogogo\n");
            }
#endif
            if (Parameter::special_flag == false)
            {
                Parameter::special_flag = true;
            }
            else
            {
                Parameter::special_flag = false;
            }
            break;
        case 'b':
#ifdef keytest
            if(autoswitch == false)
            {
                ROS_INFO("key command:now auto controll\n");
            }
            else
            {
                ROS_INFO("key command:now close auto controll\n");
            }
#endif
            statictime = getCommon.QRtime;
            if (autoswitch == false)
            {
                autoswitch = true;
                getCommon.fly_time = 0;
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
