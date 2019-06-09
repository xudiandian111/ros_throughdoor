//包含对ubuntu系统的操作
#include "Common.h"

int Common::control_time = 0;
int Common::cross_time = 0;
int Common::land_time = 0;
int Common::land_wait_time = 0;
int Common::play_time = 0;
int Common::QRtime = 100;
int Common::fly_time = 0;

Common::Common(void)
{
}
Common::~Common(void)
{
}
void Common::sigalrm_handler(int sig)
{
    play_time++;
    QRtime++;
    control_time++;
    fly_time++;
    cross_time++;
    land_time++;
    land_wait_time++;
}

void Common::set_timer() //定时器
{
    signal(SIGALRM, sigalrm_handler);

    struct itimerval itv; //linux下结构体
    itv.it_value.tv_sec = 0;
    itv.it_value.tv_usec = 50000;

    itv.it_interval.tv_sec = 0;
    itv.it_interval.tv_usec = 50000;

    setitimer(ITIMER_REAL, &itv, NULL);
}