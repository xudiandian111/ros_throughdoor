#pragma once
//ubuntu system lib
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <signal.h>
#include <sys/poll.h>
#include <termios.h>


class Common
{
public:
    Common(void);
    ~Common(void);

public:
    static void sigalrm_handler(int sig);
    void set_timer();
    static int control_time;
    static int cross_time;
    static int land_time;
    static int land_wait_time;
    static int play_time;
    static int QRtime;
    static int fly_time;
};