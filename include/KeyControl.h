#pragma once
#include <sys/poll.h>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include "Parameter.h"
#include <termio.h>
class KeyControl
{
private:
    void controlMain(std::string key);
    char getKey();

public:
    KeyControl();
    ~KeyControl();
    void keyMain();

};
