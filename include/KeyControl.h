#include <sys/poll.h>
#include <termios.h>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include "Parameter.h"

class KeyControl
{
private:
    void controlMain(std::string key);

public:
    KeyControl();
    ~KeyControl();
    void keyMain();

};
