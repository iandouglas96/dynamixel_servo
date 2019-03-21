#ifndef SERVO_H
#define SERVO_H

//C Serial stuff
#include <stdio.h>
#include <string.h>
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

class Servo {
  public:
    Servo(const char* port, int rate);
    void setPosition(int pos, int speed);
    int getPosition();
  private:
    int ser;
    void sendMessage(unsigned char* msg, int len);
    int getReply(unsigned char *msg);
};

#endif //SERVO_H
