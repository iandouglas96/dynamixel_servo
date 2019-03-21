#include "dynamixel_servo/servo.h"

Servo::Servo(const char* port, int rate) {
  ser = open(port, O_RDWR);
  
  //This config is taken from
  //https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/

  // Create new termios struct, we call it 'tty' for convention
  struct termios tty;
  memset(&tty, 0, sizeof tty);
  
  // Read in existing settings, and handle any error
  if(tcgetattr(ser, &tty) != 0) {
    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
  }

  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 9600
  cfsetispeed(&tty, rate);
  cfsetospeed(&tty, rate);

  // Save tty settings, also checking for error
  if (tcsetattr(ser, TCSANOW, &tty) != 0) {
    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  }
}

void Servo::setPosition(int pos, int speed) {
  unsigned char bottom_pos = pos & 0xff;
  unsigned char top_pos = (pos >> 8) & 0x0f;
  unsigned char bottom_speed = speed & 0xff;
  unsigned char top_speed = (speed >> 8) & 0x0f;

  unsigned char msg[] = {0x01, 0x07, 0x03, 0x1e,
    bottom_pos, top_pos, bottom_speed, top_speed};
  sendMessage(msg, sizeof(msg));
  getReply(msg);
}

int Servo::getPosition() {
  //Read 2 bytes, starting at 0x24 (current position)
  unsigned char msg[] = {0x01, 0x04, 0x02, 0x24, 0x02};
  sendMessage(msg, sizeof(msg));
  unsigned char reply[20];
  int len = getReply(reply);
  if (len == 4) {
    return reply[1] + ((reply[2]&0x0f)<<8);
  }
  return -1;
}

//Read the expected reply from the buffer, and throw it away.
//Do this to keep the buffer clean
int Servo::getReply(unsigned char *msg) {
  read(ser, msg, 4);
  if (msg[0] == 0xff && msg[1] == 0xff) { 
    unsigned char len = msg[3];
    read(ser, msg, len);
    return len;
  }
  return -1;
}

void Servo::sendMessage(unsigned char* msg, int len) {
  unsigned char full_msg[len+3];
  //Message prefix
  full_msg[0] = 0xff;
  full_msg[1] = 0xff;

  unsigned char checksum = 0x00;
  for (int i=0; i<len; i++) {
    checksum += msg[i];
    full_msg[i+2] = msg[i];
  }

  //checksum is the sum of message bytes, bitwise inverted
  full_msg[len+2] = ~checksum;

  write(ser, full_msg, sizeof(full_msg));
  
  //Read back what we wrote, due to RX and TX tied together
  read(ser, &full_msg, sizeof(full_msg));
}
