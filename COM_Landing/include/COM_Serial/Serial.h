#include  <unistd.h>     /*Unix 标准函数定义*/
#include  <sys/types.h> 
#include  <sys/stat.h> 
#include  <sys/ioctl.h> 
#include  <fcntl.h>      /*文件控制定义*/
#include  <termios.h>    /*PPSIX 终端控制定义*/
#include  <errno.h>      /*错误号定义*/
#include <sys/time.h>
#include <stdint.h>
#include <stdio.h>
#include "ArucoLanding/marker_pose.h"
#include <iostream>
using namespace std;
class SerialHandler {
public:
	SerialHandler();
	~SerialHandler();
	int portsign;
	int configStatu;
	unsigned char ch[30];
	unsigned char readbuff[512];
	int readByte;
	uint8_t x_output;
	speed_t speed;
	fd_set rd;
	fd_set wd;
	int Write_Priority;
	void init();
	char *DEV;
	void Read();
	void WritePose(double * res);
        void WritePose(const ArucoLanding::marker_pose::ConstPtr& msg);
	int ConfigPort(int fd);
	void Serial_Close();

};
