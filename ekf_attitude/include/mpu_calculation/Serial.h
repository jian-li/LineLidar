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
//#include "ArucoLanding/marker_pose.h"
//#include "marker_pose.h"
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point32.h>
#include <iostream>

#include <queue>

using namespace std;
class SerialHandler {
  
  #define PKGSIZE 22
  
public:
	SerialHandler();
	~SerialHandler();
	int portsign;
	int configStatu;
	unsigned char ch[30];
	unsigned char readbuff[512];
	int readByte;
	int readLen;
	int isReadOK;
	uint8_t x_output;
	speed_t speed;
	fd_set rd;
	fd_set wd;
	int Write_Priority;
	void init();
	char *DEV;
	void Read();
	//void WritePose(double * res);
        //void WritePose(const ArucoLanding::marker_pose::ConstPtr& msg);
	int ConfigPort(int fd);
    void Serial_Close();
    //void writeDestPos(const geometry_msgs::Pose::ConstPtr& msg);
    void writeCmd(int16_t PH,int16_t ZX,int16_t CS);

    //void writeMotionz(const geometry_msgs::Point32::ConstPtr& motionz_msg);

    //void writeTestPos(const geometry_msgs::Pose msg);
  queue<uint8_t> fifo;
  bool needclean;
};
