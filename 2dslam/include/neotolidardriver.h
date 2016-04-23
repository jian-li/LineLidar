#include <iostream>
#include <serial/serial.h>
#include <sys/time.h>
#include <ros/ros.h>

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;

#define XV11_PACKAGE_LENGTH	22
#define XV11_START_BYTE	0xFA
#define PI 3.1415926

namespace LineLidar
{
class NeotoDriver
{
public:
  NeotoDriver();
  ~NeotoDriver();

  void ParsePackage ( uint8_t * packagePointer );
  void LoadPackage ( uint8_t * packagePointer );
  uint16_t PackageChecksum ( uint8_t * packagePointer );
  void SyncUp();
  void run();
private:
  // int package_len;
  ros::Publisher scan_pub;
  timeval starttime,endtime;
  uint8_t * packagePointer;
  uint8_t *  XV11_Package;
  uint16_t * Distance;
  uint16_t pointreceived;
  serial::Serial serport;
  uint16_t GoodReadings, BadReadings,AnglesCovered;

};


}
