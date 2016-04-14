#include <iostream>
#include <ros/ros.h>
#include <neotolidardriver.h>
#include <serial/serial.h>
#include <sys/time.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>

namespace LineLidar
{

NeotoDriver::NeotoDriver()
{
    ros::NodeHandle nh;
    scan_pub = nh.advertise<sensor_msgs::LaserScan>("/base_scan", 10);
    pointreceived = 0;

    XV11_Package = new uint8_t[22];
    BadReadings = 0;
    GoodReadings = 0;
    AnglesCovered = 0;
    Distance = new uint16_t[360];
    try
    {
        //打开串口0
        serport.setPort("/dev/ttyUSB0");
        serport.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(300);

        serport.setTimeout(to);
        serport.open();

    }
    catch(serial::IOException & e)
    {
        ROS_ERROR_STREAM("Unable to open serial port");
        return ;
    }

    //判断串口是否打开
    if(serport.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        return ;
    }
}

NeotoDriver::~NeotoDriver()
{

}

uint16_t NeotoDriver::PackageChecksum(uint8_t * packagePointer)
{
    uint8_t i;
    uint16_t data[10];
    uint16_t checksum;
    uint32_t chk32;

    // group the data by word, little-endian
    for (i = 0; i < 10; i++)
    {
        data[i] = packagePointer[2*i] | (((uint16_t)packagePointer[2*i+1]) << 8);
    }

    // compute the checksum on 32 bits
    chk32 = 0;
    for (i = 0; i < 10; i++)
    {
        chk32 = (chk32 << 1) + data[i];
    }

    // return a value wrapped around on 15bits, and truncated to still fit into 15 bits
    checksum = (chk32 & 0x7FFF) + ( chk32 >> 15 ); // wrap around to fit into 15 bits
    checksum = checksum & 0x7FFF; // truncate to 15 bits
}

void NeotoDriver::ParsePackage(uint8_t * packagePointer)
{
//    pointreceived = pointreceived + 4;
    uint16_t i;
    uint16_t Index;
    uint8_t Speed;
    uint8_t InvalidFlag[4];
    uint8_t WarningFlag[4];
    uint16_t Checksum, ChecksumCalculated;

    Checksum = ((uint16_t)packagePointer[21] << 8) | packagePointer[20];
    ChecksumCalculated = PackageChecksum(packagePointer);
    if (Checksum != ChecksumCalculated)
    {
        BadReadings += 4;
    }

    Index = (packagePointer[1] - 0xA0) * 4;
    Speed = ((uint16_t)packagePointer[3] << 8) | packagePointer[2];
    InvalidFlag[0] = (packagePointer[5] & 0x80) >> 7;
    InvalidFlag[1] = (packagePointer[9] & 0x80) >> 7;
    InvalidFlag[2] = (packagePointer[13] & 0x80) >> 7;
    InvalidFlag[3] = (packagePointer[17] & 0x80) >> 7;
    WarningFlag[0] = (packagePointer[5] & 0x40) >> 6;
    WarningFlag[1] = (packagePointer[9] & 0x40) >> 6;
    WarningFlag[2] = (packagePointer[13] & 0x40) >> 6;
    WarningFlag[3] = (packagePointer[17] & 0x40) >> 6;

//    std::cout << "Index is " << Index << std::endl;
    if (Index == 0)
    {

        sensor_msgs::LaserScan scan;
        const float laser_frequency = 5.7;
        const int number_reading = 360;
        scan.header.stamp = ros::Time::now();
        scan.header.frame_id = "base_laser";
        scan.angle_min = -PI;
        scan.angle_max = PI;
        scan.angle_increment = PI * 2 / number_reading;
        scan.time_increment = (1 / laser_frequency) / (number_reading);
        scan.range_min = 0.0;
        scan.range_max = 100.0;
        scan.ranges.resize(number_reading);
        scan.intensities.resize(number_reading);

        AnglesCovered = 0;
        for (i = 0; i < 360; i++)
        {
            if (Distance[i] > 0)
            {
                scan.ranges[i] = Distance[i] * 1.0/1000;
                scan.intensities[i] = 1.0;
                AnglesCovered++;
            }
        }




        scan_pub.publish(scan);


        GoodReadings = 0;
        BadReadings = 0;
    }

    for (i = 0; i < 4; i++)
    {
        if (!InvalidFlag[i])
        {
            Distance[Index+i] = packagePointer[4+(i*4)] | ((uint16_t)(packagePointer[5+(i*4)] & 0x3F) << 8);
            GoodReadings++;
//            std::cout << Distance[Index+i] << std::endl;
        }
        else
        {
            Distance[Index+i] = 0;
            BadReadings++;
        }
    }

}

void NeotoDriver::LoadPackage(uint8_t * packagePointer)
{
//    std::cout << "start reading data" << std::endl;

    uint8_t i = 0;
//    uint8_t ch;
    serport.read(packagePointer, XV11_PACKAGE_LENGTH);
}

void NeotoDriver::SyncUp()
{
    uint8_t i;
    uint8_t in_buff[1];
    uint8_t ch = 0;

    while ( in_buff[0] != XV11_START_BYTE)
    {
        serport.read(in_buff, 1);
        std::cout <<  std::hex << (int)in_buff[0] << std::endl;
    }

    // read the rest
    i = XV11_PACKAGE_LENGTH - 1;
    while (i > 0)
    {
        serport.read(in_buff,1);
        ch = in_buff[0];
        if (ch >= 0) i--;
    }

}

void NeotoDriver::run()
{
//    while (Serial_Buffer_Count() < XV11_PACKAGE_LENGTH);

    LoadPackage(XV11_Package);

//    std::cout << "load package finished" << std::endl;
    if (XV11_Package[0] != XV11_START_BYTE)
    {
//        std::cout << "bad start header" << std::endl;
        SyncUp();
    }
    else
    {
//        std::cout << "start parsing package" << std::endl;
        ParsePackage(XV11_Package);
    }

}
}


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "neotolidardriver");



    LineLidar::NeotoDriver neotodriver;
    ros::Rate _spin_rate(6000);
    while(ros::ok())
    {
        ros::spinOnce();
        neotodriver.run();

        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "base_laser"));


        _spin_rate.sleep();
    }
    return 0;
}
