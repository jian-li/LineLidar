#include <ros/ros.h>
#include <iostream>
#include <string>
//#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <mpu_calculation.h>
#include <pixhawk_ekf.h>
#include <Serial.h>
#include <tf/transform_broadcaster.h>


#define PI 3.14159f
#define Tesla2Gauss 1.0f
#define MPU9250M_4800uT   ((float)0.6f)            // 0.6 uT/LSB
float mag_offset[3]={-71.4387,-37.9589,111.7972};
float mag_sca[9]={0.9824,-0.0149,0.0120,-0.0149,0.9845,0.0219,0.0120,0.0219,0.9445};
float MagTrueX,MagTrueY,MagTrueZ;
float roll_init=0.0,pitch_init=0.0,yaw_init=0.0;
SerialHandler m_serial;


bool parseimumsg(uint8 * dataPacket, sensor_msgs::Imu * imu_msg)
{
    float m1=0.0,m2=0.0,m3=0.0;
    //
    if(dataPacket[0] == 0x55 && dataPacket[1] == 0xaa)
    {
        //检验校验和
        unsigned int sum = 0;
        for(int i = 0; i < PKGSIZE - 1; i++)
        {
            sum += dataPacket[i];
        }
        //std::cout<< sum  << " " << imumsg[14] << std::endl;
        //
        if( sum % 256 == dataPacket[PKGSIZE - 1])
        {
            //解算加速度X,Y,Z
            int16 AccX = Byte16(int16, dataPacket[2], dataPacket[3]);
            int16 AccY = Byte16(int16, dataPacket[4], dataPacket[5]);
            int16 AccZ = Byte16(int16, dataPacket[6], dataPacket[7]);
            float Accscalar = MPU9250A_4g * GRAVITY_MSS;
            float AccTrueX = AccX * Accscalar;
            float AccTrueY = AccY * Accscalar;
            float AccTrueZ = AccZ * Accscalar;

            //解算角速度X,Y,Z

            int16 GyrX = Byte16(int16, dataPacket[8], dataPacket[9]);
            int16 GyrY = Byte16(int16, dataPacket[10], dataPacket[11]);
            int16 GyrZ = Byte16(int16, dataPacket[12], dataPacket[13]);
            float Gyrscalar = UserGyrResolution / 57.3;
            float GyrTrueX = GyrX * Gyrscalar;
            float GyrTrueY = GyrY * Gyrscalar;
            float GyrTrueZ = GyrZ * Gyrscalar;

            //解算Mag X,Y,Z
            int16 MagX = Byte16(int16, dataPacket[15], dataPacket[14]);
            int16 MagY = Byte16(int16, dataPacket[17], dataPacket[16]);
            int16 MagZ = Byte16(int16, dataPacket[19], dataPacket[18]);

            float Magscalar = MPU9250M_4800uT*Tesla2Gauss;
            float magx=MagX*Magscalar;
            float magy=MagY*Magscalar;
            float magz=MagZ*Magscalar;

            m1=magx+mag_offset[0];			m2=magy+mag_offset[1];			m3=magz+mag_offset[2];
            MagTrueY=mag_sca[0]*m1+mag_sca[1]*m2+mag_sca[2]*m3;
            MagTrueX=mag_sca[3]*m1+mag_sca[4]*m2+mag_sca[5]*m3;
            MagTrueZ=-1.0*(mag_sca[6]*m1+mag_sca[7]*m2+mag_sca[8]*m3);

            newDataMag =(bool) dataPacket[20];

            //发送Imu数据
            imu_msg->header.stamp = ros::Time::now();
            imu_msg->header.frame_id = "imu";
            imu_msg->angular_velocity.x = GyrTrueX;
            imu_msg->angular_velocity.y = GyrTrueY;
            imu_msg->angular_velocity.z = GyrTrueZ;

            imu_msg->linear_acceleration.x = AccTrueX;
            imu_msg->linear_acceleration.y = AccTrueY;
            imu_msg->linear_acceleration.z = AccTrueZ;

            return true;
        }
        else
        {
            ROS_INFO_STREAM("Checksum Error");
            return false;
        }
    }
    else
    {
        ROS_INFO_STREAM("Header Error");
        return false;
    }
}


void GetInitAng(sensor_msgs::Imu * imu_msg)
{
    float Gabs=10.0;
    float roll=0.0,pitch=0.0,yaw=0.0,Hx=0.0,Hy=0.0;
    float AccTrueX  = imu_msg->linear_acceleration.x;       // g/LSB
    float AccTrueY  = imu_msg->linear_acceleration.y;       // g/LSB
    float AccTrueZ  = imu_msg->linear_acceleration.z;       // g/LSB


    Gabs=sqrt(AccTrueX*AccTrueX+AccTrueY*AccTrueY+AccTrueZ*AccTrueZ);
    roll=atan2(AccTrueY,AccTrueZ);
    pitch=atan2(-1.0*AccTrueX,AccTrueZ);
//    pitch=asin(AccTrueX/Gabs);

    Hx=cos(pitch)*MagTrueX+sin(pitch)*sin(roll)*MagTrueY+sin(pitch)*cos(roll)*MagTrueZ;
    Hy=cos(roll)*MagTrueY-sin(roll)*MagTrueZ;

    if((Hy > -2)&& (Hy < 2))
    {
        if(Hx > 0)yaw=PI/2;
        else yaw=3*PI/2;
    }
    else
    {
        if((Hx > 0)&&(Hy > 0))yaw=atan(Hx/Hy);
        else if((Hx < 0)&&(Hy > 0))yaw=atan(Hx/Hy)+2*PI;
        else yaw=atan(Hx/Hy)+PI;
    }
    yaw=yaw-Local_MagDeclination; //本地磁偏角为-6°43′
    if(yaw < 0)yaw=yaw+2*PI;
    roll_init+=roll;pitch_init+=pitch;yaw_init+=yaw;
}


int main(int argc, char ** argv)
{ 
    ros::init(argc, argv, "mpu_calculate");
    ros::NodeHandle nh;

    //ros::Subscriber imu_data = nh.subscribe();
    ros::Publisher imuPub = nh.advertise<sensor_msgs::Imu>("imudata",1);
    //串口读取数据
    //    serial::Serial serport;

    //    try
    //    {
    //        //打开串口0
    //        serport.setPort("/dev/ttyUSB0");
    //        serport.setBaudrate(115200);
    //        serial::Timeout to = serial::Timeout::simpleTimeout(0);

    //        serport.setTimeout(to);
    //        serport.open();

    //    }
    //    catch(serial::IOException & e)
    //    {
    //        ROS_ERROR_STREAM("Unable to open serial port");
    //        return -1;
    //    }

    //    //判断串口是否打开
    //    if(serport.isOpen())
    //    {
    //        ROS_INFO_STREAM("Serial Port initialized");
    //    }
    //    else
    //    {
    //        return -1;
    //    }

    //    serport.flushInput();


    //uint8 dataPacket[PKGSIZE];
    m_serial.init();

    sensor_msgs::Imu imu_msg;

    //判断是否已经标定过
    bool calibrated = false;
    double zeroshift[3];
    int times = 0;

    ROS_INFO_STREAM("Reading from serial port");
    std::cout << "start calibrate gyro ";

    ros::Rate loop_rate(200);
    double starttime, endtime;

    while(ros::ok())
    {
        starttime = ros::Time::now().toSec();
        ros::spinOnce();

        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "imu"));

        //        int datasize = serport.read(dataPacket, packetLen);
        m_serial.Read();

        if (m_serial.isReadOK == 1)
        {
            m_serial.isReadOK = 0;
            if(parseimumsg(m_serial.readbuff, &imu_msg) == true)
            {

                //解析imu的datapacket
                if(!calibrated)
                {
                    ROS_INFO_STREAM("Calibrating");
                    if(times == 200)
                    {
                        std::cout << std::endl;
                        calibrated = true;
                        zeroshift[0] /= 200;
                        zeroshift[1] /= 200;
                        zeroshift[2] /= 200;
                        Global_Initialise(roll_init/200.0,pitch_init/200.0,yaw_init/200.0);
                    }
                    else
                    {
                        zeroshift[0] += imu_msg.angular_velocity.x;
                        zeroshift[1] += imu_msg.angular_velocity.y;
                        zeroshift[2] += imu_msg.angular_velocity.z;
                        GetInitAng(&imu_msg);
                        std::cout << ".";
                    }
                    times += 1;
                }
                else
                {
                    imu_msg.angular_velocity.x -= zeroshift[0];
                    imu_msg.angular_velocity.y -= zeroshift[1];
                    imu_msg.angular_velocity.z -= zeroshift[2];
//                    std::cout << "--------------------------------" << std::endl;
//                    std::cout << imu_msg.angular_velocity.x <<"    "<< imu_msg.angular_velocity.y << "   " << imu_msg.angular_velocity.z << std::endl;
//                    std::cout << imu_msg.linear_acceleration.x <<"   "<< imu_msg.linear_acceleration.y <<"   "<< imu_msg.linear_acceleration.z << std::endl;

                    imuPub.publish(imu_msg);

                    pixhawk_ekf(imu_msg.angular_velocity.x,imu_msg.angular_velocity.y,imu_msg.angular_velocity.z,imu_msg.linear_acceleration.x,imu_msg.linear_acceleration.y,imu_msg.linear_acceleration.z);
                    //ROS_INFO_STREAM("OK!");
                }
            }
        }
        //发送数据

        //endtime = ros::Time::now().toSec();
        //std::cout << "time" << endtime - starttime << std::endl;
        loop_rate.sleep();
    }

}
