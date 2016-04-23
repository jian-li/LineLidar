#include "wheelOdometry.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "COM_Landing/cleanBot_msg.h"
#include "tf/transform_broadcaster.h"
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <VisualCompass/AngleMsg.h>
#include <VisualCompass/VisualComMsg.h>

#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>

//data filtering 
#define filterLen 10

using namespace message_filters;
using namespace std;
using namespace CleanBot;

//轮子的量化误差
const double deltaThr = 0.0014;

double filterdata[filterLen];
int receiveddata = 0;
int curIndex = 0;
double sum = 0;
double filterdAngle;
//INS sensors average filter method
float averageFilter(float newdata)
{
    ROS_INFO("filtering the data");
    //data have updated
    if(receiveddata < filterLen)
    {
        filterdata[curIndex] = newdata;
        sum += newdata;
        receiveddata += 1;
        curIndex += 1;
        return sum/receiveddata;

    }
    else
    {
        //remove the old data
        curIndex %= filterLen;
        sum -= filterdata[curIndex];
        sum += newdata;
        filterdata[curIndex] = newdata;
        curIndex += 1;
        return sum/filterLen;

    }
}

//INS sensors weighted filtering method
void weigtedSmooth()
{

}

//INS wavlet filtering method
void wavletSmooth()
{
    
}


void imuMsgCallback(const sensor_msgs::Imu& bot_msg);
void wheelMsgCallback(const COM_Landing::cleanBot_msg& bot_msg);
//void visualCompassCallback(const std_msgs::Float64& viscomangle);

// global data
wheelOdometry botOdomPose;

ros::Publisher imuAngle;
ros::Publisher imuAngleRaw;
ros::Publisher wheelAngle;

bool startflag = false;

double curtime = 0, lasttime = 0;

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "gyrOdometry");
    ros::NodeHandle n;
    //synchronize the message from the wheel and imu, use approximate time policies

    //subscribe robot message from low-level controller serial data
    ros::Subscriber imumsg_sub = n.subscribe("/imudata",1, imuMsgCallback);
    ros::Subscriber botmsg_sub = n.subscribe("/CleanBot/BotMsg",1, wheelMsgCallback);
 //   ros::Subscriber viscom_sub = n.subscribe("/VisualCompass/angle", 1, visualCompassCallback);

    // publish odometry message to rviz
    wheelAngle = n.advertise<VisualCompass::AngleMsg>("/WheelOdometry/angle", 10);
    imuAngle = n.advertise<VisualCompass::AngleMsg>("/Gyroscope/angle", 10);
    imuAngleRaw = n.advertise<std_msgs::Float64>("/imuAngleRaw", 10);

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/CleanBot/odom", 50);

    // tf transform
    tf::TransformBroadcaster odom_broadcaster;
    //Marker publisher
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);    


    cout<<"running..."<<endl;
    cout<<"subscribe to /CleanBot/BotMsg"<<endl;
    cout<<"publish to /CleanBot/odom"<<endl;
    
    visualization_msgs::Marker points, line_strip;
    
    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
   
    //point scale
    points.scale.x = 0.02;
    points.scale.y = 0.02;
    
    //line strip scale   
    line_strip.scale.x = 0.01;
 
    //points color is green  
    points.color.g = 1.0f;
    points.color.a = 1.0;

    //line strip color is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;    

    ros::Time current_time;
    ros::Rate loop_rate(100);

    while(ros::ok())
    {
		current_time = ros::Time::now();
   		points.header.stamp = current_time;
		points.header.frame_id = line_strip.header.frame_id = "odom";		
		points.ns = line_strip.ns = "Markers";
		
		points.action = line_strip.action = visualization_msgs::Marker::ADD;
		points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
			
		points.id = 0;
		line_strip.id = 1;
		
		
		// publish transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";
		// evaluate pose (position and rotation)
		odom_trans.transform.translation.x = botOdomPose.mPosX;
		odom_trans.transform.translation.y = botOdomPose.mPosY;
		odom_trans.transform.translation.z = 0.0;
		// create quaternion from yaw angle for rotation
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(botOdomPose.mTheta);
		odom_trans.transform.rotation = odom_quat;
		
		// send the transform
		odom_broadcaster.sendTransform(odom_trans);
		
		
		// publish odometry message over ROS
		nav_msgs::Odometry odom;
		// set position
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";
		odom.pose.pose.position.x = botOdomPose.mPosX;
		odom.pose.pose.position.y = botOdomPose.mPosY;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;
		// set velocity
		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = botOdomPose.mVelX;
		odom.twist.twist.linear.y = botOdomPose.mVelY;
		odom.twist.twist.angular.z = botOdomPose.mVelTheta;
	   	
		//add points
		geometry_msgs::Point p;
		p.x = botOdomPose.mPosX;
		p.y = botOdomPose.mPosY;
		p.z = 0.0;
		
		points.points.push_back(p);
		line_strip.points.push_back(p);		
	
		// publish message
		odom_pub.publish(odom);
		//publish the marker
		marker_pub.publish(points);
		marker_pub.publish(line_strip);
		
		//ROS_INFO("%5.3f %5.3f %5.3f",botOdomPose.mPosX,botOdomPose.mPosY,botOdomPose.mTheta);
		
		ros::spinOnce();
		loop_rate.sleep();
    }
    return -1;
}

//imu message callback 
void imuMsgCallback(const sensor_msgs::Imu& imu_msg)
{

    std_msgs::Float64 dthraw;
    dthraw.data = imu_msg.angular_velocity.z * 0.01; 
    imuAngleRaw.publish(dthraw);

    filterdAngle = averageFilter(imu_msg.angular_velocity.z);

    if(startflag == false)
    {
        startflag = true;
        //curtime = imu_msg.header.stamp.toSec();
        curtime = ros::Time::now().toSec();
        lasttime = curtime;
    }
    else
    {
        lasttime = curtime;
        //curtime = imu_msg.header.stamp.toSec();
        curtime = ros::Time::now().toSec();
        VisualCompass::AngleMsg dth;
        dth.header.stamp = ros::Time::now();
        dth.Angle = filterdAngle * 0.01;
        std::cout << "times:" << curtime - lasttime << std::endl;
        imuAngle.publish(dth);
    }
    
}

//void visualcompassCallback(const std_msgs::Float64& viscomangle)
//{
    
//}

//wheel message callback 
void wheelMsgCallback(const COM_Landing::cleanBot_msg& bot_msg)
{
	//ROS_INFO("get bot msg.");
    uint8_t PZ;
    uint16_t RC,LC,TC;
    int16_t LD,RD;
    //wheelOdometry& bo=botOdomPose;
    
    PZ = bot_msg.PZ;
    RC = bot_msg.RC;
    LC = bot_msg.LC;
    TC = bot_msg.TC;
    LD = bot_msg.LD;
    RD = bot_msg.RD;

    if(botOdomPose.mbStartflag)
    {
        //first frame, don't handle, only save the values
        botOdomPose.mbStartflag = false;
    }   
    else
    {   
        //compute odometry data
        double dt = (TC - botOdomPose.mpreTC)*botOdomPose.mParamBotLLPeriod;        
        //left wheel direction
        int ldir = static_cast<int>(botOdomPose.mpreLD>=0) - static_cast<int>(botOdomPose.mpreLD < 0);
        std::cout << "ldir:" << ldir << std::endl;

        //left wheel distance
        double ldif = LC-botOdomPose.mpreLC;
        if(ldif<-65000) 
            ldif += 65536;
        double lw = ldir*ldif*botOdomPose.mParamEncStepDis;///botOdomPose.mParamBotEncRoundCnt*botOdomPose.mParamBotWheelCf;

        //right
        int rdir = static_cast<int>(botOdomPose.mpreRD>=0) - static_cast<int>(botOdomPose.mpreRD < 0);
        std::cout << "rdir:" << rdir << std::endl;

        //right wheel distance
        double rdif = RC-botOdomPose.mpreRC;
        if(rdif<-65000) 
            rdif += 65536;
        double rw = rdir*rdif*botOdomPose.mParamEncStepDis;///botOdomPose.mParamBotEncRoundCnt*botOdomPose.mParamBotWheelCf;

        // delta distance
        double ds = (lw+rw)*0.5;
        // delta theta (angle, in radius. x/y/z - forward/left/upward, turn left ~ positive angle)
        double dth = (rw - lw)*botOdomPose.mParamInvBotWheelBsl ;

        lasttime = curtime;
        curtime = ros::Time::now().toSec();


        VisualCompass::AngleMsg wheeldth;
        if(fabs(dth) > 0.02)
            dth = 0;
        if(fabs(dth - filterdAngle * 0.01) > deltaThr) 
            dth = filterdAngle * 0.01;

        wheeldth.header.stamp = ros::Time::now();
        wheeldth.Angle = dth;
        wheelAngle.publish(wheeldth);

        // delta x/y
        double deltaX = ds*cos(botOdomPose.mTheta + dth*0.5);
        double deltaY = ds*sin(botOdomPose.mTheta + dth*0.5);

        float threshold = 0.01;

        // update x/y/theta
        botOdomPose.mPosX += deltaX;
        botOdomPose.mPosY += deltaY;
        botOdomPose.mTheta += dth;
        // update velocity
        botOdomPose.mVelX = ds/dt;
        botOdomPose.mVelY = 0.0;
        botOdomPose.mVelTheta = dth/dt;

        //printf("LC=%d preLC=%d, RC=%d preRC=%d\n",LC,botOdomPose.mpreLC,RC,botOdomPose.mpreRC);

        //printf("ds=%.5f, dth=%.5f\n",ds,dth);
        //printf("rdir=%d, rdif=%.5f, rw=%.5f\n",rdif,rdir,rw);
        //printf("ldir=%d, ldif=%.5f, lw=%.5f\n",ldif,ldir,lw);
    }

    // record pre-value
    botOdomPose.mprePZ = PZ;
    botOdomPose.mpreRC = RC;
    botOdomPose.mpreLC = LC;
    botOdomPose.mpreTC = TC;
    botOdomPose.mpreLD = LD;
    botOdomPose.mpreRD = RD;
}
