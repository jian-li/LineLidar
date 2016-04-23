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
#include <sensor_msgs/Imu.h>

using namespace message_filters;
using namespace std;
using namespace CleanBot;

void BotMsgCallback(const COM_Landing::cleanBot_msg::ConstPtr& bot_msg);

// global data
wheelOdometry botOdomPose;

ros::Publisher odom_th_pub;

bool startflag = false;

double curtime = 0, lasttime = 0;

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "wheelOdometry");
    ros::NodeHandle n;
    //synchronize the message from the wheel and imu, use approximate time policies

    //subscribe robot message from low-level controller serial data
    ros::Subscriber botmsg_sub = n.subscribe("/CleanBot/BotMsg",1, BotMsgCallback);
    // publish odometry message to rviz
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/CleanBot/odom", 50);
    odom_th_pub = n.advertise<sensor_msgs::Imu>("/Odom/angVel",50);

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
    ros::Rate loop_rate(30);

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

void BotMsgCallback(const COM_Landing::cleanBot_msg::ConstPtr& bot_msg)
{
	//ROS_INFO("get bot msg.");
    uint8_t PZ;
    uint16_t RC,LC,TC;
    int16_t LD,RD;
    //wheelOdometry& bo=botOdomPose;
    
    PZ = bot_msg->PZ;
    RC = bot_msg->RC;
    LC = bot_msg->LC;
    TC = bot_msg->TC;
    LD = bot_msg->LD;
    RD = bot_msg->RD;

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
        //std::cout << "ldir:" << ldir << std::endl;

        //left wheel distance
        double ldif = LC-botOdomPose.mpreLC;
        if(ldif<-65000) 
            ldif += 65536;
        double lw = ldir*ldif*botOdomPose.mParamEncStepDis;///botOdomPose.mParamBotEncRoundCnt*botOdomPose.mParamBotWheelCf;

        //right
        int rdir = static_cast<int>(botOdomPose.mpreRD>=0) - static_cast<int>(botOdomPose.mpreRD < 0);
        //std::cout << "rdir:" << rdir << std::endl;

        //right wheel distance
        double rdif = RC-botOdomPose.mpreRC;
        if(rdif<-65000) 
            rdif += 65536;
        double rw = rdir*rdif*botOdomPose.mParamEncStepDis;///botOdomPose.mParamBotEncRoundCnt*botOdomPose.mParamBotWheelCf;

        // delta distance
        double ds = (lw+rw)*0.5;
        // delta theta (angle, in radius. x/y/z - forward/left/upward, turn left ~ positive angle)
        double dth = (rw - lw)*botOdomPose.mParamInvBotWheelBsl ;

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
