#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <TriangulationNode.h>

namespace LineLidar
{
	TriangulationNode::TriangulationNode()
	{
		pointclouds_pub = nh.advertise<>("/pointcloud_topic", 30);
		image_raw_sub = nh.subscribe("/usb_cam/image_raw", 30, usbcam_image_callback);
	}


void Triangulation::usbcam_image_callback(const cv_image::ConstPtr usbcam_image_ptr)
{
	
}

int main(int argc,char ** argv)
{
	ros::init(argc, argv, "TriangulationNode");

	TriangulationNode triangulation_node;
	
	ros::Rate loop_rate(10);

	while(ros::ok())
	{
		ros::spinOnce();
	}
	return 0;
}
}