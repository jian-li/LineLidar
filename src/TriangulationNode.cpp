#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <TriangulationNode.h>
#include <sensor_msgs/LaserScan.h>
#include <cv_bridge/cv_bridge.h>

namespace LineLidar
{
	Triangulation::Triangulation()
	{
		ros::NodeHandle nh;
		pointclouds_pub = nh.advertise<sensor_msgs::LaserScan>("/pointcloud_topic", 30);
		image_raw_sub = nh.subscribe("/usb_cam/image_raw", 30, &Triangulation::usbcam_image_callback, this);
	}

	Triangulation::~Triangulation()
	{

	}

	void Triangulation::usbcam_image_callback(const sensor_msgs::ImageConstPtr usbcam_image_ptr)
	{
		//convert sensor_msgs images to opencv image
		cv_bridge::CvImagePtr cv_ptr;
	    try
	    {
	      cv_ptr = cv_bridge::toCvCopy(usbcam_image_ptr, sensor_msgs::image_encodings::BGR8);
	    }
	    catch (cv_bridge::Exception& e)
	    {
	      ROS_ERROR("cv_bridge exception: %s", e.what());
	      return;
	    }
		cv::Mat raw_image = cv_ptr->image.clone();

		//extract the laser line
		cv::Mat gray_image;
		cv::cvtColor(raw_image, gray_image, CV_BGR2GRAY);

		int image_width, image_height;
		image_width = raw_image.size().width;
		image_height = raw_image.size().height;

		int max_column_gray_index_array[image_width];
		int valid_column[image_width];
		int value_column_number = 0;

		for(int i = 0 ; i < image_width; i++)
		{
			int max_column_gray_value = gray_image.at<uchar>(0,i);
			max_column_gray_index_array[i] = 0;
			for(int j = 1; j < image_height; j++)
			{
				if(max_column_gray_value < gray_image.at<uchar>(j,i) )
				{
					max_column_gray_value = gray_image.at<uchar>(j,i);
					max_column_gray_index_array[i] = j;
				}
			}
			if(max_column_gray_value > 128 )
			{
				valid_column[value_column_number] = i;
				value_column_number ++;
			}
		}

		float column_gray_center[value_column_number];
		//calculate the gray center
		for(int i = 0; i < value_column_number; i++)
		{
			if(max_column_gray_index_array[valid_column[i]] > 5 && max_column_gray_index_array[valid_column[i]] < 240 )
			{
				double gray_value_sum = 0;
				double gray_weighted_index_sum = 0;
				for(int j = max_column_gray_index_array[valid_column[i]] - 5; j < max_column_gray_index_array[valid_column[i]] + 5; j++)
				{
					gray_value_sum = gray_value_sum + gray_image.at<uchar>(j, valid_column[i]);
					gray_weighted_index_sum = gray_weighted_index_sum + gray_image.at<uchar>(j, valid_column[i]) * j;
				}
				column_gray_center[valid_column[i]] = gray_weighted_index_sum / gray_value_sum;
			}
		}

		//convert to sensor_msgs::pointcloud



	}


}

int main(int argc,char ** argv)
{
	ros::init(argc, argv, "TriangulationNode");

	LineLidar::Triangulation triangulation_node;
	
	ros::Rate loop_rate(10);

	while(ros::ok())
	{
		ros::spinOnce();
	}
	return 0;
}