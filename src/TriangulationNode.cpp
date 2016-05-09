#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <TriangulationNode.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>

namespace LineLidar
{
Triangulation::Triangulation()
{
    ros::NodeHandle nh;
    this->baseline_len = 10;
    this->focal_len = 3.1;
    pointclouds_pub = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud_topic", 30);
    image_raw_sub = nh.subscribe("/usb_cam/image_raw", 30, &Triangulation::usbcam_image_callback, this);
}

Triangulation::~Triangulation()
{

}

void Triangulation::usbcam_image_callback(const sensor_msgs::ImageConstPtr usbcam_image_ptr)
{
    ROS_INFO("callback started");
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
    int valid_column_number = 0;

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
            valid_column[valid_column_number] = i;
            valid_column_number ++;
        }
    }

    float column_gray_center[valid_column_number];
    //calculate the gray center
    for(int i = 0; i < valid_column_number; i++)
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

        std::cout << "sub pixel index" << column_gray_center[valid_column[i]] << std::endl;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    basic_cloud_ptr->width = valid_column_number;
    basic_cloud_ptr->height = 1;


    std::cout << "valid_column_number"<< valid_column_number << std::endl;
    for(int i = 0; i < valid_column_number; i++)
    {
        std::cout << "debug" << std::endl;
        pcl::PointXYZ basic_point;
        basic_point.x = 1.0 * i/100;
        basic_point.y = 1.0 * i/100;
        basic_point.z = 1.0 * 10/100;
        basic_cloud_ptr->points.push_back (basic_point);
//        basic_cloud_ptr->points[i].x = 0;
//        basic_cloud_ptr->points[i].y = 0;
//        basic_cloud_ptr->points[i].z = 0;
    }

    std::cout << "debug" << std::endl;

    sensor_msgs::PointCloud2 PCL_msgs;
    pcl::toROSMsg(*basic_cloud_ptr, PCL_msgs);
    PCL_msgs.header.stamp  = ros::Time::now();
    PCL_msgs.header.frame_id = "/base_link";
    pointclouds_pub.publish(PCL_msgs);

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
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/base_link"));

        loop_rate.sleep();
    }
    return 0;
}
