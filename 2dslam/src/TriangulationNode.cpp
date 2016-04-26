#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <TriangulationNode.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <curses.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>

float Brightness;
float Contrast ;
float Saturation;
float Gain;

int B;
int C;
int S;
int G;

using namespace cv;
using namespace std;

char winName[20]="Live";
Mat frame;
VideoCapture cap ( 1 );

void onTrackbar_changed ( int, void * )
{
        Brightness =float ( B ) /100;
        Contrast   =float ( C ) /100;
        Saturation =float ( S ) /100;
        Gain       =float ( G ) /100;

        cap.set ( CV_CAP_PROP_BRIGHTNESS,Brightness );
        cap.set ( CV_CAP_PROP_CONTRAST, Contrast );
        cap.set ( CV_CAP_PROP_SATURATION, Saturation );
        cap.set ( CV_CAP_PROP_GAIN, Gain );

}

using namespace std;
using namespace cv;

namespace LineLidar
{
Triangulation::Triangulation()
{
        ros::NodeHandle nh;
        this->baseline_len = 80;
        this->focal_len = 3.1;
        this->pixel_size = 6.35/5*4/1280;
        pointclouds_pub = nh.advertise<sensor_msgs::PointCloud2> ( "/pointcloud_topic", 30 );
        image_raw_sub = nh.subscribe ( "/usb_cam/image_raw", 30, &Triangulation::usbcam_image_callback, this );

}

Triangulation::~Triangulation()
{

}

void Triangulation::image_undistort ( Mat raw_image, Mat undistorted_image )
{
        cv::FileStorage fSettings ( "/home/saodiseng/catkin_ws/src/LineLidar/data/camera_data.yaml", cv::FileStorage::READ );
        float fx = fSettings["Camera.fx"];
        float fy = fSettings["Camera.fy"];
        float cx = fSettings["Camera.cx"];
        float cy = fSettings["Camera.cy"];

        cv::Mat K = cv::Mat::eye ( 3,3,CV_32F );
        K.at<float> ( 0,0 ) = fx;
        K.at<float> ( 1,1 ) = fy;
        K.at<float> ( 0,2 ) = cx;
        K.at<float> ( 1,2 ) = cy;
//   intrinsic_matrix = K.clone();
        K.copyTo ( intrinsic_matrix );

        cv::Mat DistCoef ( 5,1,CV_32F );
        DistCoef.at<float> ( 0 ) = fSettings["Camera.k1"];
        DistCoef.at<float> ( 1 ) = fSettings["Camera.k2"];
        DistCoef.at<float> ( 2 ) = fSettings["Camera.p1"];
        DistCoef.at<float> ( 3 ) = fSettings["Camera.p2"];
        DistCoef.at<float> ( 4 ) = fSettings["Camera.k3"];
        DistCoef.copyTo ( dist_coefficent );
//   dist_coefficent = DistCoef.clone();
        undistort ( raw_image, undistorted_image, intrinsic_matrix, dist_coefficent );
        cout << intrinsic_matrix << endl;
        cout << dist_coefficent << endl;
        imshow ( "distorted image", undistorted_image );
        waitKey ( 1 );
}

void Triangulation::process_image ( Mat raw_image )
{
        Mat undistorted_image = raw_image.clone();


        image_undistort ( raw_image, undistorted_image );
        raw_image = undistorted_image.clone();

        cv::Mat gray_image;
        cv::cvtColor ( raw_image, gray_image, CV_BGR2GRAY );

        int image_width, image_height;
        image_width = raw_image.size().width;
        image_height = raw_image.size().height;
        cout << image_width << image_height << endl;

        int max_column_gray_index_array[image_width];
        float column_gray_center[image_width];
        int valid_column[image_width];
        int valid_column_number = 0;

        std::cout << "here ok" << std::endl;
        for ( int i = 0 ; i < image_width; i++ ) {
                int max_column_gray_value = gray_image.at<uchar> ( 0,i );
                max_column_gray_index_array[i] = 0;
                for ( int j = 1; j < image_height; j++ ) {
                        if ( max_column_gray_value < gray_image.at<uchar> ( j,i ) ) {
                                max_column_gray_value = gray_image.at<uchar> ( j,i );
                                max_column_gray_index_array[i] = j;
                        }
                }
                if ( max_column_gray_value > 50 ) {
                        valid_column[valid_column_number] = i;
                        valid_column_number ++;
                        if ( max_column_gray_index_array[i] > 5 && max_column_gray_index_array[i] < 240 ) {
                                double gray_value_sum = 0;
                                double gray_weighted_index_sum = 0;
                                for ( int j = max_column_gray_index_array[i] - 5; j < max_column_gray_index_array[i] + 5; j++ ) {
                                        gray_value_sum = gray_value_sum + gray_image.at<uchar> ( j, i );
                                        gray_weighted_index_sum = gray_weighted_index_sum + gray_image.at<uchar> ( j, i ) * j;
                                }
                                if ( gray_value_sum != 0 )
                                        column_gray_center[i] = gray_weighted_index_sum / gray_value_sum;
                        }
                }
        }


//    float column_gray_center[valid_column_number];
        //calculate the gray center
        std::cout << "here ok" << std::endl;
        for ( int i = 0; i < valid_column_number; i++ ) {
//        if(max_column_gray_index_array[valid_column[i]] > 5 && max_column_gray_index_array[valid_column[i]] < 240 )
//        {
                //            double gray_value_sum = 0;
                //            double gray_weighted_index_sum = 0;
                //            for(int j = max_column_gray_index_array[valid_column[i]] - 5; j < max_column_gray_index_array[valid_column[i]] + 5; j++)
                //            {
                //                gray_value_sum = gray_value_sum + gray_image.at<uchar>(j, valid_column[i]);
                //                gray_weighted_index_sum = gray_weighted_index_sum + gray_image.at<uchar>(j, valid_column[i]) * j;
                //            }
                //            if(gray_value_sum != 0)
                //                column_gray_center[valid_column[i]] = gray_weighted_index_sum / gray_value_sum;
//        }
        }

        std::cout << "here ok" << std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr ( new pcl::PointCloud<pcl::PointXYZ> );
        basic_cloud_ptr->width = valid_column_number;
        basic_cloud_ptr->height = 1;


        std::cout << "valid_column_number"<< valid_column_number << std::endl;
        for ( int i = 0; i < valid_column_number; i++ ) {
                pcl::PointXYZ basic_point;
//       basic_point.x = ( float ) baseline_len * focal_len/ ( ( 1.0 * 240 - column_gray_center[valid_column[i]] ) * this->pixel_size * 1000 ) ;
                basic_point.x = ( float ) 15189.577421 / ( 240 - column_gray_center[valid_column[i]] ) / 100;
                if ( i == ( int ) valid_column_number / 2 ) {
                        std::cout << i << std::endl;
                        std::cout << basic_point.x << std::endl;
                        cout << column_gray_center[valid_column[i]] << endl;
                }
                basic_point.y = ( float ) ( valid_column[i] - 320 ) * this->pixel_size * basic_point.x / focal_len ;
                basic_point.z = 1.0 * 10/100;
                basic_cloud_ptr->points.push_back ( basic_point );
                //        basic_cloud_ptr->points[i].x = 0;
                //        basic_cloud_ptr->points[i].y = 0;
                //        basic_cloud_ptr->points[i].z = 0;
        }

        std::cout << "debug" << std::endl;

        sensor_msgs::PointCloud2 PCL_msgs;
        pcl::toROSMsg ( *basic_cloud_ptr, PCL_msgs );
        PCL_msgs.header.stamp  = ros::Time::now();
        PCL_msgs.header.frame_id = "/neato_laser";
        pointclouds_pub.publish ( PCL_msgs );
}

void Triangulation::usbcam_image_callback ( const sensor_msgs::ImageConstPtr usbcam_image_ptr )
{
        ROS_INFO("callback started");
        //convert sensor_msgs images to opencv image
        cv_bridge::CvImagePtr cv_ptr;
        try {
                cv_ptr = cv_bridge::toCvCopy ( usbcam_image_ptr, sensor_msgs::image_encodings::BGR8 );
        } catch ( cv_bridge::Exception &e ) {
                ROS_ERROR ( "cv_bridge exception: %s", e.what() );
                return;
        }
        cv::Mat raw_image = cv_ptr->image.clone();

        //extract the laser line
        cv::Mat gray_image;
        cv::cvtColor ( raw_image, gray_image, CV_BGR2GRAY );

        int image_width, image_height;
        image_width = raw_image.size().width;
        image_height = raw_image.size().height;

        int max_column_gray_index_array[image_width];
        float column_gray_center[image_width];
        int valid_column[image_width];
        int valid_column_number = 0;

        std::cout << "here ok" << std::endl;
        for ( int i = 0 ; i < image_width; i++ ) {
                int max_column_gray_value = gray_image.at<uchar> ( 0,i );
                max_column_gray_index_array[i] = 0;
                for ( int j = 1; j < image_height; j++ ) {
                        if ( max_column_gray_value < gray_image.at<uchar> ( j,i ) ) {
                                max_column_gray_value = gray_image.at<uchar> ( j,i );
                                max_column_gray_index_array[i] = j;
                        }
                }
                if ( max_column_gray_value > 50 ) {
                        valid_column[valid_column_number] = i;
                        valid_column_number ++;
                        if ( max_column_gray_index_array[i] > 5 && max_column_gray_index_array[i] < 240 ) {
                                double gray_value_sum = 0;
                                double gray_weighted_index_sum = 0;
                                for ( int j = max_column_gray_index_array[i] - 5; j < max_column_gray_index_array[i] + 5; j++ ) {
                                        gray_value_sum = gray_value_sum + gray_image.at<uchar> ( j, i );
                                        gray_weighted_index_sum = gray_weighted_index_sum + gray_image.at<uchar> ( j, i ) * j;
                                }
                                if ( gray_value_sum != 0 )
                                        column_gray_center[i] = gray_weighted_index_sum / gray_value_sum;
                        }
                }
        }


//    float column_gray_center[valid_column_number];
        //calculate the gray center
        std::cout << "here ok" << std::endl;
        for ( int i = 0; i < valid_column_number; i++ ) {
//        if(max_column_gray_index_array[valid_column[i]] > 5 && max_column_gray_index_array[valid_column[i]] < 240 )
//        {
                //            double gray_value_sum = 0;
                //            double gray_weighted_index_sum = 0;
                //            for(int j = max_column_gray_index_array[valid_column[i]] - 5; j < max_column_gray_index_array[valid_column[i]] + 5; j++)
                //            {
                //                gray_value_sum = gray_value_sum + gray_image.at<uchar>(j, valid_column[i]);
                //                gray_weighted_index_sum = gray_weighted_index_sum + gray_image.at<uchar>(j, valid_column[i]) * j;
                //            }
                //            if(gray_value_sum != 0)
                //                column_gray_center[valid_column[i]] = gray_weighted_index_sum / gray_value_sum;
//        }
        }

        std::cout << "here ok" << std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr ( new pcl::PointCloud<pcl::PointXYZ> );
        basic_cloud_ptr->width = valid_column_number;
        basic_cloud_ptr->height = 1;


        std::cout << "valid_column_number"<< valid_column_number << std::endl;
        for ( int i = 0; i < valid_column_number; i++ ) {
                pcl::PointXYZ basic_point;
                basic_point.x = baseline_len * focal_len/ ( ( 240 - column_gray_center[valid_column[i]] ) * this->pixel_size * 1000 ) ;
                //        std::cout << basic_point.x << std::endl;
                basic_point.y = ( valid_column[i] - 320 ) * this->pixel_size * basic_point.x / focal_len ;
                basic_point.z = 1.0 * 10/100;
                basic_cloud_ptr->points.push_back ( basic_point );
                //        basic_cloud_ptr->points[i].x = 0;
                //        basic_cloud_ptr->points[i].y = 0;
                //        basic_cloud_ptr->points[i].z = 0;
        }

        std::cout << "debug" << std::endl;

        sensor_msgs::PointCloud2 PCL_msgs;
        pcl::toROSMsg ( *basic_cloud_ptr, PCL_msgs );
        PCL_msgs.header.stamp  = ros::Time::now();
        PCL_msgs.header.frame_id = "/base_link";
        pointclouds_pub.publish ( PCL_msgs );

        //delete basic_cloud_ptr;
}


}

int main ( int argc,char **argv )
{
        ros::init ( argc, argv, "TriangulationNode" );

        if(!cap.isOpened()) // check if we succeeded
                return -1;

        cout << "Press 's' to save snapshot" << endl;
        namedWindow(winName);

        Brightness = cap.get(CV_CAP_PROP_BRIGHTNESS );
        Contrast   = cap.get(CV_CAP_PROP_CONTRAST );
        Saturation = cap.get(CV_CAP_PROP_SATURATION );
        Gain       = cap.get(CV_CAP_PROP_GAIN );

        cout << "====================================" << endl << endl;
        cout << "Default Brightness -------> " << Brightness << endl;
        cout << "Default Contrast----------> " << Contrast << endl;
        cout << "Default Saturation--------> " << Saturation << endl;
        cout << "Default Gain--------------> " << Gain << endl << endl;
        cout << "====================================" << endl;

        B = int( Brightness*100 );
        C = int( Contrast*100 );
        S = int( Saturation*100 );
        G = int( Gain*100 );

        createTrackbar( "Brightness",winName, &B, 100, onTrackbar_changed );
        createTrackbar( "Contrast",winName, &C, 100,onTrackbar_changed );
        createTrackbar( "Saturation",winName, &S, 100,onTrackbar_changed );
        createTrackbar( "Gain",winName, &G, 100,onTrackbar_changed );

        LineLidar::Triangulation triangulation_node;

        ros::Rate loop_rate( 30 );

        using namespace LineLidar;
        while (ros::ok()) {
                cap >> frame; // get a new frame from camera

                triangulation_node.process_image(frame);
                ros::spinOnce();
                static tf::TransformBroadcaster br;
                tf::Transform transform;
                transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
                tf::Quaternion q;
                q.setRPY(0, 0, 0);
                transform.setRotation(q);
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/neato_laser" ) );

                loop_rate.sleep();
        }
        return 0;
}
