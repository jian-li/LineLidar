#ifndef _PCL_ICP_H
#define _PCL_ICP_H

#include <ros/ros.h>
#include "2dslam.h"
#include <sensor_msgs/LaserScan.h>
#include "tf/transform_listener.h"
#include <laser_geometry/laser_geometry.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

namespace LineLidar
{
class pclicp
{
public:
        pclicp();
        ~pclicp();
        void new_scan_callback ( const sensor_msgs::LaserScan::ConstPtr &scan );
private:
        tf::TransformListener listener;
        int number_of_scan_got;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_last;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_current;
        laser_geometry::LaserProjection projector;

        ros::Subscriber scan_sub;

};

}
#endif
