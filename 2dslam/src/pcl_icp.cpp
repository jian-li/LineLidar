#include <ros/ros.h>
#include <pcl/io/pcd_io.h>

#include <pcl_icp.h>
#include <pcl_conversions/pcl_conversions.h>

namespace LineLidar
{
pclicp::pclicp()
{
        pcl::PointCloud<pcl::PointXYZ>::Ptr src ( new pcl::PointCloud<pcl::PointXYZ> );
        cloud_last = src;
        pcl::PointCloud<pcl::PointXYZ>::Ptr dst ( new pcl::PointCloud<pcl::PointXYZ> );
        cloud_current = dst;

        number_of_scan_got = 0;

        ros::NodeHandle nh;
        scan_sub = nh.subscribe ( "/base_scan", 2, &pclicp::new_scan_callback, this );
}

pclicp::~pclicp()
{

}

void pclicp::new_scan_callback ( const sensor_msgs::LaserScan::ConstPtr &scan )
{
        sensor_msgs::PointCloud2 cloud;
        projector.transformLaserScanToPointCloud ( "base_laser", *scan, cloud, listener );
        if ( number_of_scan_got == 0 ) {
                pcl::fromROSMsg ( cloud, *cloud_last );
                number_of_scan_got ++;
        } else {
                if ( number_of_scan_got == 1 ) {
                        pcl::fromROSMsg ( cloud, *cloud_current );
                        number_of_scan_got ++;
                } else {
                        cloud_last = cloud_current;
                        pcl::fromROSMsg ( cloud, *cloud_current );
                        // do icp algorithm
                        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
                        icp.setInputSource( cloud_last );
                        icp.setInputTarget( cloud_current );
                        pcl::PointCloud<pcl::PointXYZ> Final;
                        icp.align ( Final );
                        
                        if ( icp.hasConverged() ) {
                                std::cout << icp.getFitnessScore() << std::endl;
                                std::cout << icp.getFinalTransformation() << std::endl;
                        }
                }
        }
}
}

/*
 * TODO: use imu estimated pose to project the 3d data
 *       use several pointcloud to do icp
 * reference the hectorslam 
 */

int main(int argc, char **argv )
{
        ros::init ( argc, argv, "pclicp" );

        LineLidar::pclicp pcl_al;

        ros::Rate spin_rate ( 7 );

        while ( ros::ok() ) {
                ros::spinOnce();
                spin_rate.sleep();
        }
        return 0;
}

