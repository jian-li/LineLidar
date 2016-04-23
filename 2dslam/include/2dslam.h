#ifndef _SLAM_2d
#define _SLAM_2d

#include "pointmatcher/PointMatcher.h"
#include <sys/time.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>

#define PI 3.1415926

//using namespace PointMatcher;
typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef typename PointMatcher<float>::DataPoints::Label Label;
typedef typename PointMatcher<float>::DataPoints::Labels Labels;

namespace LineLidar
{
class slam2d
{
public:
  slam2d();
  ~slam2d();
  PointMatcher<float>::DataPoints * laserscantopmcloud ( sensor_msgs::LaserScan::ConstPtr laser_scan_ptr );
  void setcurrent_scan ( DP* current_scan );
  void setref_scan ( DP* ref_scan );
  void getnew_scan ( DP* new_scan );
  void do_icp ( sensor_msgs::LaserScan::ConstPtr laser_scan_ptr );
private:
  ros::Subscriber scan_sub;
  /*pointmatcher filter related parameters*/
  bool use_downsampling_filter;
  bool use_augmented_filter;
  PM::DataPointsFilters * downsampling_filters;
  PM::DataPointsFilters * augmented_filters;


  /*pointmatcher icp related parameters*/
  int scan_num;
  bool new_frame;
  bool force3d;
  bool first_scan;
  bool transform_calculated;
  double start_angle, end_angle;
  PM::TransformationParameters transform;
  DP* ref_scan;
  DP* current_scan;
  PointMatcher<float> pointmatcher;
  tf::Transform init_transform;
  tf::Transform transform_to_pub;
  PM::ICP icp_al;
};
}
#endif
