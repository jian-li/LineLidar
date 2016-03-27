#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>

namespace LineLidar
{
class Triangulation
{
public:
	Triangulation();
	~Triangulation();
	void usbcam_image_callback(const sensor_msgs::ImageConstPtr usbcam_image_ptr);
private:
	float baseline_len;
	float focal_len;
    float pixel_size;
	ros::Publisher pointclouds_pub;
	ros::Subscriber image_raw_sub;
};
}
