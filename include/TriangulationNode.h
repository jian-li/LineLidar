
namespace LineLidar
{
class Triangulation
{
public:
	Triangulatoin();
	~Triangulation();
	usbcam_image_callback();
private:
	ros::NodeHandle nh;
	ros::Publisher pointclouds_pub;
	ros::Subscriber image_raw_sub;
}
}