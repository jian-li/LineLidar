#include <2dslam.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <fstream>
#include <iostream>

/*include the library headers*/
#include <yaml-cpp/yaml.h>

using namespace std;

namespace LineLidar
{
slam2d::slam2d()
{
        /*config the parameter use yaml file*/
        YAML::Node config = YAML::LoadFile ( "/home/saodiseng/workspace/LineLidar/data/2dslamconfig.yaml" );

        this->use_downsampling_filter = config["usedownsampling"];
        this->use_augmented_filter = config["usedesciptoraugmenting"];

        if ( this->use_downsampling_filter ) {
                //        downsampling_filters = new PM::DataPointsFilter(
                //                    PM::get().DataPointsFilterRegistrar.create(
                //                        "SurfaceNormalDataPointsFilter",
                //                        map_list_of
                //                        ("binSize", "10")
                //                        ("epsilon", "5")
                //                        ("keepNormals", "0")
                //                        ("keepDensities", "1")
                //                        )
                //                    );
        }

        if ( true ) {
                std::string inputfilter_path = "/home/saodiseng/workspace/LineLidar/inputfilter.yaml";
                ifstream inputfilterstr ( inputfilter_path.c_str() );
                if ( inputfilterstr.good() ) {
                        //            PM::DataPointsFilters f(ifs);
                        //                f.apply(d);
                        std::cout <<"filter initialized" << std::endl;
                        this->augmented_filters = new PM::DataPointsFilters ( inputfilterstr );
                } else {
                        ROS_ERROR_STREAM ( "Cannot load input filters config from YAML file " << inputfilter_path );
                }
        }

        ros::NodeHandle nh;
        start_angle = -PI;
        end_angle = PI;
        this->scan_num = 0;
        this->scan_sub = nh.subscribe ( "/base_scan", 2, &slam2d::do_icp, this );
        this->force3d = false;
        this->new_frame = false;
        this->first_scan = true;
        this->transform_calculated = false;
        this->init_transform.setOrigin ( tf::Vector3 ( 0,0,0 ) );
        tf::Quaternion q;
        q.setRPY ( 0,0,0 );
        this->init_transform.setRotation ( q );
        this->transform_to_pub = this->init_transform;
        //    this->icp_al = new PM::ICP();

        //set up icp al parameter

        icp_al.setDefault();
        std::string yaml_path = "/home/saodiseng/workspace/LineLidar/config.yaml";
        ifstream ifs ( yaml_path.c_str() );
        if ( !ifs.good() ) {
                cerr << "Cannot open config file ";
                exit ( 1 );
        } else
                icp_al.loadFromYaml ( ifs );


        this->ref_scan = new DP();
        this->current_scan = new DP();
}

slam2d::~slam2d()
{

}

void slam2d::setref_scan ( DP *mref_scan )
{
        this->ref_scan = mref_scan;
}

void slam2d::setcurrent_scan ( DP *mcurrent_scan )
{
        this->current_scan = mcurrent_scan;
}

PointMatcher<float>::DataPoints *slam2d::laserscantopmcloud ( sensor_msgs::LaserScan::ConstPtr laser_scan_ptr )
{
        //convert laser scan msg to pointmatcher datapoints
        const double angle_increment = 2 * PI/360;

        Labels featLabels;
        int goodCount = 0;
        //
        featLabels.push_back ( Label ( "x",1 ) );
        featLabels.push_back ( Label ( "y",1 ) );
        if ( this->force3d ) {
                featLabels.push_back ( Label ( "z",1 ) );
        }
        featLabels.push_back ( Label ( "pad", 1 ) );

        //desc
        Labels descLabels;
        if ( !laser_scan_ptr->intensities.empty() ) {
                descLabels.push_back ( Label ( "intensity", 1 ) );
                assert ( laser_scan_ptr->intensities.size() == laser_scan_ptr->ranges.size() );
        }


        int good_id_array[360];
        for ( int i = 0; i < 360; i++ ) {
                if ( laser_scan_ptr->ranges[i] > laser_scan_ptr->range_min && laser_scan_ptr->ranges[i] < laser_scan_ptr->range_max ) {
                        good_id_array[goodCount] = i;
                        goodCount ++;
//            std::cout << "range" << 1.0 * laser_scan_ptr ->ranges[i] <<  std::endl;
                }
        }

        std::cout <<"good point number: " <<goodCount << std::endl;


        if ( goodCount == 0 ) {
                //return null points
                return new PointMatcher<float>::DataPoints();
        }

        PointMatcher<float>::DataPoints *cloud = new PointMatcher<float>::DataPoints ( featLabels, descLabels, goodCount );
        for ( int i = 0; i < goodCount; i++ ) {
                double current_angle = start_angle + angle_increment * good_id_array[i];
                double x = laser_scan_ptr->ranges[good_id_array[i]] * cos ( current_angle );
                double y = laser_scan_ptr->ranges[good_id_array[i]] * sin ( current_angle );
                double z = 0;
                cloud->features ( 0,i ) = x;
                cloud->features ( 1,i ) = y;
                if ( this->force3d ) {
                        cloud->features ( 2,i ) = z;
                }
        }
        return cloud;
        std::cout << "good number size" << cloud->getNbPoints() << std::endl;

}

void slam2d::getnew_scan ( DP *mnew_scan )
{
        //filter the scan
        std::cout << "here" << std::endl;
        this->augmented_filters->apply ( *mnew_scan );

        if ( this->first_scan ) {
                this->setcurrent_scan ( mnew_scan );
        } else {
                delete this->ref_scan;
                this->setref_scan ( this->current_scan );
                this->setcurrent_scan ( mnew_scan );
                this->new_frame = true;
        }
}

void slam2d::do_icp ( sensor_msgs::LaserScan::ConstPtr laser_scan_ptr )
{
        this->scan_num ++;
        DP *new_data_points = new DP();
        new_data_points = this->laserscantopmcloud ( laser_scan_ptr );

        this->getnew_scan ( new_data_points );

        std::cout << "start icp algorithm" << std::endl;
        std::cout << "scan number received" << this->scan_num << std::endl;

        timeval starttime,endtime;
        gettimeofday ( &starttime,0 );

        std::cout << this->current_scan << this->ref_scan << std::endl;
        if ( !this->first_scan ) {
                std::cout << this->current_scan->getNbPoints() << std::endl;
                std::cout << this->ref_scan->getNbPoints() << std::endl;
                std::cout << "not first scan" << std::endl;
                PM::TransformationParameters initial_transform ( 3,3 );
                initial_transform <<  1,0,0,
                                  0,1,0,
                                  0,0,1 ;
                transform =icp_al.compute ( *this->current_scan, *this->ref_scan, initial_transform );
                double diff_x = transform ( 0,2 ) - initial_transform ( 0,2 );
                double diff_y = transform ( 1,2 ) - initial_transform ( 1,2 );
//        if(diff_x * diff_x + diff_y * diff_y > 0.25)
//            transform = initial_transform;
//        transform = icp_al(*this->current_scan, *this->ref_scan);
//        DP data_out(*this->current_scan);
//        icp_al.transformations.apply(this->current_scan, transform);
                std::cout << "Final transformation:" << std::endl << transform << std::endl;

                //send the tf transform
                static tf::TransformBroadcaster br;
                std::cout << "translation is :" << transform ( 0,2 ) << " " << transform ( 1,2 ) << std::endl;
                tf::Vector3 origin ( 1.0 * transform ( 0,2 ), 1.0 * transform ( 1,2 ), 0 );
                tf::Matrix3x3 rotation ( transform ( 0,0 ), transform ( 0,1 ), 0,
                                         transform ( 1,0 ), transform ( 1,1 ), 0,
                                         0, 0, 1
                                       );
                //        rotation(0,0) = transform(0,0);
                //        rotation(0,1) = transform(0,1);
                //        rotation(1,0) = transform(1,0);
                //        rotation(1,1) = transform(1,1);
                //        rotation(2,2) = 1;
                tf::Transform temp ( rotation, origin );
                std::cout << "send transform" << std::endl;
                transform_to_pub.mult ( temp, this->transform_to_pub );
                //        std::cout << "Final transformation:" << std::endl << this->init_transform << std::endl;
                br.sendTransform ( tf::StampedTransform ( transform_to_pub, ros::Time::now(), "/world", "/neato_laser" ) );
        } else {
                std::cout << " first scan" << std::endl;
                this->first_scan = false;
        }

        gettimeofday ( &endtime,0 );
        double timeuse = 1000000* ( endtime.tv_sec - starttime.tv_sec ) + endtime.tv_usec - starttime.tv_usec;
        std::cout << timeuse/1000 << std::endl;
}
}


int main ( int argc, char **argv )
{
        ros::init ( argc, argv, "slam2d" );

        std::cout << "start " << std::endl;
        LineLidar::slam2d slam_2d;

        ros::Rate spin_rate ( 7 );

        while ( ros::ok() ) {
                ros::spinOnce();
                spin_rate.sleep();
        }
        return 0;
}
