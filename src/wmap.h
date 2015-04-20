#ifndef WMAP_HPP
#define WMAP_HPP

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <Eigen/Eigen>

// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <string.h>
#include <sstream>
#include <cmath>
#include <vector>

#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <visualization_msgs/Marker.h>

//Opencv
//#include "cv.h"
//#include "highgui.h"
//#include <opencv.hpp>

#define SIZE 550
#define RESOLUTION 0.1
#define MAX_IR_RANGE 4.5
#define MIN_IR_RANGE 0.5
#define WINDOW_SIZE 100

namespace mapper{

using namespace std;

class wmap{

public:
    bool save_data;
    wmap();
    string path_imgrec;
    string path_bow;

    geometry_msgs::PoseStamped f_pose;

    int counter_imgrec;
    int n_keymatches;
    double sec_stamp_pdc;
    void pcl2Callback(const sensor_msgs::PointCloud2::ConstPtr& input);
    void bow();
    void images_fast_map();
    ~wmap();


private:

    ros::Subscriber pcl2_sub_;
    ros::Publisher filtered_pub_;
    tf::TransformListener tf_listener_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;


};
}//namespace mapper
#endif // WMAP_HPP
