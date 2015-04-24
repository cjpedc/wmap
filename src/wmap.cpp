#include "wmap.h"

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/plane_refinement_comparator.h>

namespace mapper{

pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

wmap::wmap(){

    ROS_INFO_STREAM("Starting MAP");
    ros::NodeHandle n_;
    // Subscribe to point cloud
    //pcl2_sub_ = n_.subscribe ("/kinect2/depth_highres/points", 1, &wmap::pcl2Callback, this);

}

wmap::~wmap() { }
/*
void wmap::pcl2Callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {

    ROS_INFO_STREAM("Received pcl2Callback");

    pcl::fromROSMsg(*msg, *input_cloud);

   return;
}
*/


}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "");
    mapper::wmap wmap_node;
    ros::Rate loop_rate(10);

    tf::TransformListener tf(ros::Duration(10));
    costmap_2d::Costmap2DROS costmap("my_costmap", tf);
    costmap_2d::Costmap2DROS cost();

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
