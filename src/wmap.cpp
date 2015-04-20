#include "wmap.h"

namespace mapper{

wmap::wmap(){

    ROS_INFO_STREAM("Starting Point Cloud");
    ros::NodeHandle n_;
    // Subscribe to point cloud
    pcl2_sub_ = n_.subscribe ("/kinect2/depth_highres/points", 1, &wmap::pcl2Callback, this);

    cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Publish a point cloud of filtered data that is not part of the floor
    //       filtered_pub_ = n_.advertise< pcl::PointCloud<pcl::PointXYZRGB> > ("fltr_pcl", 1);

}

wmap::~wmap() { }

void wmap::pcl2Callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {

    ROS_INFO_STREAM("Received pcl2Callback");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    //        pcl::fromPCLPointCloud2 (*msg, *tempCloud);
    //cloud = tempCloud;

    // Make new point cloud that is in the working frame
    //        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
    /*
        pcl::PCLPointCloud2 pcl_pc;
        pcl_conversions::toPCL( *pcl2_in , pcl_pc);

        pcl::PointCloud<pcl::PointXYZ> cloud;

        pcl::fromPCLPointCloud2(pcl_pc, cloud);
*/
}



}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "");
    mapper::wmap wmap_node;
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
