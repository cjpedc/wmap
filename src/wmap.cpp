#include "wmap.h"

//#include <pcl/common/transforms.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/segmentation/plane_refinement_comparator.h>


namespace costmap_2d {
  class Costmap2DNode {
    public:
      Costmap2DNode(tf::TransformListener& tf) : costmap_ros_("costmap", tf){}
    private:
      Costmap2DROS costmap_ros_;
  };
};

int main(int argc, char** argv){
  ros::init(argc, argv, "costmap_node");

  tf::TransformListener tf(ros::Duration(10));

  costmap_2d::Costmap2DNode* costmap_node;
  costmap_node = new costmap_2d::Costmap2DNode(tf);

  ros::spin();

  delete costmap_node;

  return(0);
}

/*
namespace mapper{

pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

wmap::wmap(){

    ROS_INFO_STREAM("Starting MAP");

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

/*
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wmap_costmap");
    ros::NodeHandle n_;
    //mapper::wmap wmap_node;
    //ros::Rate loop_rate(10);

    tf::TransformListener tf(ros::Duration(10));
    costmap_2d::Costmap2DROS costmap("my_costmap", tf);

    ros::spin();

    /*
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    */
/*
}
*/
