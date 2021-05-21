
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PCLColorer
{
public:
  PCLColorer() : node_handle_(), node_handle_private_("~")
  {
    pcl_sub_ = node_handle_.subscribe("pcl_input", 1, &PCLColorer::cloud_in_cb, this);
    pcl_colored_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("pcl_colored", 1);
    r = node_handle_private_.param("r", 254);
    g = node_handle_private_.param("g", 0);
    b = node_handle_private_.param("b", 0);
    ROS_INFO_STREAM("Got RGB: " << r << ", " << g << ", " << b);
  }

private:
  ros::NodeHandle node_handle_;
  ros::NodeHandle node_handle_private_;
  ros::Subscriber pcl_sub_;
  ros::Publisher pcl_colored_pub_;
  int r, g, b;

  void cloud_in_cb(const sensor_msgs::PointCloud2 &msg)
  {
    // solution found in https://answers.ros.org/question/44053/adding-rgb-data-into-a-point-cloud/?answer=44248#post-id-44248
    // conversion taken from https://answers.ros.org/question/210606/need-pcl-conversion-help/?answer=210608#post-id-210608
    pcl::PCLPointCloud2 pcl2;
    pcl::PointCloud<pcl::PointXYZRGB> colored_pcl;
    pcl_conversions::toPCL(msg, pcl2);
    pcl::fromPCLPointCloud2(pcl2, colored_pcl);

    pcl::PointCloud<pcl::PointXYZRGB>::iterator it;
    for (it = colored_pcl.begin(); it < colored_pcl.end(); it++) {
      // taken from here: https://pointclouds.org/documentation/structpcl_1_1_r_g_b.html
      it->rgb = ((int) r) << 16 | ((int) g) << 8 | ((int) b);

    }

    sensor_msgs::PointCloud2 result_pcl;
    pcl::PCLPointCloud2 tmp;
    pcl::toPCLPointCloud2(colored_pcl, tmp);
    pcl_conversions::fromPCL(tmp, result_pcl);

    pcl_colored_pub_.publish(result_pcl);
  }
};

int main(int argc, char ** argv)
{
// Initialize ROS
  ros::init(argc, argv, "pcl_colorer");
  PCLColorer node;

  ros::spin();
}