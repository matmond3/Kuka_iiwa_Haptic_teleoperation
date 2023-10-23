#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2());
pcl::PLYWriter writer;

void callback(const sensor_msgs::PointCloud2 msg)
{
  //cout<<"here2"<<endl;
  pcl_conversions::toPCL(msg, *cloud);
  //cout<<"here3"<<endl;
	writer.writeBinary("/home/matteo/catkin_ws/Provafin.ply", *cloud);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "PointCloud_reader");
  ros::NodeHandle nh;
  //cout<<"here1"<<endl;
  ros::Subscriber sub = nh.subscribe("camera/cloud", 1, callback);
  ros::spin();
}

// In order to save the Point Cloud in files use the following command in terminal
// When the node is running

// rosrun pcl_ros bag_to_pcd <input_file.bag> <topic> <output_directory>
// rosbag record <topic name>