#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>
#include <image_transport/image_transport.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/io.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/core/mat.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;
using namespace std;
using namespace cv;

// cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << 830.725, 0, 336.639, 0, 692.817, 203.056, 0, 0, 1);
// cv::Mat distCoeffs= (cv::Mat_<double>(1,5) << 0.00611456, 0, 0, 0, 0);
cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << 2263.89807, 0.0, 596.234646, 0.0, 2616.13015, 459.196393, 0.0, 0.0, 1);
cv::Mat distCoeffs= (cv::Mat_<double>(1,5) << -0.23800603, 5.16235061, -0.01268776, 0.02026168, 0.0);
//cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::generateCustomDictionary(2, 64);
cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
static int image_count = 0; // added this
ros::Publisher pub;

void callback(const sensor_msgs::ImageConstPtr& msg /*const char* filename*/)
{
  geometry_msgs::Pose pos;
  cv_bridge::CvImageConstPtr cv_ptr=NULL;
  try
    {
    // Coversion ROS image in OpenCV
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    
  }
  
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s'.", msg->encoding.c_str());
  }

  cv::Mat dImg = cv_ptr->image;

  /*// SAVE IMAGE AS .png file
  // They are saved in the folder where you launch the ros node using rosrun.

  if(image_count<3){
    std::stringstream sstream;                               // added this
    sstream << "my_image" << image_count << ".png" ;                  // added this
    ROS_ASSERT( cv::imwrite( sstream.str(),  cv_ptr->image ) );      // added this
    image_count++;
  } */
  
  cv::Mat imageCopy = dImg.clone();
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  //std::vector<cv::Point2f> rejectedCandidates;
  //cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
  //cv::aruco::detectMarkers(dImg, dictionary, corners, ids, parameters, rejectedCandidates);
  cv::aruco::detectMarkers(dImg, dictionary, corners, ids);

  // if at least one marker detected
  if (ids.size() > 0){
    cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
  } 

  std::vector<cv::Vec3d> rvecs, tvecs;
  cv::aruco::estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);

  //Draw the reference frame of each QR code
  for (int i = 0; i < rvecs.size(); ++i) {

    auto rvec = rvecs[i];
    auto tvec = tvecs[i];
    cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvec, tvec, 0.05);

  }

  cout<<tvecs[0]<<endl;
  cout<<rvecs[0]<<endl;
  cv::Vec3d r;
  //r=tvecs[0]+(tvecs[1]-tvecs[0])/2;
  r = tvecs[0];

  cv::Vec3d dist=(tvecs[1]-tvecs[0]);
  
  pos.position.x = tvecs[0][0];
  pos.position.y = tvecs[0][1];
  pos.position.z = tvecs[0][2];
  pos.orientation.x = rvecs[0][0];
  pos.orientation.y = rvecs[0][1];
  pos.orientation.z = rvecs[0][2];
  pos.orientation.w = 1;
  pub.publish(pos);

  //std::cout<<tvecs[0]<<" "<<tvecs[1]<<std::endl;
  //cout<<norm(dist, NORM_L2, noArray())<<endl;

  cv::imshow("OPENCV_WINDOW", imageCopy);
  int key = cv::waitKey( 50 );

}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "RGB_reader");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/rgb", 1, callback);
  pub= nh.advertise<geometry_msgs::Pose>("computation/marker/position", 100);

  ros::spin();
}