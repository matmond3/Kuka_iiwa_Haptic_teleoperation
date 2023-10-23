#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/io.h>
#include <pcl/io/ply_io.h>

#include "3DCamera.hpp"
#include <opencv2/imgproc.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>

#include <chrono>
using namespace std::chrono;

cv::Mat d_map(cv::Size(0,0), CV_16U);
cv::Mat rgb_map(cv::Size(0,0), CV_8UC3);
std::mutex rgb_mutex;
pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());

typedef pcl::PointXYZRGB  PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

PointCloudT::Ptr pc = NULL;

cs::ICameraPtr camera;

image_transport::Publisher pub_rgb;
image_transport::Publisher pub_depth;
ros::Publisher pub_cloud;


void depth_callback(cs::IFramePtr frame, void* usrData){
	//std::cout<<"New depth frame!"<<std::endl;
	d_map = cv::Mat(frame->getHeight(), frame->getWidth(), CV_16U, (void*)frame->getData(FRAME_DATA_FORMAT_Z16));
	
	// Send depth image to ROS
	sensor_msgs::ImagePtr dep = cv_bridge::CvImage(std_msgs::Header(), "u16", d_map).toImageMsg();	
	pub_depth.publish(dep);

	{
		const std::lock_guard<std::mutex> lock(rgb_mutex);
		if(rgb_map.size() != cv::Size(0,0)){
			//std::cout<<"Create PCL cloud!"<<std::endl;

			Intrinsics intr;
			camera->getIntrinsics(STREAM_TYPE_DEPTH, intr);
			Intrinsics intrColor;			
			camera->getIntrinsics(STREAM_TYPE_RGB, intrColor);
			Extrinsics extrColor;
			camera->getExtrinsics(extrColor);
			Distort distortColor;
			camera->getDistort(STREAM_TYPE_RGB, distortColor);

			float scale = 0.1f;
			PropertyExtension value;
			if (SUCCESS == camera->getPropertyExtension(PROPERTY_EXT_DEPTH_SCALE, value))
			{
				scale = value.depthScale;
			}

			cs::Pointcloud cam_pc;	
			cam_pc.generatePoints((unsigned short*)d_map.data, d_map.cols, d_map.rows, scale, &intr, &intrColor, &extrColor);

			const auto vertices = cam_pc.getVertices();
			const auto texcoords = cam_pc.getTexcoords();
			//const auto normals = cam_pc.getNormals();

			pc.reset(new PointCloudT());
			pc->resize(cam_pc.size());
			//n_pc.reset(new PointCloudNT());
			//n_pc->resize(cam_pc.size());

			for (int i = 0; i < cam_pc.size(); i++)
			{
				PointT &p=pc->points[i];
				//pcl::Normal &np=n_pc->points[i];

				p.x = vertices[i].x;
				p.y = vertices[i].y;
				p.z = vertices[i].z;
				//np.normal_x = normals[i].x;
				//np.normal_y = normals[i].y;
				//np.normal_z = normals[i].z;
				
				int x, y;
				x = int(texcoords[i].u * rgb_map.cols);
				if (x >= rgb_map.cols) x = rgb_map.cols - 1;
				if (x < 0)	x = 0;
				y = int(texcoords[i].v * rgb_map.rows);
				if (y >= rgb_map.rows) y = rgb_map.rows - 1;
				if (y < 0)	y = 0;
				unsigned char* color = rgb_map.data + (y * rgb_map.cols + x) * 3;
				p.r = color[0];
				p.g = color[1];
				p.b = color[2];		
			}

			// PointCloud::Ptr cloud_msg (new PointCloudT);
			pc->header.frame_id = "dept_camera";
  			// pc->height = pc->width = 1;
  			// pc->points.push_back (pcl::PointXYZ(1.0, 2.0, 3.0));

    		// pcl_conversions::toPCL(ros::Time::now(), pc->header.stamp);

			// //FILTER
			// pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
			// sor.setInputCloud (*pc);
			// sor.setLeafSize (0.1, 0.1, 0.1);
			// sor.filter (cloud_filtered);

			// cout<< intrColor.fx<<" "<<intrColor.fy<<" "<<intrColor.cx<<" "<<intrColor.cy<<" "<<endl;
			// cout<< distortColor.k1<<" "<< distortColor.k2<<" "<< distortColor.k3<<" "<< distortColor.k4<<" "<< distortColor.k5<<" "<<endl;
			
			toPCLPointCloud2(*pc, *cloud);
			sensor_msgs::PointCloud2 cloud_msg;
			pcl_conversions::fromPCL(*cloud, cloud_msg);
    		pub_cloud.publish (cloud_msg);

		}
	}
}

void rgb_callback(cs::IFramePtr frame, void* usrData){
	//std::cout<<"New rgb frame!"<<std::endl;
	const std::lock_guard<std::mutex> lock(rgb_mutex);
	rgb_map = cv::Mat(frame->getHeight(), frame->getWidth(), CV_8UC3, (void*)frame->getData()); // RGB format
	//cv::cvtColor(rgb_frame->frames[0], rgb_frame->frames[0], cv::COLOR_RGB2BGR); // Convert to BGR (default format for OpenCV computations)	
	
	// Send RGB image to ROS
	sensor_msgs::ImagePtr msg_rgb = cv_bridge::CvImage(std_msgs::Header(), "rgb8", rgb_map).toImageMsg();	
	pub_rgb.publish(msg_rgb);


}

int main(int argc, char **argv)
{
    camera = cs::getCameraPtr();

    std::cout<<"Attempting to start Acusense camera"<<std::endl;

	// Attempt connection
	int ret = camera->connect();
	if (ret != SUCCESS)
	{
		std::cout<<"Acusense camera connect failed with error code " <<std::to_string(ret)<<std::endl;
		return false;
	}

	// Get  camera info
	CameraInfo info;
	ret = camera->getInfo(info);
	if (ret != SUCCESS)
	{
		std::cout<<"Could not retrieve Acusense camera info. Error code: " << std::to_string(ret)<<std::endl;
		return false;
	}

	// Display camera info	
	std::cout<<"Name: " << std::string(info.name)<<std::endl;
	std::cout<<"Serial: " << std::string(info.serial)<<std::endl;
	std::cout<<"Unique ID: " << std::string(info.uniqueId)<<std::endl;
	std::cout<<"Firmware version: " << std::string(info.firmwareVersion)<<std::endl;
	std::cout<<"Algorithm version: " << std::string(info.algorithmVersion)<<std::endl;	

	std::cout<<"Successfully started Acusense camera"<<std::endl;    
	
	// Start rgb stream
	StreamInfo rgb_info;
	rgb_info.format = STREAM_FORMAT::STREAM_FORMAT_RGB8;
	rgb_info.width = 1600;
	rgb_info.height = 1200;
	rgb_info.fps = 10.0;
	ret = camera->startStream(STREAM_TYPE_RGB, rgb_info, rgb_callback, NULL);
	if (ret != SUCCESS)
	{
		std::cout<<"Failed to start camera rgb stream. Error code: " << std::to_string(ret)<<std::endl;
		return false;
	}

    // Set initial depth parameters	
	PropertyExtension ext;	
	ext.depthRange.min = 50;
	ext.depthRange.max = 2000;
	//ext.autoExposureMode = AUTO_EXPOSURE_MODE_CLOSE;	
	ext.autoExposureMode = AUTO_EXPOSURE_MODE_HIGH_QUALITY;

	ret = camera->setPropertyExtension(PROPERTY_EXT_DEPTH_RANGE, ext);
	if (ret != SUCCESS) {
		std::cout<<"Could not set Acusense depth range. Error code: " << std::to_string(ret)<<std::endl;
		return false;
	}	
	
	float frame_time = 7000.0f;	
    ret = camera->setProperty(STREAM_TYPE_DEPTH, PROPERTY_FRAMETIME, frame_time);
	if (ret != SUCCESS) {
		std::cout<<"Could not modify Acusense depth frame time. Error code: " << std::to_string(ret)<<std::endl;
		return false;
	}

	ret = camera->setProperty(STREAM_TYPE_DEPTH, PROPERTY_EXPOSURE, frame_time - 1000.0f);
	if (ret != SUCCESS) {
		std::cout<<"Could not modify Acusense depth exposure. Error code: " << std::to_string(ret)<<std::endl;
		return false;
	}

	ret = camera->setProperty(STREAM_TYPE_DEPTH, PROPERTY_GAIN, 16.0f);
	if (ret != SUCCESS) {
		std::cout<<"Could not modify Acusense depth gain. Error code: " << std::to_string(ret)<<std::endl;
		return false;
	}

	ret = camera->setPropertyExtension(PROPERTY_EXT_AUTO_EXPOSURE_MODE, ext);
	if (ret != SUCCESS) {
		std::cout<<"Could not set Acusense depth auto-exposure mode. Error code: " << std::to_string(ret)<<std::endl;
		return false;
	}

	// Set initial rgb parameters	
	/*
	ret = camera->setProperty(STREAM_TYPE_RGB, PROPERTY_ENABLE_AUTO_FOCUS, 1);
	if (ret != SUCCESS)
	{
		log("Could not set Acusense RGB auto-focus. Error code: " + std::to_string(ret));
		return false;
	}
	*/

	ret = camera->setProperty(STREAM_TYPE_RGB, PROPERTY_ENABLE_AUTO_EXPOSURE, 1);
	if (ret != SUCCESS)
	{
		std::cout<<"Could not set Acusense RGB auto-exposure. Error code: " << std::to_string(ret)<<std::endl;
		return false;
	}

    ret = camera->setProperty(STREAM_TYPE_RGB, PROPERTY_ENABLE_AUTO_WHITEBALANCE, 1);
	if (ret != SUCCESS)
	{
		std::cout<<"Could not set Acusense RGB auto-whitebalance. Error code: " << std::to_string(ret)<<std::endl;
		return false;
	}

	// get  informations of depth-stream 
	std::vector<StreamInfo> streamInfos;
	ret = camera->getStreamInfos(STREAM_TYPE_DEPTH,  streamInfos);
	if(ret != SUCCESS)
	{
		printf("camera get stream info failed(%d)!\n", ret);
		return -1;
	}

	// display  informations of depth-stream 
	for (auto streamInfo : streamInfos)
	{
		// printf("depth format:%2d, width:%4d, height:%4d, fps:%2.1f\n", streamInfo.format, streamInfo.width, streamInfo.height, streamInfo.fps);
	}
	//printf("\n");

	// Start depth stream
	StreamInfo depth_info;
	/*
	// Balanced for speed
	depth_info.format = STREAM_FORMAT::STREAM_FORMAT_Z16;
	depth_info.width = 640;
	depth_info.height = 400;
	depth_info.fps = 10.0;	
	*/
	// Slow, high quality
	depth_info.format = STREAM_FORMAT::STREAM_FORMAT_Z16;
	depth_info.width = 1280;
	depth_info.height = 800;
	depth_info.fps = 2.0;	

	ret = camera->startStream(STREAM_TYPE_DEPTH, depth_info, depth_callback, NULL);
	if (ret != SUCCESS)
	{
		std::cout<<"Failed to start camera depth stream. Error code: " << std::to_string(ret)<<std::endl;
		return false;
	}	
    
    ros::init(argc,argv,"Image_reader");

    ros::NodeHandle n;

	image_transport::ImageTransport it(n);
	pub_rgb = it.advertise("camera/rgb", 1);
	pub_depth = it.advertise("camera/depth", 1);
	pub_cloud = n.advertise<sensor_msgs::PointCloud2>("camera/cloud", 1);
	    
	ros::Rate loop_rate(5);

	while(pc==NULL){
		
		loop_rate.sleep();
	}
	
	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  	viewer->setBackgroundColor (0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pc);
	viewer->addPointCloud<PointT> (pc, rgb, "cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
	viewer->initCameraParameters ();

    high_resolution_clock::time_point start_t = high_resolution_clock::now();
	while(ros::ok)
    {   
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pc);
        viewer->updatePointCloud<PointT> (pc, rgb, "cloud");
		viewer->spinOnce (100);
		
		loop_rate.sleep();

		auto dt = std::chrono::duration_cast<std::chrono::seconds>(high_resolution_clock::now() - start_t);
		if(dt.count()>30){
			break;
		}
    }

    // Stop depth stream
	ret = camera->stopStream(STREAM_TYPE_DEPTH);	
	if (ret != SUCCESS)
	{
		std::cout<<"Failed to stop camera depth stream. Error code: " << std::to_string(ret)<<std::endl;
		return false;
	}	

	// Stop rgb stream
	ret = camera->stopStream(STREAM_TYPE_RGB);
	if (ret != SUCCESS)
	{
		std::cout<<"Failed to stop camera rgb stream. Error code: " << std::to_string(ret)<<std::endl;
		return false;
	}
	
	// Disconnect camera
	ret = camera->disconnect();
	if (ret != SUCCESS)
	{
		std::cout<<"Acusense camera disconnect failed with error code " << std::to_string(ret)<<std::endl;
		return false;
	}

	std::cout<<"Successfully disconnected Acusense camera"<<std::endl;
	
}