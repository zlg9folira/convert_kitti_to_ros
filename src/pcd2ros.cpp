//* To run:  rosrun convert_pcd_to_ros pcd2ros
#include "ros/ros.h"
#include <iostream>
#include <std_msgs/Float64.h>
#include <iomanip>
#include <glob.h>
#include <fstream>
#include <stdlib.h>
#include <rosbag/bag.h>

//---pcl -includes
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>



std::vector<std::string> globVector(const std::string& pattern){
	glob_t glob_result;
	glob(pattern.c_str(),GLOB_TILDE,NULL,&glob_result);
	std:: vector<std::string> files;
	for(unsigned int i=0;i<glob_result.gl_pathc;++i){
		files.push_back(std::string(glob_result.gl_pathv[i]));
	}
	globfree(&glob_result);
	return files;
}

void read_pcd_to_cloud(std::string pcd,pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	if(pcl::io::loadPCDFile<pcl::PointXYZ> (pcd,*cloud)==-1)
		PCL_ERROR("Couldn't load file \n");
}

static void show_usage()
{
	printf("Usage:\trosrun convert_pcd_to_ros pcd2ros\nRequired options:\n");
	printf("\t--folder /path/to/read/pcd/files\n");
	printf("\t--outpath /path/to/ROSBAG.bag\n");
}

int main(int argc, char** argv)
{

	std::string topic_name = "/os1_cloud_node/points";
	std::string frame_name = "os1_lidar";
	int sensor_rate = 10;
	std::string folder;
	std::string bagfile;

	if (argc < 2) {
		std::cout << "Insufficient number of arguments: " << argc << std::endl;
		show_usage();
		return 1;
	}
	std::vector <std::string> sources;
	for (int i = 1; i < argc; ++i) {
		std::string arg = argv[i];
		if ((arg == "-h") || (arg == "--help")) {
			show_usage();
			return 0;
		}else if ((arg == "-f") || (arg == "--folder")) {
			if (i + 1 < argc) { // Make sure we aren't at the end of argv!
				folder = argv[++i]; // Increment 'i' so we don't get the argument as the next argv[i].
				printf("Input folder:\t%s\n",folder.c_str());
			} else { // Uh-oh, there was no argument to the destination option.
				std::cerr << "--folder option requires one argument." << std::endl;
				return 1;
			}  

		}else if ((arg == "-o") || (arg == "--outpath")) {
			if (i + 1 < argc) { // Make sure we aren't at the end of argv!
				bagfile = argv[++i]; // Increment 'i' so we don't get the argument as the next argv[i].
				printf("Output BAG:\t%s\n",bagfile.c_str());
			} else { // Uh-oh, there was no argument to the destination option.
				std::cerr << "--outpath option requires one argument." << std::endl;
				return 1;
			}  
		} else {
			std::cout << "INVALID Option!\n";
			show_usage();
		}
	}


	// initialise the node
	ros::init(argc, argv, "pcd2ros"); // name of node
	std::cout << "pcd2ros node initialized" << std::endl;
	ros::NodeHandle nh;
	ros:: Publisher publisher = nh.advertise<pcl::PCLPointCloud2> (topic_name, 1);

	rosbag::Bag bag;
	bag.open(bagfile,rosbag::bagmode::Write);

	int count =0;
	// handle ROS communication events
	ros::Rate loop_rate(1);


	// make a loop over all input PCD files
	std::string globVector_files = folder + "/*";
	std::vector<std::string> files = globVector(globVector_files);
	std::cout<< "\nsize of files: " << files.size() << std::endl;
	double time = ros::Time::now().toSec(); 
	ros::Time stamp =   ros::Time::now();   
	for (int j=1;j<files.size();j++){
		std::string pose3PcdName = files[j];
		std::string pose3Pcd =  pose3PcdName;
		pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;

		read_pcd_to_cloud(pose3Pcd,input_cloud);

		sensor_msgs::PointCloud2 myCloud2;
		//pcl_conversions::fromPCL(*input_cloud,myCloud2);
		pcl::toROSMsg(*input_cloud,myCloud2);
		// Publish the data
		myCloud2.header.frame_id= frame_name;
		ros::Duration delta_t (0, sensor_rate/0.0000001);
		stamp += delta_t; 
		myCloud2.header.stamp = stamp;

		publisher.publish (myCloud2);
		bag.write(topic_name,myCloud2.header.stamp,myCloud2);
		ros::spinOnce();
		std::cout << "count:\t" << count << "/" << files.size() << "\tCloud size: " 
			  << input_cloud->points.size() << " points" << std::setprecision (25)
			  << " timestamp:" << myCloud2.header.stamp << std::endl;
		count++;
	}
	bag.close();
	std::cout<< "Bag file write finish"<< std::endl;
	return 1;
}

