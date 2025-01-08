#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <set>
#include <unordered_set>
#include <boost/format.hpp>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <velodyne_pcl/point_types.h>

#include <armadillo>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>

//Plane model segmentation
//http://pointclouds.org/documentation/tutorials/planar_segmentation.php#planar-segmentation

ros::Publisher pub;

void ransac_callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr inlierPoints (new pcl::PointCloud<pcl::PointXYZI>),
										inlierPoints_neg (new pcl::PointCloud<pcl::PointXYZI>);
    
	pcl::PCLPointCloud2* cloud_intermediate = new pcl::PCLPointCloud2;
    pcl::PointCloud<pcl::PointXYZI> cloud;

	pcl_conversions::toPCL(*input, *cloud_intermediate);
	// Convert PCL::PointCloud2 to PCL::PointCloud<PointXYZI>
    pcl::fromPCLPointCloud2(*cloud_intermediate, cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_p = cloud.makeShared();

	// Object for storing the plane model coefficients.
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());


	// 오프젝트 생성 Create the segmentation object.
	pcl::SACSegmentation<pcl::PointXYZI> seg;
	seg.setOptimizeCoefficients (true);       //(옵션) // Enable model coefficient refinement (optional).
	seg.setModelType (pcl::SACMODEL_PLANE);    //적용 모델  // Configure the object to look for a plane.
	seg.setMethodType (pcl::SAC_RANSAC);       //적용 방법   // Use RANSAC method.
	seg.setMaxIterations (1000);
	//seg.setMaxIterations (1000);               //최대 실행 수
	seg.setDistanceThreshold (0.0001);   
	//seg.setDistanceThreshold (0.05);       //inlier로 처리할 거리 정보   // Set the maximum allowed distance to the model.
	seg.setRadiusLimits(0, 0.1);     // cylinder경우, Set minimum and maximum radii of the cylinder.
	seg.setInputCloud (cloud_p);                //입력 
	seg.segment (*inliers, *coefficients);    //세그멘테이션 적용 


	pcl::copyPointCloud<pcl::PointXYZI>(*cloud_p, *inliers, *inlierPoints);

	//[옵션]] 바닥 제거 결과 얻기 
	//Extracting indices from a PointCloud
	//http://pointclouds.org/documentation/tutorials/extract_indices.php

	pcl::ExtractIndices<pcl::PointXYZI> extract;
	extract.setInputCloud (cloud_p);
	extract.setIndices (inliers);
	extract.setNegative (true); //false
	extract.filter (*inlierPoints_neg);

    pub.publish(*inlierPoints_neg);
}


int main(int argc,char** argv)
{
    ros::init(argc, argv, "lidar_ransac");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("lidar_roi", 1, ransac_callback);
    pub = nh.advertise<sensor_msgs::PointCloud2> ("lidar_ransac",1);

    std::cout << "ransac complete" << std::endl;

    ros::spin();
}
