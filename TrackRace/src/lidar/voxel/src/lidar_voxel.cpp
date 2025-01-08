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


#include <armadillo>

ros::Publisher pub;

void cloud_callBack(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // container for original and filtered data
    pcl::PCLPointCloud2 * cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud); // pointer for dynamic cloud
    pcl::PCLPointCloud2 cloud_filtered;

    //conversion ROS sensor_msg/PointCloud2 -> pcl lib의 PointCloud2 type 
    pcl_conversions::toPCL(*input, *cloud);
    
    // pcl 의 VoxelGrid 타입
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr);
    // 샘플링 하는 방법 이거 너무 작게 하면 샘플링 에러 메세지 뜸 고것을 주의 하자
    //leaf size  1cm 격자의 x, y, z 크기
    //sor.setLeafSize (0.09f, 0.09f, 0.11f);
    sor.setLeafSize (0.07f, 0.07f, 0.09f);
    sor.filter(cloud_filtered);

    sensor_msgs::PointCloud2 output;
    //pcl lib PointCLoud2 type -> ROS sensor_msgs data
    pcl_conversions::moveFromPCL(cloud_filtered, output);
    pub.publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_voxel");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, cloud_callBack);
    pub = nh.advertise<sensor_msgs::PointCloud2> ("lidar_voxel", 1);

    std::cout << "voxel complete" << std::endl;

    ros::spin();
}
