/*----------------------------------------------------*/
/*----------------------------------------------------*/
/*----------------------------------------------------*/
/*----------------------------------------------------*/
/*----------------------------------------------------*/
/*------------민환 outlier 코드 수정_250103 -----------*/
/*----------------------------------------------------*/
/*----------------------------------------------------*/
/*----------------------------------------------------*/
/*----------------------------------------------------*/
/*----------------------------------------------------*/

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
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

ros::Publisher pub_sor;
ros::Publisher pub_roir;

// outlier 제거 콜백 함수
void outlier_removal_callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // ROS PointCloud2 메시지 PCL PointCloud 형식으로 전환
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_sor(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_roir(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(*input, *cloud);

    // Statistical Outlier Removal (SOR) 필터 적용
    // 주변 이웃과 큰 차이를 보이는 점 제거거
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50); // 이웃한 점의 개수
    sor.setStddevMulThresh(1.0); // outlier로 간주할 거리 기준 배수
    sor.filter(*cloud_filtered_sor);

    // SOR 필터링 결과 퍼블리싱
    sensor_msgs::PointCloud2 output_sor;
    pcl::toROSMsg(*cloud_filtered_sor, output_sor);
    output_sor.header = input->header;
    pub_sor.publish(output_sor);

    // Radius Outlier Removal (ROIR) 필터 적용
    // 반경 내 이웃 점의 개수를 기반으로 점 제거
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> roir;
    roir.setInputCloud(cloud_filtered_sor); // SOR 출력 데이터를 입력으로 사용
    roir.setRadiusSearch(0.5); // 탐색 반경 설정 
    roir.setMinNeighborsInRadius(5); // 반경 내 최소 이웃 점의 개수 
    roir.filter(*cloud_filtered_roir);

    // ROIR 필터링 결과 퍼블리싱 
    sensor_msgs::PointCloud2 output_roir;
    pcl::toROSMsg(*cloud_filtered_roir, output_roir);
    output_roir.header = input->header;
    pub_roir.publish(output_roir);
}

int main(int argc, char** argv)
{
    // ROS 노드 초기화 
    ros::init(argc, argv, "outlier_removal");
    ros::NodeHandle nh;

    // ROI 필터링 이후의 포인트 클라우드 데이터 구독 
    ros::Subscriber sub = nh.subscribe("lidar_roi", 1, outlier_removal_callback);

    pub_sor = nh.advertise<sensor_msgs::PointCloud2>("lidar_outlier_sor", 1);
    pub_roir = nh.advertise<sensor_msgs::PointCloud2>("lidar_outlier_roir", 1);

    ROS_INFO("outlier complete");

    ros::spin();

    return 0;
}
