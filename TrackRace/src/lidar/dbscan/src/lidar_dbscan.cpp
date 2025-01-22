/*----------------------------------------------------*/
/*----------------------------------------------------*/
/*----------------------------------------------------*/
/*----------------------------------------------------*/
/*----------------------------------------------------*/
/* 아래 코드는 윤수빈이 장애물 회피를 위해 작성한 코드임_240902 */
/*----------------------------------------------------*/
/*----------------------------------------------------*/
/*----------------------------------------------------*/
/*----------------------------------------------------*/
/*----------------------------------------------------*/


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <mlpack.hpp>
#include <mlpack/methods/dbscan/dbscan.hpp>
#include <armadillo>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32MultiArray.h>
#include <pcl/filters/passthrough.h>

ros::Publisher pub;
ros::Publisher marker_pub;
ros::Publisher obstacle_pub;
ros::Publisher point_pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    
    // 입력 데이터가 비었는지 확인 - 민환
    if (cloud->points.empty()) {
        ROS_WARN("DBSCAN received an empty point cloud. Skipping.");
        return;
    }

    // ROI 필터 적용
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.0, 20.0); // 앞뒤 거리 제한
    pass.filter(*cloud);

    pass.setFilterFieldName("y");
    pass.setFilterLimits(-2.0, 2.0); // 좌우 거리 제한
    pass.filter(*cloud);

    pass.setFilterFieldName("z");
    pass.setFilterLimits(-0.5, 2.0); // 상하 거리 제한
    pass.filter(*cloud);
    
    // 필터링 후 데이터가 비었는지 확인 - 민환 추가
    if (cloud->points.empty()) {
        ROS_WARN("Point cloud is empty after filtering. Skipping DBSCAN.");
        return;
    }

    arma::mat data(3, cloud->points.size());
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        data(0, i) = cloud->points[i].x;
        data(1, i) = cloud->points[i].y;
        data(2, i) = cloud->points[i].z;
    }

    // DBSCAN 파라미터 설정
    double epsilon = 0.5; // 클러스터링 거리
    size_t minPoints = 5; // 최소 포인트 수
    mlpack::dbscan::DBSCAN<> dbscan(epsilon, minPoints);

    arma::Row<size_t> assignments;
    dbscan.Cluster(data, assignments);
    
    // DBSCAN 결과 확인 - 민환
    if (assignments.n_elem == 0 || arma::max(assignments) == size_t(-1)) {
        ROS_WARN("No clusters found by DBSCAN. Skipping visualization.");
        return;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::map<size_t, pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;

    for (size_t i = 0; i < assignments.n_elem; ++i)
    {
        if (clusters.find(assignments[i]) == clusters.end())
        {
            clusters[assignments[i]] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        }
        clusters[assignments[i]]->points.push_back(pcl::PointXYZ(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z));
    }

    visualization_msgs::MarkerArray marker_array;
    std_msgs::Float32MultiArray obstacle_array;
    std_msgs::Float32MultiArray points_array;
    int marker_id = 0;

    for (const auto& cluster : clusters)
    {
        pcl::PointXYZ min_pt, max_pt;
        pcl::getMinMax3D(*cluster.second, min_pt, max_pt);

        double width = max_pt.x - min_pt.x;
        double depth = max_pt.y - min_pt.y;
        double height = max_pt.z - min_pt.z;

        if (width <= 0.5 && depth <= 0.5 && height >= 0.2 && height <= 1.0)
        {
            // 네 가지 점 계산
            points_array.data.clear();
            points_array.data.push_back(min_pt.x);
            points_array.data.push_back(min_pt.y);

            points_array.data.push_back(min_pt.x);
            points_array.data.push_back(max_pt.y);

            points_array.data.push_back(max_pt.x);
            points_array.data.push_back(min_pt.y);

            points_array.data.push_back(max_pt.x);
            points_array.data.push_back(max_pt.y);

            // Publish points array
            point_pub.publish(points_array);

            // result_cloud에 포인트 추가
            for (const auto& point : cluster.second->points)
            {
                pcl::PointXYZRGB colored_point;
                colored_point.x = point.x;
                colored_point.y = point.y;
                colored_point.z = point.z;
                colored_point.r = 255;
                colored_point.g = 0;
                colored_point.b = 0;
                result_cloud->points.push_back(colored_point);
            }

            visualization_msgs::Marker marker;
            marker.header.frame_id = "velodyne";
            marker.header.stamp = ros::Time::now();
            marker.ns = "cluster_boxes";
            marker.id = marker_id++;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = (min_pt.x + max_pt.x) / 2.0;
            marker.pose.position.y = (min_pt.y + max_pt.y) / 2.0;
            marker.pose.position.z = (min_pt.z + max_pt.z) / 2.0;
            marker.scale.x = width;
            marker.scale.y = depth;
            marker.scale.z = height;
            marker.color.a = 0.5;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.lifetime = ros::Duration();

            marker_array.markers.push_back(marker);

            // Add obstacle data
            obstacle_array.data.push_back(marker.pose.position.x);
            obstacle_array.data.push_back(marker.pose.position.y);
            obstacle_array.data.push_back(marker.pose.position.z);
            obstacle_array.data.push_back(width);
            obstacle_array.data.push_back(depth);
            obstacle_array.data.push_back(height);
        }
    }

    marker_pub.publish(marker_array);
    obstacle_pub.publish(obstacle_array);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*result_cloud, output);
    output.header.frame_id = "velodyne";
    pub.publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_dbscan");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("lidar_ransac", 1, cloud_cb);
    pub = nh.advertise<sensor_msgs::PointCloud2>("lidar_dbscan", 1);
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
    obstacle_pub = nh.advertise<std_msgs::Float32MultiArray>("obstacles", 1);
    point_pub = nh.advertise<std_msgs::Float32MultiArray>("obstacle_info", 1);
    
    std::cout << "dbscan complete" << std::endl;

    ros::spin();

    return 0;
}


