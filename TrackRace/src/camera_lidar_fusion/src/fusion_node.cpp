/* 
다음 코드는 윤수빈이 장애물 2D 플롯을 위해 수정한 코드
** 수정 시, 꼭 백업해둘 것 !!! **
** Parameter 다 맞춰둔거니까 건들지 마세요! **
*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h> 
#include <opencv2/opencv.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <std_msgs/Float32MultiArray.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>

#include <mlpack.hpp>
#include <mlpack/methods/dbscan/dbscan.hpp>
#include <armadillo>

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

class FusionNode {
public:
    FusionNode() : extrinsic_matrix_(Eigen::Matrix4d::Identity()), intrinsic_matrix_(Eigen::Matrix3d::Identity()) {
        camera_sub_ = nh_.subscribe("/camera1/usb_cam1/image_raw", 1, &FusionNode::cameraCallback, this);
        marker_sub_ = nh_.subscribe("/visualization_marker_array", 1, &FusionNode::markerCallback, this);
        lidar_sub_ = nh_.subscribe("/lidar_ransac", 1, &FusionNode::lidarCallback, this); 

        image_pub_ = nh_.advertise<sensor_msgs::Image>("/camera_lidar_fusion/output", 1);
        obstacle_info_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/obstacle_info", 1);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
        point_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("obstacle_info_points", 1); 

        // 내부 파라미터
        intrinsic_matrix_ << 440.448278884377, 0, 321.492290400808,
                             0, 455.317535181246, 135.774750100701,
                             0, 0, 1;

        // 외부 파라미터
        extrinsic_matrix_ << -0.0405243758643159, -0.999178255809739, 0.000766862318099915, -0.0201615025392103,
                             -0.431570238357808, 0.0168112786088913, -0.901922674221753, 0.536260686706105,
                              0.901168632568063, -0.0368808084041212, -0.431896864594854, 1.23601061670107,
                              0, 0, 0, 1;
    }

    void cameraCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            current_image_ = cv_ptr->image;
            current_image_msg_ = msg;
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

    void markerCallback(const visualization_msgs::MarkerArray::ConstPtr& marker_array) {
        ROS_INFO("Marker callback triggered");
        if (current_image_.empty()) {
            ROS_WARN("No image received yet.");
            return;
        }

        cv::Mat overlay_image = current_image_.clone();

        for (const auto& marker : marker_array->markers) {
            Eigen::Vector3d position(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);
            
            double x = position[0];
            double y = position[1];
            double z = position[2];

            // 객체 거리 계산
            double distance = position.norm();
            ROS_INFO("Object coordinates: x = %f, y = %f, z = %f", x, y, z);
            ROS_INFO("Object distance: %f meters", distance);

            // 3D 좌표를 2D 이미지 평면으로 투영
            Eigen::Vector2d pixel_coords = projectTo2D(position);

            // 이미지에 사각형 그리기
            cv::rectangle(overlay_image, cv::Point(pixel_coords[0] - 5, pixel_coords[1] - 5),
                          cv::Point(pixel_coords[0] + 5, pixel_coords[1] + 5), cv::Scalar(0, 255, 0), 2);

            // 거리와 좌표를 이미지에 표시
            std::string distance_text = cv::format("Distance: %.2f m", distance);
            std::string xy_coordinate = cv::format("x: %.2f, y: %.2f", x, y);

            int baseline = 0;
            double fontScale = 0.3; // 폰트 크기 축소
            int thickness = 0.8;     // 두께 축소
            cv::Size textSize = cv::getTextSize(distance_text, cv::FONT_HERSHEY_SIMPLEX, fontScale, thickness, &baseline);
            cv::Point text_origin(pixel_coords[0] + 10, pixel_coords[1]);
            cv::putText(overlay_image, distance_text, text_origin, cv::FONT_HERSHEY_SIMPLEX, fontScale, cv::Scalar(0, 255, 0), thickness);
            cv::putText(overlay_image, xy_coordinate, text_origin + cv::Point(0, textSize.height + 5), cv::FONT_HERSHEY_SIMPLEX, fontScale, cv::Scalar(0, 255, 0), thickness);

            // 장애물 정보 메시지 생성 및 퍼블리시
            std_msgs::Float32MultiArray obstacle_msg;
            obstacle_msg.data.push_back(x);
            obstacle_msg.data.push_back(y);
            obstacle_msg.data.push_back(z);
            obstacle_msg.data.push_back(pixel_coords[0]);
            obstacle_msg.data.push_back(pixel_coords[1]);
            obstacle_msg.data.push_back(distance);
            obstacle_info_pub_.publish(obstacle_msg);
        }

        cv_bridge::CvImage out_msg;
        out_msg.header = current_image_msg_->header;
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        out_msg.image = overlay_image;
        image_pub_.publish(out_msg.toImageMsg());
    }

    void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
        // 포인트 클라우드 변환
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);

        // ROI 필터 적용
        pcl::PassThrough<pcl::PointXYZ> pass;

        pass.setInputCloud(cloud);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(-0.2, 20.0); // 앞뒤 거리 제한
        pass.filter(*cloud);

        pass.setFilterFieldName("y");
        pass.setFilterLimits(-8.0, 8.0); // 좌우 거리 제한
        pass.filter(*cloud);

        pass.setFilterFieldName("z");
        pass.setFilterLimits(-0.8, 0.6); // 상하 거리 제한
        pass.filter(*cloud);


        // 데이터 변환
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
        int marker_id = 0;

        for (const auto& cluster : clusters)
        {
            pcl::PointXYZ min_pt, max_pt;
            pcl::getMinMax3D(*cluster.second, min_pt, max_pt);

            double width = max_pt.x - min_pt.x;
            double depth = max_pt.y - min_pt.y;
            double height = max_pt.z - min_pt.z;

        // ---------------------------------------------------------------------------------
            if (width <= 0.5 && depth <= 0.5 && height >= 0.1 && height <= 1.0) //라바콘
            //if (width <= 2 && depth <= 2 && height >= 0.1 && height <= 2.0) //정적장애물
        // ---------------------------------------------------------------------------------
            {
                // 클러스터의 중심점 계산
                Eigen::Vector3d centroid(0, 0, 0);
                for (const auto& point : cluster.second->points) {
                    centroid += Eigen::Vector3d(point.x, point.y, point.z);
                }
                centroid /= cluster.second->points.size();

                double x = centroid[0];
                double y = centroid[1];
                double z = centroid[2];
                double distance = centroid.norm();

                // 장애물 정보 메시지 생성 및 퍼블리시
                std_msgs::Float32MultiArray obstacle_msg;
                obstacle_msg.data.push_back(x);
                obstacle_msg.data.push_back(y);
                obstacle_msg.data.push_back(z);
                obstacle_msg.data.push_back(-1); // 카메라 픽셀 좌표는 없음
                obstacle_msg.data.push_back(-1);
                obstacle_msg.data.push_back(distance);
                obstacle_info_pub_.publish(obstacle_msg);

                // 마커 생성
                visualization_msgs::Marker marker;
                marker.header.frame_id = "velodyne";
                marker.header.stamp = ros::Time::now();
                marker.ns = "cluster_boxes";
                marker.id = marker_id++;
                marker.type = visualization_msgs::Marker::CUBE;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = centroid[0];
                marker.pose.position.y = centroid[1];
                marker.pose.position.z = centroid[2];
                marker.scale.x = width;
                marker.scale.y = depth;
                marker.scale.z = height;
                marker.color.a = 0.5;
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                marker.lifetime = ros::Duration();

                marker_array.markers.push_back(marker);
            }
        }

        marker_pub_.publish(marker_array);
    }

private:
    Eigen::Vector2d projectTo2D(const Eigen::Vector3d& pt_3d) {
        Eigen::Vector4d pt_3d_homogeneous(pt_3d[0], pt_3d[1], pt_3d[2], 1.0);
        Eigen::Vector4d pt_camera_frame = extrinsic_matrix_ * pt_3d_homogeneous;
        Eigen::Vector3d pt_2d_homogeneous = intrinsic_matrix_ * pt_camera_frame.head<3>();

        Eigen::Vector2d pt_2d;
        pt_2d[0] = pt_2d_homogeneous[0] / pt_2d_homogeneous[2];
        pt_2d[1] = pt_2d_homogeneous[1] / pt_2d_homogeneous[2];

        return pt_2d;
    }

    ros::NodeHandle nh_;
    ros::Subscriber camera_sub_;
    ros::Subscriber marker_sub_;
    ros::Subscriber lidar_sub_;
    ros::Publisher image_pub_;
    ros::Publisher obstacle_info_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher point_pub_;
    cv::Mat current_image_;
    sensor_msgs::ImageConstPtr current_image_msg_;
    Eigen::Matrix4d extrinsic_matrix_;
    Eigen::Matrix3d intrinsic_matrix_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "fusion_node");
    FusionNode fusion_node;
    ros::spin();
    return 0;
}