#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <iostream>
#include <cmath>
#include <optional>

class AutonomousNavigation : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr location_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr orientation_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr movement_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

    
    double target_lat_;
    double target_lon_;
    double current_heading_; 
    bool target_reached_ = false;
    bool obstacle_detected_ = false;
    enum Mode { NAVIGATING, AVOIDING };
    Mode mode_ = NAVIGATING;


    double haversineDistance(double lat1, double lon1, double lat2, double lon2)
    {
        const double R = 6371000; 
        const double to_rad = M_PI / 180.0;

        double dlat = (lat2 - lat1) * to_rad;
        double dlon = (lon2 - lon1) * to_rad;

        double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
                   std::cos(lat1 * to_rad) * std::cos(lat2 * to_rad) *
                   std::sin(dlon / 2) * std::sin(dlon / 2);

        double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
        return R * c;
    }

    void publishVelocity(double linear, double angular)
    {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = linear;
        twist_msg.angular.z = angular;
        movement_pub_->publish(twist_msg);
    }

    double calculateHeading(double lat1, double lon1, double lat2, double lon2)
    {
        const double to_rad = M_PI / 180.0;
        double dlon = (lon2 - lon1) * to_rad;

        double y = std::sin(dlon) * std::cos(lat2 * to_rad);
        double x = std::cos(lat1 * to_rad) * std::sin(lat2 * to_rad) -
                   std::sin(lat1 * to_rad) * std::cos(lat2 * to_rad) * std::cos(dlon);

        double initial_heading = std::atan2(y, x);
        return M_PI + M_PI_2 - initial_heading;
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    auto q = msg->orientation;
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    current_heading_ = normalizeAngle(std::atan2(siny_cosp, cosy_cosp));
}


    double normalizeAngle(double angle)
    {
        while (angle > M_PI)
            angle -= 2 * M_PI;
        while (angle < -M_PI)
            angle += 2 * M_PI;
        return angle;
    }

    void take_action(const pcl::PointXYZ& nearest_point,  double linear_speed, geometry_msgs::msg::Twist& cmd_vel)
    {
        double angular_speed = 2.0;
        double threshold = 1.0;
    
        double distance = std::sqrt(nearest_point.x * nearest_point.x + nearest_point.y * nearest_point.y);
    
        if (distance < threshold) {
            if (nearest_point.y > 0.2) {
                // Obstacle to the left
                cmd_vel.linear.x = 0.4;
                cmd_vel.angular.z = -0.8;
            } else if (nearest_point.y < -0.3) {
                // Obstacle to the right
                cmd_vel.linear.x = 0.4;
                cmd_vel.angular.z = 0.8;
            } else if (nearest_point.y < -0.5) {
                cmd_vel.linear.x = 0.5;
                cmd_vel.angular.z = 1.0;
            } else if (nearest_point.y > 0.5) {
                
                cmd_vel.linear.x = 0.5;
                cmd_vel.angular.z = 1.0;
            } else {
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = angular_speed;
            }
        } else {
            cmd_vel.linear.x = linear_speed;
            cmd_vel.angular.z = 0.0;
        }
    }
    

void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(0.05f, 0.05f, 0.05f);
    voxel.filter(*cloud_voxel);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.02);
    seg.setMaxIterations(100);
    seg.setInputCloud(cloud_voxel);
    seg.segment(*inliers, *coefficients);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_ground(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_voxel);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_no_ground);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_no_ground);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.05);
    ec.setMinClusterSize(20);
    ec.setMaxClusterSize(15000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_no_ground);
    ec.extract(cluster_indices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& indices : cluster_indices) {
        for (const auto& idx : indices.indices) {
            clustered_cloud->points.push_back(cloud_no_ground->points[idx]);
        }
    }

    clustered_cloud->width = clustered_cloud->points.size();
    clustered_cloud->height = 1;
    clustered_cloud->is_dense = true;

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*clustered_cloud, output);
    output.header = cloud_msg->header;
    cloud_pub_->publish(output);

    pcl::PointXYZ nearest_point;
    double min_distance = std::numeric_limits<double>::max();
    bool obstacle_within_range = false;

    for (const auto& cluster : cluster_indices) {
        for (const auto& index : cluster.indices) {
            const auto& pt = cloud_no_ground->points[index];  
            double dist = std::sqrt(pt.x * pt.x + pt.y * pt.y);
            if (dist < min_distance) {
                min_distance = dist;
                nearest_point = pt;
            }
            if (dist < 1.5) {
                obstacle_within_range = true;
            }
        }
    }
    
    if (obstacle_within_range) {
        if (mode_ != AVOIDING) {
            RCLCPP_INFO(this->get_logger(), "Obstacle within 1.5m. Entering AVOIDING mode.");
            mode_ = AVOIDING;
        }

        geometry_msgs::msg::Twist cmd_vel;
        double linear_speed = 0.3;
        take_action(nearest_point, linear_speed, cmd_vel);
        movement_pub_->publish(cmd_vel);

    } else {
        if (mode_ != NAVIGATING) {
            RCLCPP_INFO(this->get_logger(), "No close obstacle. Resuming NAVIGATING mode.");
            mode_ = NAVIGATING;
        }
    }
}


void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    if (target_reached_) {
        return;
    }

    if (mode_ == AVOIDING) {
        RCLCPP_INFO(this->get_logger(), "Currently in AVOIDING mode. Skipping GPS-based navigation.");
        return;
    }

    double curr_lat = msg->latitude;
    double curr_lon = msg->longitude;

    double dist = haversineDistance(curr_lat, curr_lon, target_lat_, target_lon_);

    if (dist < 0.5) {
        RCLCPP_INFO(this->get_logger(), "Target reached. Stopping.");
        publishVelocity(0.0, 0.0);
        target_reached_ = true;
        rclcpp::shutdown();
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Currently in NAVIGATING mode.");

    double desired_heading = calculateHeading(curr_lat, curr_lon, target_lat_, target_lon_);
    desired_heading = normalizeAngle(desired_heading);
    double angular_error = normalizeAngle(desired_heading - current_heading_);

    double angular_velocity = std::clamp(angular_error * 1.5, -1.0, 1.0);
    double linear_velocity = std::max(0.1, 1.0 - std::abs(angular_error));

    RCLCPP_INFO(this->get_logger(), "Navigating to target. Distance: %.2f m | Angular error: %.2f rad", dist, angular_error);

    publishVelocity(linear_velocity, angular_velocity);
}


public:
    AutonomousNavigation() : Node("autonomous_navigation")
    {
        std::cout << "Enter target latitude: ";
        std::cin >> target_lat_;

        std::cout << "Enter target longitude: ";
        std::cin >> target_lon_;

        orientation_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&AutonomousNavigation::imuCallback, this, std::placeholders::_1));

        location_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps", 10, std::bind(&AutonomousNavigation::gpsCallback, this, std::placeholders::_1));
        
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/d430/depth/points", 10, std::bind(&AutonomousNavigation::cloudCallback, this, std::placeholders::_1));
        
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/obstacles", 10);

        movement_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutonomousNavigation>());
    rclcpp::shutdown();
    return 0;
}