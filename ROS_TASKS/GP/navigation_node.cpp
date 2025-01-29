#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <cmath>
#include <optional>

class AutonomousNavigation : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr location_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr orientation_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr movement_pub_;
    
    double target_lat_;
    double target_lon_;
    double current_heading_; 
    bool target_reached_ = false;

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
        current_heading_ = std::atan2(siny_cosp, cosy_cosp);
    }

    double normalizeAngle(double angle)
    {
        while (angle > M_PI)
            angle -= 2 * M_PI;
        while (angle < -M_PI)
            angle += 2 * M_PI;
        return angle;
    }

    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        if (target_reached_) {
            publishVelocity(0.0, 0.0);
            return;
        }

        double curr_lat = msg->latitude;
        double curr_lon = msg->longitude;

        double dist = haversineDistance(curr_lat, curr_lon, target_lat_, target_lon_);
        double desired_heading = calculateHeading(curr_lat, curr_lon, target_lat_, target_lon_);

        desired_heading = normalizeAngle(desired_heading);
        current_heading_ = normalizeAngle(current_heading_);

        double angular_error = normalizeAngle(desired_heading - current_heading_);

        RCLCPP_INFO(this->get_logger(), "----------------------------------------");
        RCLCPP_INFO(this->get_logger(), "Navigation Status Update:");
        RCLCPP_INFO(this->get_logger(), "Distance to Target: %.4f meters", dist);
        RCLCPP_INFO(this->get_logger(), "Current Orientation: %.4f rad, Desired Orientation: %.4f rad", 
                    current_heading_, desired_heading);
        RCLCPP_INFO(this->get_logger(), "Angular Error: %.4f radians", angular_error);

        if (dist < 0.5) {
            publishVelocity(0.0, 0.0);
            RCLCPP_INFO(this->get_logger(), "----------------------------------------");
            RCLCPP_INFO(this->get_logger(), "Reached the destination successfully!");
            RCLCPP_INFO(this->get_logger(), "----------------------------------------");
            target_reached_ = true;
            return;
        }

        double angular_velocity = angular_error;
        double linear_velocity = std::max(0.0, 1.0 - std::abs(angular_error));
        
        publishVelocity(linear_velocity, angular_velocity);
    }

public:
    AutonomousNavigation() : Node("autonomous_navigation")
    {
        std::cout << "Enter target latitude: ";
        std::cin >> target_lat_;

        std::cout << "Enter target longitude: ";
        std::cin >> target_lon_;

        location_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps", 10, std::bind(&AutonomousNavigation::gpsCallback, this, std::placeholders::_1));
        
        orientation_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&AutonomousNavigation::imuCallback, this, std::placeholders::_1));

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
