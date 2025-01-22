#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <chrono>
#include <memory>
#include <cmath>

using namespace std::chrono_literals;

class RoverNavigation : public rclcpp::Node {
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    geometry_msgs::msg::Twist cmd_vel_;

    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_yaw_ = 0.0;
    double target_x_ = 0.0;
    double target_y_ = 0.0;

    const double DISTANCE_THRESHOLD = 0.2;
    const double FINAL_DISTANCE_CHECK = 0.25;
    const double ANGULAR_THRESHOLD = 0.15;
    const double VELOCITY_THRESHOLD = 0.01;

    const double MAX_LINEAR_SPEED = 1.0;
    const double MIN_LINEAR_SPEED = 0.2;
    const double MAX_ANGULAR_SPEED = 1.0;
    const double KP_LINEAR = 1.0;
    const double KP_ANGULAR = 2.0;

    int stop_confirmation_counter_ = 0;
    const int STOP_CONFIRMATION_THRESHOLD = 25;
    bool stopping_sequence_initiated_ = false;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );

        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, current_yaw_);
    }

    double limit_velocity(double velocity, double max_velocity, double min_velocity = 0.0) {
        double limited = std::max(-max_velocity, std::min(velocity, max_velocity));
        if (limited > 0) {
            return std::max(limited, min_velocity);
        } else if (limited < 0) {
            return std::min(limited, -min_velocity);
        }
        return 0.0;
    }

    void execute_stop_sequence() {
        for (int i = 0; i < 5; i++) {
            cmd_vel_.linear.x = 0.0;
            cmd_vel_.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd_vel_);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        RCLCPP_INFO(this->get_logger(), "Stop sequence executed - Robot should be stationary");
    }

    bool validate_stop_position() {
        double dx = target_x_ - current_x_;
        double dy = target_y_ - current_y_;
        double final_distance = sqrt(dx*dx + dy*dy);

        if (final_distance <= FINAL_DISTANCE_CHECK) {
            RCLCPP_INFO(this->get_logger(), "Final position validated - Distance to target: %.3f meters", final_distance);
            return true;
        }
        return false;
    }

public:
    RoverNavigation() : Node("rover_navigation") {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&RoverNavigation::odom_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Enhanced Rover Navigation Node initialized");
    }

    void set_target(double x, double y) {
        target_x_ = x;
        target_y_ = y;
        stopping_sequence_initiated_ = false;
        stop_confirmation_counter_ = 0;
        RCLCPP_INFO(this->get_logger(), "New target set: x=%.2f, y=%.2f", x, y);
    }

    bool move_to_target() {
        rclcpp::Rate rate(50);

        while (rclcpp::ok()) {
            double dx = target_x_ - current_x_;
            double dy = target_y_ - current_y_;
            double distance = sqrt(dx*dx + dy*dy);

            if (distance < DISTANCE_THRESHOLD) {
                if (!stopping_sequence_initiated_) {
                    RCLCPP_INFO(this->get_logger(), "Approaching target - Initiating stop sequence");
                    stopping_sequence_initiated_ = true;
                    execute_stop_sequence();
                }

                stop_confirmation_counter_++;
                if (stop_confirmation_counter_ >= STOP_CONFIRMATION_THRESHOLD) {
                    if (validate_stop_position()) {
                        RCLCPP_INFO(this->get_logger(), "Target reached and stop confirmed!");
                        return true;
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Stop position validation failed - adjusting position");
                        stopping_sequence_initiated_ = false;
                        stop_confirmation_counter_ = 0;
                    }
                }
            } else {
                stopping_sequence_initiated_ = false;
                stop_confirmation_counter_ = 0;

                double target_yaw = atan2(dy, dx);
                double yaw_error = target_yaw - current_yaw_;

                while (yaw_error > M_PI) yaw_error -= 2*M_PI;
                while (yaw_error < -M_PI) yaw_error += 2*M_PI;

                if (fabs(yaw_error) > ANGULAR_THRESHOLD) {
                    cmd_vel_.linear.x = 0.0;
                    cmd_vel_.angular.z = limit_velocity(KP_ANGULAR * yaw_error, MAX_ANGULAR_SPEED);
                } else {
                    double speed_factor = std::min(1.0, distance / 2.0);
                    cmd_vel_.linear.x = limit_velocity(KP_LINEAR * speed_factor, MAX_LINEAR_SPEED, MIN_LINEAR_SPEED);
                    cmd_vel_.angular.z = limit_velocity(KP_ANGULAR * yaw_error, MAX_ANGULAR_SPEED);
                }

                cmd_vel_pub_->publish(cmd_vel_);
            }

            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }

        return false;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoverNavigation>();

    double target_x, target_y;
    std::cout << "Enter target X coordinate: ";
    std::cin >> target_x;
    std::cout << "Enter target Y coordinate: ";
    std::cin >> target_y;

    node->set_target(target_x, target_y);
    node->move_to_target();

    rclcpp::shutdown();
    return 0;
}

