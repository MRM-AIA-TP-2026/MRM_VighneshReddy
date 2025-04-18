#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/stitching.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>
#include <thread>

class PanoramaNode : public rclcpp::Node {
public:
    PanoramaNode() : Node("panorama_node"), image_count_(0) {
        camera_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/d430/color/image_raw", 10, std::bind(&PanoramaNode::imageCallback, this, std::placeholders::_1));

        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&PanoramaNode::imuCallback, this, std::placeholders::_1));

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "Panorama node initialized.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    std::vector<cv::Mat> images_;
    float current_orientation_ = 0.0;
    size_t image_count_;

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (image_count_ >= 3) {
            return;
        }

        try {
            cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
            images_.push_back(image);
            image_count_++;
            RCLCPP_INFO(this->get_logger(), "Captured image %zu", image_count_);

            if (image_count_ < 3) {
                rotateRover(8); // Turn by 15 degrees
            } else {
                stitchPanorama();
            }
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge error: %s", e.what());
        }
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        double roll, pitch, yaw;
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        current_orientation_ = static_cast<float>(yaw * 180.0 / M_PI);
    }

    void rotateRover(float angle_deg) {
        float angular_speed = 0.2; 
        float angle_rad = angle_deg * M_PI / 180.0;
        float duration = angle_rad / angular_speed;

        geometry_msgs::msg::Twist twist;
        twist.angular.z = angular_speed;

        auto start_time = this->now();
        while ((this->now() - start_time).seconds() < duration) {
            cmd_vel_publisher_->publish(twist);
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }

        twist.angular.z = 0.0;
        cmd_vel_publisher_->publish(twist);
        rclcpp::sleep_for(std::chrono::milliseconds(500));
    }

    void stitchPanorama() {
        cv::Ptr<cv::Stitcher> stitcher = cv::Stitcher::create(cv::Stitcher::PANORAMA);

        cv::Mat pano;
        cv::Stitcher::Status status = stitcher->stitch(images_, pano);

        if (status != cv::Stitcher::OK) {
            RCLCPP_ERROR(this->get_logger(), "Error stitching images, status code: %d", static_cast<int>(status));
            return;
        }

        addCompass(pano);

        cv::imshow("Final Panorama", pano);
        cv::imwrite("panorama_with_compass.jpg", pano);
        cv::waitKey(0);
        cv::destroyAllWindows();
    }

    void addCompass(cv::Mat &image) {
        int radius = 50;
        cv::Point center(image.cols - radius - 20, radius + 20);

        cv::circle(image, center, radius, cv::Scalar(255, 255, 255), -1);
        cv::circle(image, center, radius - 5, cv::Scalar(0, 0, 0), -1);

        cv::putText(image, "N", center + cv::Point(-10, -radius + 20), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
        cv::putText(image, "S", center + cv::Point(-10, radius - 5), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
        cv::putText(image, "E", center + cv::Point(radius - 15, 5), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
        cv::putText(image, "W", center + cv::Point(-radius + 5, 5), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

        float angle_rad = current_orientation_ * (M_PI / 180.0);
        cv::Point needle_end(
            static_cast<int>(center.x + radius * 0.8 * std::sin(angle_rad)),
            static_cast<int>(center.y - radius * 0.8 * std::cos(angle_rad)));

        cv::line(image, center, needle_end, cv::Scalar(0, 0, 255), 2);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PanoramaNode>());
    rclcpp::shutdown();
    return 0;
}

