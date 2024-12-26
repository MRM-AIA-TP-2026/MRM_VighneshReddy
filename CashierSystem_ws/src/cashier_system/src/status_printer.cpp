#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include "cashier_system/msg/bill.hpp"


class StatusPrinter : public rclcpp::Node {
public:
    StatusPrinter() : Node("status_printer") {
        this->declare_parameter<int>("inventory", 100);
        this->declare_parameter<float>("income", 0.0);

        timer_ = this->create_wall_timer(
            5000ms, std::bind(&StatusPrinter::print_status, this)
        );
    }

private:
    void print_status() {
        int inventory = this->get_parameter("inventory").get_value<int>();
        float income = this->get_parameter("income").get_value<float>();

        std::string input;
        std::cout << "Type 'status' to view current inventory: ";
        std::cin >> input;

        if (input == "status") {
            RCLCPP_INFO(this->get_logger(), "Current Inventory: %d, Current Income: %.2f", inventory, income);
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StatusPrinter>());
    rclcpp::shutdown();
    return 0;
}

