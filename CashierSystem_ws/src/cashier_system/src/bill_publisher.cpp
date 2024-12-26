#include "rclcpp/rclcpp.hpp"
#include "cashier_system/msg/bill.hpp"
#include <iostream>

class BillPublisher : public rclcpp::Node {
public:
    BillPublisher() : Node("bill_publisher") {
        publisher_ = this->create_publisher<cashier_system::msg::Bill>("bill_topic", 10);
        timer_ = this->create_wall_timer(
            1000ms, std::bind(&BillPublisher::publish_bill, this)
        );
    }

private:
    void publish_bill() {
        int quantity;
        float price;

        std::cout << "Enter quantity: ";
        std::cin >> quantity;
        std::cout << "Enter price: ";
        std::cin >> price;

        auto message = cashier_system::msg::Bill();
        message.bill_number = bill_number_++;
        message.timestamp = this->now();
        message.quantity = quantity;
        message.price = price;
        message.total = quantity * price;

        RCLCPP_INFO(this->get_logger(), "Publishing Bill %d: Total = %.2f", message.bill_number, message.total);
        publisher_->publish(message);
    }

    rclcpp::Publisher<cashier_system::msg::Bill>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int bill_number_ = 1;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BillPublisher>());
    rclcpp::shutdown();
    return 0;
}

