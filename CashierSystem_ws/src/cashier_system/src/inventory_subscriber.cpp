#include "rclcpp/rclcpp.hpp"
#include "cashier_system/msg/bill.hpp"

class InventorySubscriber : public rclcpp::Node {
public:
    InventorySubscriber() : Node("inventory_subscriber") {
        this->declare_parameter<int>("inventory", 100);
        this->declare_parameter<float>("income", 0.0);

        subscription_ = this->create_subscription<cashier_system::msg::Bill>(
            "bill_topic", 10, std::bind(&InventorySubscriber::handle_bill, this, std::placeholders::_1)
        );
    }

private:
    void handle_bill(const cashier_system::msg::Bill::SharedPtr msg) {
        int inventory = this->get_parameter("inventory").get_value<int>();
        float income = this->get_parameter("income").get_value<float>();

        inventory -= msg->quantity;
        income += msg->total;

        this->set_parameter(rclcpp::Parameter("inventory", inventory));
        this->set_parameter(rclcpp::Parameter("income", income));

        RCLCPP_INFO(this->get_logger(), "Received Bill: %d, Quantity: %d, Total: %.2f", msg->bill_number, msg->quantity, msg->total);
        RCLCPP_INFO(this->get_logger(), "Updated Inventory: %d, Updated Income: %.2f", inventory, income);
    }

    rclcpp::Subscription<cashier_system::msg::Bill>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InventorySubscriber>());
    rclcpp::shutdown();
    return 0;
}

