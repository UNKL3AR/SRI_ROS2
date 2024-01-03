#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include "sri_interface/msg/six_axis_fts.hpp"
#include "omni_msgs/msg/omni_feedback.hpp"

using std::placeholders::_1;

class FeedbackConverterNode : public rclcpp::Node {
public:
    FeedbackConverterNode() : Node("feedback_converter_node") {
        subscription_ = this->create_subscription<sri_interface::msg::SixAxisFTS>(
            "/M3733C_Force_Sensor", 10,
            std::bind(&FeedbackConverterNode::raw_data_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<omni_msgs::msg::OmniFeedback>("/phantom/force_feedback", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(timerIntervalMs),
                                         std::bind(&FeedbackConverterNode::publish_kalman_filtered_force, this));
    }

private:
    void raw_data_callback(const sri_interface::msg::SixAxisFTS::SharedPtr data) {
    // 添加新的力传感器数据到历史数据向量中
    x_force_history.push_back(data->x_force);
    y_force_history.push_back(data->y_force);
    z_force_history.push_back(data->z_force);

    // 限制历史数据向量的长度，只保留最近的N个数据
    const int max_history_size = 5;
    if (x_force_history.size() > max_history_size) {
        x_force_history.erase(x_force_history.begin());
        y_force_history.erase(y_force_history.begin());
        z_force_history.erase(z_force_history.begin());
    }

    // 计算历史数据向量的平均值作为滤波后的力值
    filtered_x_force = std::accumulate(x_force_history.begin(), x_force_history.end(), 0.0) / x_force_history.size();
    filtered_y_force = std::accumulate(y_force_history.begin(), y_force_history.end(), 0.0) / y_force_history.size();
    filtered_z_force = std::accumulate(z_force_history.begin(), z_force_history.end(), 0.0) / z_force_history.size();

    if (!initialized) {
        x_init = filtered_x_force;
        y_init = filtered_y_force;
        z_init = filtered_z_force;
        initialized = true;
    }

    // 更新force_msg消息的值
    force_msg.force.x = -(filtered_x_force-x_init);
    force_msg.force.y = (filtered_y_force-y_init);
    force_msg.force.z = -(filtered_z_force-z_init);
}

    void publish_kalman_filtered_force() {
        // Compute force differences using Kalman-filtered values
        // Publish the force feedback message
        if (publisher_ && publisher_->get_subscription_count() > 0) {
            publisher_->publish(force_msg);
        } else {
            RCLCPP_ERROR(this->get_logger(), "No subscribers for force feedback topic");
        }

        // Log count for debugging
        RCLCPP_INFO(this->get_logger(), "Count: %d", count++);
    }

    static constexpr int timerIntervalMs = 10; // Define timer interval

    std::vector<double> x_force_history;
    std::vector<double> y_force_history;
    std::vector<double> z_force_history;

    double filtered_x_force = 0.0;
    double filtered_y_force = 0.0;
    double filtered_z_force = 0.0;

    double x_init = 0;
    double y_init = 0;
    double z_init = 0;
    bool initialized = false;
    omni_msgs::msg::OmniFeedback force_msg;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sri_interface::msg::SixAxisFTS>::SharedPtr subscription_;
    rclcpp::Publisher<omni_msgs::msg::OmniFeedback>::SharedPtr publisher_;
    int count = 0;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<FeedbackConverterNode>();

    if (node) {
        rclcpp::spin(node);
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Failed to create node");
    }

    rclcpp::shutdown();
    return 0;
}


