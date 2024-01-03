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

// class KalmanFilter {
// public:
//     // Constructor with default noise variances
//     KalmanFilter(double processNoise = 0.001, double measurementNoise = 0.1)
//         : Q(processNoise), R(measurementNoise),
//           P_k_k1(1), Kalman_gain(0), P_k1_k1(1), x_k_k1(0),
//           Z_k(0), kalman_adc_old(0) {}

//     // Kalman filter method
//     double kalman(double ADC_Value) {
//         Z_k = ADC_Value;

//         double x_k_k1_updated;
//         if (std::abs(kalman_adc_old - ADC_Value) >= 60) {
//             x_k_k1_updated = ADC_Value * 0.382 + kalman_adc_old * 0.618;
//         } else {
//             x_k_k1_updated = kalman_adc_old;
//         }

//         // Update Kalman filter variables only once
//         x_k_k1 = x_k_k1_updated;
//         P_k_k1 = P_k1_k1 + Q;
//         Kalman_gain = P_k_k1 / (P_k_k1 + R);

//         double kalman_adc = x_k_k1 + Kalman_gain * (Z_k - kalman_adc_old);
//         P_k1_k1 = (1 - Kalman_gain) * P_k_k1;
//         P_k_k1 = P_k1_k1;
//         kalman_adc_old = kalman_adc;

//         return kalman_adc;
//     }

// private:
//     double Q;               // Process noise variance
//     double R;               // Measurement noise variance
//     double P_k_k1;          // Predicted state estimation error covariance
//     double Kalman_gain;              // Kalman gain
//     double P_k1_k1;         // Updated state estimation error covariance
//     double x_k_k1;          // Predicted state estimate
//     double Z_k;             // Latest measurement
//     double kalman_adc_old;  // Old Kalman filtered value

// };

// class FeedbackConverterNode : public rclcpp::Node {
// public:
//     FeedbackConverterNode() : Node("feedback_converter_node") {
//         subscription_ = this->create_subscription<sri_interface::msg::SixAxisFTS>(
//             "/M3733C_Force_Sensor", 10,
//             std::bind(&FeedbackConverterNode::raw_data_callback, this, std::placeholders::_1));
//         publisher_ = this->create_publisher<omni_msgs::msg::OmniFeedback>("/phantom/force_feedback", 10);
//         timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&FeedbackConverterNode::publish_kalman_filtered_force, this));
//     }

// private:
//     void raw_data_callback(const sri_interface::msg::SixAxisFTS::SharedPtr data) {
//         // Apply Kalman filter to incoming force values
//         kalman_x_force = kf.kalman(data->x_force);
//         kalman_y_force = kf.kalman(data->y_force);
//         kalman_z_force = kf.kalman(data->z_force);
//     }

//     void publish_kalman_filtered_force() {
//         omni_msgs::msg::OmniFeedback force_msg;

//         // Compute force differences using Kalman-filtered values
//         force_msg.force.x = kalman_x_force - x_last_force;
//         force_msg.force.y = kalman_y_force - y_last_force;
//         force_msg.force.z = kalman_z_force - z_last_force;

//         // Update last Kalman-filtered force values
//         x_last_force = kalman_x_force;
//         y_last_force = kalman_y_force;
//         z_last_force = kalman_z_force;

//         // Publish the force feedback message
//         publisher_->publish(force_msg);

//         // Debugging: Output count
//         std::cout << "Count: " << count++ << std::endl;
//     }

//     double kalman_x_force = 0;
//     double kalman_y_force = 0;
//     double kalman_z_force = 0;

//     double x_last_force = 0;
//     double y_last_force = 0;
//     double z_last_force = 0;
//     KalmanFilter kf; // Assuming KalmanFilter class is properly defined
//     rclcpp::TimerBase::SharedPtr timer_;
//     rclcpp::Subscription<sri_interface::msg::SixAxisFTS>::SharedPtr subscription_;
//     rclcpp::Publisher<omni_msgs::msg::OmniFeedback>::SharedPtr publisher_;
//     int count = 0;
// };

// int main(int argc, char *argv[]) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<FeedbackConverterNode>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }

// ... Existing includes and definitions

class KalmanFilter {
public:
    KalmanFilter(double processNoise = 0.001, double measurementNoise = 0.1)
        : Q(processNoise), R(measurementNoise), P_k_k1(1), Kalman_gain(0),
          P_k1_k1(1), x_k_k1(0), Z_k(0), kalman_adc_old(0) {}

    double kalman(double ADC_Value) {
        Z_k = ADC_Value;
        double x_k_k1_updated;
        if (std::abs(kalman_adc_old - ADC_Value) >= 60) {
            x_k_k1_updated = ADC_Value * 0.382 + kalman_adc_old * 0.618;
        } else {
            x_k_k1_updated = kalman_adc_old;
        }

        x_k_k1 = x_k_k1_updated;
        P_k_k1 = P_k1_k1 + Q;
        Kalman_gain = P_k_k1 / (P_k_k1 + R);

        double kalman_adc = x_k_k1 + Kalman_gain * (Z_k - kalman_adc_old);
        P_k1_k1 = (1 - Kalman_gain) * P_k_k1;
        P_k_k1 = P_k1_k1;
        kalman_adc_old = kalman_adc;

        return kalman_adc;
    }

private:
    double Q;
    double R;
    double P_k_k1;
    double Kalman_gain;
    double P_k1_k1;
    double x_k_k1;
    double Z_k;
    double kalman_adc_old;
};

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
        kalman_x_force = kf.kalman(data->x_force);
        kalman_y_force = kf.kalman(data->y_force);
        kalman_z_force = kf.kalman(data->z_force);
        force_msg.force.x = kalman_x_force - x_last_force;
        force_msg.force.y = kalman_y_force - y_last_force;
        force_msg.force.z = kalman_z_force - z_last_force;
        // Update last Kalman-filtered force values
        x_last_force = kalman_x_force;
        y_last_force = kalman_y_force;
        z_last_force = kalman_z_force;
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

    double kalman_x_force = 0;
    double kalman_y_force = 0;
    double kalman_z_force = 0;

    double x_last_force = 0;
    double y_last_force = 0;
    double z_last_force = 0;

    omni_msgs::msg::OmniFeedback force_msg;
    KalmanFilter kf;
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


