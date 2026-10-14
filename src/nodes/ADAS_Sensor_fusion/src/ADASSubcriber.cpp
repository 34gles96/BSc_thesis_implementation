#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float64.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "ADAS.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class AdasSensorSubscriber : public rclcpp::Node
{
public:
    using GnssMsg = sensor_msgs::msg::NavSatFix;

    using GnssSyncPolicy =
        message_filters::sync_policies::ApproximateTime<
            GnssMsg,
            GnssMsg>;

    AdasSensorSubscriber()
        : Node("adas_sensor_subscriber")
    {

        // =========================================================
        // initialize EKF state
        // =========================================================
        ekf_state_.p.setZero();

        ekf_state_.q << 1.0,
            0.0,
            0.0,
            0.0;

        ekf_state_.bg.setZero();

        ekf_state_.P =
            Eigen::Matrix<double, 10, 10>::Identity();

        // =========================================================
        // IMU SUBSCRIBER
        // =========================================================

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data",
            rclcpp::SensorDataQoS(),
            std::bind(
                &AdasSensorSubscriber::imuCallback,
                this,
                _1));

        // =========================================================
        // STEERING ANGLE SUBSCRIBER
        //
        // Here I use std_msgs/Float64.
        // Replace this with your own custom vehicle message if needed.
        // =========================================================

        steering_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/vehicle/steering_angle",
            10,
            std::bind(
                &AdasSensorSubscriber::steeringCallback,
                this,
                _1));

        // =========================================================
        // GNSS SUBSCRIBERS
        //
        // These two are synchronized because the dual-GNSS baseline
        // should be computed from measurements belonging to nearly
        // the same measurement epoch.
        // =========================================================

        gnss1_sub_.subscribe(
            this,
            "/gnss1/fix",
            rmw_qos_profile_sensor_data);

        gnss2_sub_.subscribe(
            this,
            "/gnss2/fix",
            rmw_qos_profile_sensor_data);

        // =========================================================
        // APPROXIMATE TIME SYNCHRONIZER
        // =========================================================

        gnss_sync_ =
            std::make_shared<
                message_filters::Synchronizer<GnssSyncPolicy>>(
                GnssSyncPolicy(10),
                gnss1_sub_,
                gnss2_sub_);

        gnss_sync_->registerCallback(
            std::bind(
                &AdasSensorSubscriber::gnssCallback,
                this,
                _1,
                _2));

        RCLCPP_INFO(
            this->get_logger(),
            "ADAS sensor subscriber started.");
    }

private:
    // =============================================================
    // IMU CALLBACK
    // =============================================================

    void imuCallback(
        const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // processing

        imu.acceleration << msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z;

        imu.angular_velocity << msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z;

        imu.stamp = rclcpp::Time(msg->header.stamp);

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
        }

        // =========================================================
        // EKF PREDICTION
        // =========================================================
        // TODO
        // =========================================================

        /*
        RCLCPP_DEBUG(
            this->get_logger(),
            "IMU prediction: wz = %.3f rad/s, steering = %.3f rad",
            wz,
            steering_angle);
            */
    }

    // =============================================================
    // STEERING CALLBACK
    // =============================================================

    void steeringCallback(
        const std_msgs::msg::Float64::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        latest_steering_angle_ = msg->data;
        steering_received_ = true;
    }

    // =============================================================
    // SYNCHRONIZED DUAL-GNSS CALLBACK
    // =============================================================

    void gnssCallback(
        const GnssMsg::ConstSharedPtr &gnss1,
        const GnssMsg::ConstSharedPtr &gnss2)
    {
        // GNSS receiver 1-2
        DualGnssData_.isUsed = false;

        // TODO : sensor proccesing intro struct aligned with matlab coder lib

        RCLCPP_INFO(
            this->get_logger(),
            "Synchronized GNSS pair received");
    }

private:
    EkfState ekf_state_;
    DualGnssData DualGnssData_;
    ImuData imu;
    SteeringData SteeringData_;
    // =============================================================
    // NORMAL ROS SUBSCRIBERS
    // =============================================================

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr
        imu_sub_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
        steering_sub_;

    // =============================================================
    // MESSAGE FILTER GNSS SUBSCRIBERS
    // =============================================================

    message_filters::Subscriber<GnssMsg>
        gnss1_sub_;

    message_filters::Subscriber<GnssMsg>
        gnss2_sub_;

    std::shared_ptr<
        message_filters::Synchronizer<GnssSyncPolicy>>
        gnss_sync_;

    // =============================================================
    // LATEST VEHICLE DATA
    // =============================================================

    std::mutex data_mutex_;

    double latest_steering_angle_{0.0};

    bool steering_received_{false};
};

// ================================================================
// MAIN
// ================================================================

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<AdasSensorSubscriber>();

    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(),
        4);

    executor.add_node(node);

    executor.spin();

    rclcpp::shutdown();

    return 0;
}
// TODO sensor std:_mutex using!!