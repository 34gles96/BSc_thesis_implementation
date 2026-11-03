
// TODO: retunred value of EKF_func
// TODO : std::ofstream object needs to write msg->acc values into log file

/*
ROS2 msg: due to sensor data trasnitting introduced a trasnmission latency that follows noramlly distributions: timestemp not constant

EKF predictions say:
x_{k+1} = f(x_k, u_k) + w_k , where w_k ~ N(0,Q);     Q describes imperfect bicycle-model assumptions


*/
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <algorithm>
#include <fstream>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float64.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include <nav_msgs/msg/odometry.hpp>

#include "ADAS.hpp"
#include "EKF_interface.hpp"
#include "loadADAScfg.hpp"
#include "loadEKFconfig.hpp"
#include "myLog.hpp"
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
        : Node(
              "adas_sensor_subscriber",
              rclcpp::NodeOptions().append_parameter_override(
                  "use_sim_time", true))
    {
        // ====================  PUBLISHER  ====================
        // initialization of publisher for tranmtteing estimated pose ofr subscribers: camNode(trajectory),
        ekf_pose_pub_ =
            this->create_publisher<geometry_msgs::msg::PoseStamped>(
                "/adas/ekf/pose", rclcpp::SensorDataQoS());

        // initialization of publisher for tranmtteing estimated pose ofr subscribers : MPC
        ekf_odom_pub_ =
            this->create_publisher<nav_msgs::msg::Odometry>(
                "/adas/ekf/odometry",
                rclcpp::SensorDataQoS());

        // ==================== MATLAB EKF ====================

        EKF_func_initialize();

        imu_mounting_ = loadImuMounting(
            "/ADC_27/configData/cam_config/ADAScfg.yaml");

        vehicle_params_ =
            loadVehicleParams("/ADC_27/configData/cam_config/ADAScfg.yaml");

        // ==================== LOG FILE init ====================
        myLOG::initializeGnssLog();
        myLOG::initializeEkfLog();
        ekfReport::initializeReport();

        // =========================================================
        // initialize EKF state
        // =========================================================
        ekf_config_ =
            loadEkfConfig(
                "/home/base/ADC_27/configData/cam_config/EKFcfg.yaml");

        // Initial gyro bias mean
        generated_ekf_state_.bg[0] = 0.0;
        generated_ekf_state_.bg[1] = 0.0;
        generated_ekf_state_.bg[2] = 0.0;

        // Initial gyro-bias covariance
        for (int i = 0; i < 3; ++i)
        {
            const double sigma =
                ekf_config_.gyro_bias_std[i];

            const int state_index = 7 + i;

            generated_ekf_state_.P[state_index + 10 * state_index] = sigma * sigma; // P_0[8:10, 8:10]
            // The other parts of P_0 remain zero until your first valid dual-GNSS initialization inside MATLAB sets:
            // state.P(1:3,1:3) = P_position;
            // state.P(4:7,4:7) = P_quaternion;
        }

        std::fill_n(
            generated_ekf_state_.Q,
            100,
            0.0);

        /*
        state index 0 -> E -> Q[0]
        state index 1 -> N -> Q[11]
        state index 2 -> U -> Q[22]
        ...
        */
        const double sigmaE =
            ekf_config_.bicycle_position_std[0];

        const double sigmaN =
            ekf_config_.bicycle_position_std[1];

        generated_ekf_state_.Q[0] =
            sigmaE * sigmaE;

        generated_ekf_state_.Q[11] =
            sigmaN * sigmaN;

        // ====================================================

        imu_group_ =
            this->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);

        steering_group_ =
            this->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);

        gnss_group_ =
            this->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);

        velocity_group_ =
            this->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions imu_options;
        imu_options.callback_group = imu_group_;

        rclcpp::SubscriptionOptions steering_options;
        steering_options.callback_group = steering_group_;

        rclcpp::SubscriptionOptions gnss_options;
        gnss_options.callback_group = gnss_group_;

        rclcpp::SubscriptionOptions velocity_options;
        velocity_options.callback_group = velocity_group_;

        // =========================================================
        // IMU SUBSCRIBER
        // =========================================================

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/vehicles/beamng_bridge/sensors/imu_fast",
            rclcpp::SensorDataQoS(),
            std::bind(
                &AdasSensorSubscriber::imuCallback,
                this,
                _1),
            imu_options);

        // =========================================================
        // STEERING ANGLE SUBSCRIBER
        //
        // Here I use std_msgs/Float64.
        //
        // =========================================================

        steering_sub_ =
            this->create_subscription<std_msgs::msg::Float64>(
                "/vehicle/steering_angle",
                10,
                std::bind(
                    &AdasSensorSubscriber::steeringCallback,
                    this,
                    _1),
                steering_options);

        // =========================================================
        // wheel speed SUBSCRIBER
        //
        // Here I use std_msgs/Float64.
        //
        // =========================================================

        velocity_sub_ =
            this->create_subscription<std_msgs::msg::Float64>(
                "/vehicle/wheelspeed",
                10,
                std::bind(
                    &AdasSensorSubscriber::velocityCallback,
                    this,
                    _1),
                velocity_options);

        // =========================================================
        // GNSS SUBSCRIBERS
        //
        // These two are synchronized because the dual-GNSS baseline
        // should be computed from measurements belonging to nearly
        // the same measurement epoch.
        // =========================================================

        // GNSS 1 = P1 = RIGHT antenna
        gnss1_sub_.subscribe(
            this,
            "/vehicles/beamng_bridge/sensors/gps_base_front",
            rmw_qos_profile_sensor_data,
            gnss_options);

        // GNSS 2 = P2 = LEFT antenna
        gnss2_sub_.subscribe(
            this,
            "/vehicles/beamng_bridge/sensors/gps_rover_front",
            rmw_qos_profile_sensor_data,
            gnss_options);

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

    ~AdasSensorSubscriber()
    {
        EKF_func_terminate();

        myLOG::closeGnssLog();
        myLOG::closeEkfLog();

        ekfReport::closeReport(); // stop ros2 bag via sript
    }

private:
    // ==================== MATLAB EKF ====================
    GeneratedImuMounting imu_mounting_{};
    VehicleParams vehicle_params_{};
    EkfConfig ekf_config_{};
    // =============================================================
    // IMU CALLBACK
    // =============================================================

    void imuCallback(
        const sensor_msgs::msg::Imu::SharedPtr msg)
    {

        GeneratedGnssData generatedGnss{};
        SteeringData steeringTmp;
        VelocityData velocityTmp;
        bool new_gnss = false;
        // =========================================================
        //  Local copies of shared sensor data
        // =========================================================
        {
            std::lock_guard<std::mutex> lock(sensor_mutex_);

            if (!steering_received_ || !velocity_received_) // needed at least one steering and wheel velocity recevied and workigng wit the latest steering always
            {
                return;
            }
            else
            {
                steeringTmp = SteeringData_;
                velocityTmp = VelocityData_;
            }

            new_gnss = gnss_received_ && !DualGnssData_.isUsed;
            if (new_gnss)
            {
                /*
                lla[0] = latitude
                lla[1] = longitude
                lla[2] = altitude
                */

                generatedGnss.lla1[0] = DualGnssData_.lla1.x();
                generatedGnss.lla1[1] = DualGnssData_.lla1.y();
                generatedGnss.lla1[2] = DualGnssData_.lla1.z();

                generatedGnss.lla2[0] = DualGnssData_.lla2.x();
                generatedGnss.lla2[1] = DualGnssData_.lla2.y();
                generatedGnss.lla2[2] = DualGnssData_.lla2.z();

                generatedGnss.hAcc1 = DualGnssData_.hAcc1;
                generatedGnss.vAcc1 = DualGnssData_.vAcc1;

                generatedGnss.hAcc2 = DualGnssData_.hAcc2;
                generatedGnss.vAcc2 = DualGnssData_.vAcc2;

                generatedGnss.timestamp = DualGnssData_.timestamp_ms;

                // for safety reasion: frame transformation happens insde EKF
                const double nan =
                    std::numeric_limits<double>::quiet_NaN();

                for (int i = 0; i < 3; ++i)
                {
                    generatedGnss.position1_enu[i] = nan; // it is computed in EKF, so it s a placholder in cpp app
                    generatedGnss.position2_enu[i] = nan;
                    generatedGnss.position_enu[i] = nan;
                    generatedGnss.baseline_enu[i] = nan;
                    generatedGnss.baseline_body[0] = 0.0; // constructed from scnario.json , and trasnfromed into ros2 body frame convention
                    generatedGnss.baseline_body[1] = 0.8;
                    generatedGnss.baseline_body[2] = 0.0;
                }

                generatedGnss.valid = true;
                DualGnssData_.isUsed = true;
            }
        }

        // =========================================================
        //  Prepare IMU data
        // =========================================================
        // =========== MATLAB =================
        GeneratedImuData generatedImu{};

        // ============================================================
        // IMU mounting (for transformaing sensor frame into boudy frame)
        // ============================================================
        generatedImu.mounting = imu_mounting_;
        // ============================================================
        // IMU VALIDITY + TIMESTAMP
        // ============================================================

        generatedImu.valid = true;

        generatedImu.timestamp =
            static_cast<double>(
                rclcpp::Time(msg->header.stamp).nanoseconds()) /
            1e6; // milliseconds

        // ============================================================
        // GYROSCOPE VECTOR
        //
        // ROS measurement is in IMU SENSOR frame.
        // MATLAB EKF_func performs SENSOR -> BODY transformation.
        // ============================================================

        generatedImu.gyro[0] =
            msg->angular_velocity.x;

        generatedImu.gyro[1] =
            msg->angular_velocity.y;

        generatedImu.gyro[2] =
            msg->angular_velocity.z;

        // ============================================================
        // GYROSCOPE COVARIANCE
        //
        // ROS:
        //      row-major indexing
        //      index = row*3 + col
        //
        // MATLAB Coder:
        //      column-major indexing
        //      index = row + 3*col
        // ============================================================

        for (int row = 0; row < 3; ++row)
        {
            for (int col = 0; col < 3; ++col)
            {
                generatedImu.gyro_covariance[row + 3 * col] =
                    msg->angular_velocity_covariance[row * 3 + col];
            }
        }

        // ============================================================
        // ACCELEROMETER VECTOR
        //
        // ROS measurement is in IMU SENSOR frame.
        // MATLAB EKF_func performs SENSOR -> BODY transformation.
        // ============================================================

        generatedImu.accel[0] =
            msg->linear_acceleration.x;

        generatedImu.accel[1] =
            msg->linear_acceleration.y;

        generatedImu.accel[2] =
            msg->linear_acceleration.z;

        // ============================================================
        // ACCELEROMETER COVARIANCE
        //
        // ROS covariance belongs to the SENSOR-frame acceleration.
        //
        // Do NOT rotate it here.
        // MATLAB EKF_func will perform:
        //
        // R_acc_body =
        //     R_body_sensor *
        //     R_acc_sensor *
        //     R_body_sensor.'
        // ============================================================

        for (int row = 0; row < 3; ++row)
        {
            for (int col = 0; col < 3; ++col)
            {
                generatedImu.accel_covariance[row + 3 * col] =
                    msg->linear_acceleration_covariance[row * 3 + col];
            }
        }

        // =====================================

        //  ======================= invoking EKF ==================================
        GeneratedVehicleData generatedVehicle{};

        generatedVehicle.velocity =
            velocityTmp.velocity;

        generatedVehicle.steering_angle =
            steeringTmp.angle;

        generatedVehicle.wheelbase =
            vehicle_params_.wheelbase;

        // report into log file
        ekfReport::reportEKF_in(
            generatedImu,
            generatedGnss,
            generatedVehicle);

        const double imu_sim_sec =
            rclcpp::Time(msg->header.stamp).seconds();

        const double ros_sim_now_sec =
            this->get_clock()->now().seconds();

        RCLCPP_INFO(
            this->get_logger(),
            "IMU stamp: %.3f s | ROS sim now: %.3f s",
            imu_sim_sec,
            ros_sim_now_sec);

        EKF_func(
            &generatedImu,
            &generatedGnss,
            &generatedVehicle,
            &generated_ekf_state_);

        // loging estimated pos.
        myLOG::logEkfState(
            generatedImu.timestamp,
            generated_ekf_state_);

        //  ================== TRANMITTING ESTIMATED POSE VIA PUBLISHER =======================================

        geometry_msgs::msg::PoseStamped pose_msg;

        // EKF state corresponds to this IMU update
        pose_msg.header.stamp = msg->header.stamp;

        // Coordinate frame: East-North-Up
        pose_msg.header.frame_id = "enu";

        // Estimated CoG position in ENU
        pose_msg.pose.position.x =
            generated_ekf_state_.PosENU_cog[0];

        pose_msg.pose.position.y =
            generated_ekf_state_.PosENU_cog[1];

        pose_msg.pose.position.z =
            generated_ekf_state_.PosENU_cog[2];

        // MATLAB quaternion: [qw, qx, qy, qz]
        // ROS quaternion:    [qx, qy, qz, qw]

        pose_msg.pose.orientation.x =
            generated_ekf_state_.q[1];

        pose_msg.pose.orientation.y =
            generated_ekf_state_.q[2];

        pose_msg.pose.orientation.z =
            generated_ekf_state_.q[3];

        pose_msg.pose.orientation.w =
            generated_ekf_state_.q[0];

        // Publish pose to camera/MPC subscribers
        // Publishing only after the EKF has successfully initialized its ENU reference and valid pose.
        ekf_pose_pub_->publish(pose_msg);

        // ==========================================================
        // PUBLISH EKF POSE AND VEHICLE VELOCITY FOR MPC
        // ==========================================================

        nav_msgs::msg::Odometry odom_msg{};

        // Same timestamp and ENU reference frame as the EKF pose
        odom_msg.header = pose_msg.header;

        // Velocity is expressed in the vehicle body frame
        odom_msg.child_frame_id = "base_link";

        // Copy EKF-estimated CoG position and orientation
        odom_msg.pose.pose = pose_msg.pose;

        // Longitudinal vehicle speed from wheel-speed measurement
        // Assumption: positive body-X direction is forward
        odom_msg.twist.twist.linear.x = velocityTmp.velocity;

        // No lateral or vertical velocity estimates available here
        // Do not populate these with invented measurements.

        // Transmit to MPC node
        ekf_odom_pub_->publish(odom_msg);
    }

    // =============================================================
    // STEERING CALLBACK
    // =============================================================

    void steeringCallback(
        const std_msgs::msg::Float64::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(sensor_mutex_);

        SteeringData_.angle = msg->data;
        steering_received_ = true;
    }

    // =============================================================
    // WHEEL VELOCITY CALLBACK
    // =============================================================
    void velocityCallback(
        const std_msgs::msg::Float64::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(sensor_mutex_);

        VelocityData_.velocity = msg->data;

        velocity_received_ = true;
    }

    // =============================================================
    // SYNCHRONIZED DUAL-GNSS CALLBACK
    // =============================================================

    void gnssCallback(
        const GnssMsg::ConstSharedPtr &gnss1,
        const GnssMsg::ConstSharedPtr &gnss2)
    {

        // log file DEBUG
        myLOG::logGnssLla(
            gnss1,
            gnss2);

        DualGnssData gnssTmp{};

        const double t1_ms =
            static_cast<double>(
                rclcpp::Time(gnss1->header.stamp).nanoseconds()) /
            1e6;

        const double t2_ms =
            static_cast<double>(
                rclcpp::Time(gnss2->header.stamp).nanoseconds()) /
            1e6;

        gnssTmp.timestamp_ms = 0.5 * (t1_ms + t2_ms);

        gnssTmp.lla1 << gnss1->latitude,
            gnss1->longitude,
            gnss1->altitude;

        gnssTmp.lla2 << gnss2->latitude,
            gnss2->longitude,
            gnss2->altitude;

        //  seting these according to your GNSS accuracy model (normally ditributed noise added in python ros-beaNG intergration but cov matrexes r tranmitetted bcxs or ROS architecture..)
        gnssTmp.hAcc1 =
            std::sqrt(gnss1->position_covariance[0]);

        gnssTmp.vAcc1 =
            std::sqrt(gnss1->position_covariance[8]);

        gnssTmp.hAcc2 =
            std::sqrt(gnss2->position_covariance[0]);

        gnssTmp.vAcc2 =
            std::sqrt(gnss2->position_covariance[8]);

        {
            std::lock_guard<std::mutex> lock(sensor_mutex_);

            DualGnssData_ = gnssTmp; // shared resource is DualGnssData with imu callback (avoiding expensive isntruvtions in lock)
            DualGnssData_.isUsed = false;

            gnss_received_ = true; // first traansitted gnss is avaible to init ENU frame in EKF func
        }
    }

    EkfState ekf_state_;
    GeneratedEkfState generated_ekf_state_{};
    DualGnssData DualGnssData_;
    SteeringData SteeringData_;

    // thread groups : enabling to parall executions for callback, shared mem is protected by using std::mutex
    rclcpp::CallbackGroup::SharedPtr imu_group_;
    rclcpp::CallbackGroup::SharedPtr steering_group_;
    rclcpp::CallbackGroup::SharedPtr gnss_group_;
    rclcpp::CallbackGroup::SharedPtr velocity_group_;
    // =============================================================
    // NORMAL ROS SUBSCRIBERS
    // =============================================================

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr
        imu_sub_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
        steering_sub_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
        velocity_sub_;

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

    std::mutex sensor_mutex_;
    bool steering_received_{false};
    bool gnss_received_{false};
    VelocityData VelocityData_;
    bool velocity_received_{false};

    // =============================================================
    // PUBLISHERS
    // =============================================================

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        ekf_pose_pub_; // for CAM node

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
        ekf_odom_pub_; // for MPC node
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