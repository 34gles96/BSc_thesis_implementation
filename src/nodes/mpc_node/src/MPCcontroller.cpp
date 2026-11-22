#include <beamng_msgs/msg/vehicle_control.hpp>
#include "speedPI.hpp"
#include "steering.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <array>
#include <algorithm>
#include <vector>
#include <cstdint>

#include <cmath>

#include <iterator>

#include <mutex>
#include <chrono>
#include <utility>

#include <deque>
#include <cmath>
#include <limits>
#include "mpcLogger.hpp"
#include <functional>

#include <rclcpp/create_timer.hpp>

#include "mpcInterface.hpp"
#include "mpcLogger.hpp"
#include "trajectoryConnector.hpp"
#include "buildRefHorizont.hpp"

struct TrajectoryPointVector
{
    std::vector<TrajectoryPoint> desired_trajectory_;
    double timestamp_ms;
};

class MPCSubscriber : public rclcpp::Node
{
public:
    MPCSubscriber()
        : Node(
              "mpc_subscriber",
              rclcpp::NodeOptions()
                  .append_parameter_override("use_sim_time", true))
    {
        mpcController_initialize();
        mpcLoggger::initializeMPCLog();

        MPC::init_mpc_cfg("/ADC_27/configData/mpc_cfg.yaml", this->mpc_cfg_);

        trajectory_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        mpc_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        ekf_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions options1;
        options1.callback_group = trajectory_group_;

        rclcpp::SubscriptionOptions options2;
        options2.callback_group = ekf_group_;

        trajectory_sub_ =
            this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/adas/desired_trajectory/enu",
                rclcpp::QoS(10),
                std::bind(
                    &MPCSubscriber::trajectoryCallback,
                    this,
                    std::placeholders::_1),
                options1);

        ekf_pose_sub_ =
            this->create_subscription<nav_msgs::msg::Odometry>(
                "/adas/ekf/odometry",
                rclcpp::SensorDataQoS(),
                std::bind(
                    &MPCSubscriber::EKFCallback,
                    this,
                    std::placeholders::_1),
                options2);

        control_pub_ =
            this->create_publisher<beamng_msgs::msg::VehicleControl>(
                "/control", 10);

        mpc_timer_ = rclcpp::create_timer(
            this,
            this->get_clock(),
            rclcpp::Duration::from_seconds(1.0 / 30.0),
            std::bind(&MPCSubscriber::performMPC, this),
            mpc_group_);
    }

    ~MPCSubscriber()
    {
        mpcLoggger::closeMPCLog();
        mpcController_terminate();
    }

private:
    // ==================== subscriber callbacks ======================
    void trajectoryCallback(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
    {
        // =====================================================
        // 1. MEASURE TRAJECTORY AGE AT CALLBACK ENTRY
        // =====================================================

        const double callback_sim_time_ms =
            static_cast<double>(
                this->get_clock()->now().nanoseconds()) /
            1e6;

        const double trajectory_timestamp_ms =
            static_cast<double>(
                rclcpp::Time(msg->header.stamp).nanoseconds()) /
            1e6;

        const double reception_age_ms =
            callback_sim_time_ms - trajectory_timestamp_ms;

        // =====================================================
        // 2. EXTRACT TRAJECTORY POINTS
        // =====================================================

        std::string frame = msg->header.frame_id;

        sensor_msgs::PointCloud2ConstIterator<int32_t> u(*msg, "u");
        sensor_msgs::PointCloud2ConstIterator<int32_t> v(*msg, "v");

        sensor_msgs::PointCloud2ConstIterator<double> x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<double> y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<double> z(*msg, "z");

        const size_t N =
            static_cast<size_t>(msg->width) * msg->height;

        std::vector<TrajectoryPoint> trajectory;
        trajectory.reserve(N);

        for (size_t i = 0; i < N; ++i)
        {
            TrajectoryPoint point;

            point.u = *u;
            point.v = *v;

            point.E = *x;
            point.N = *y;
            point.U = *z;

            trajectory.push_back(point);

            ++u;
            ++v;
            ++x;
            ++y;
            ++z;
        }

        // =====================================================
        // 3. UPDATE SHARED TRAJECTORY
        // =====================================================

        {
            std::lock_guard<std::mutex> lock(trajectory_mutex_);

            TrajectoryPointVector_.desired_trajectory_ =
                std::move(trajectory);

            TrajectoryPointVector_.timestamp_ms =
                trajectory_timestamp_ms;
        }

        // =====================================================
        // 4. LOGGING
        // =====================================================

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Received trajectory: points=%zu | frame=%s | "
            "Age at callback entry=%.3f ms",
            N,
            frame.c_str(),
            reception_age_ms);
    }

    void EKFCallback(
        const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        // =====================================================
        // 1. CURRENT SIMULATOR TIME
        // =====================================================

        const double sim_now_ms =
            static_cast<double>(
                this->get_clock()->now().nanoseconds()) /
            1e6;

        // =====================================================
        // 2. EKF MEASUREMENT TIMESTAMP
        // =====================================================

        const double ekf_timestamp_ms =
            static_cast<double>(
                rclcpp::Time(msg->header.stamp).nanoseconds()) /
            1e6;

        // =====================================================
        // 3. EKF MEASUREMENT AGE
        // =====================================================

        const double ekf_age_ms =
            sim_now_ms - ekf_timestamp_ms;

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "EKF measurement age at callback entry: %.3f ms",
            ekf_age_ms);

        // =====================================================
        // 4. STORE EKF MEASUREMENT
        // =====================================================

        std::lock_guard<std::mutex> lock(ekf_mutex_);

        ekf_buffer_.push_back(*msg);

        constexpr std::size_t ekf_buffer_size = 150;

        if (ekf_buffer_.size() > ekf_buffer_size)
            ekf_buffer_.pop_front();
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
        trajectory_sub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        ekf_pose_sub_;

    // ====================== CONTROL PUBLISH  ===============================================================
    SpeedPI speed_pi_{0.5, 0.1};

    double v_target_{0.0};
    bool speed_target_initialized_{false};

    rclcpp::Publisher<beamng_msgs::msg::VehicleControl>::SharedPtr
        control_pub_;

    // ======================================================================================

    void performMPC()
    {

        constexpr int GENERATED_NP = 10; // needed to tunning in maltab static lib via matlab implementation

        if (mpc_cfg_.Np != GENERATED_NP)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Invalid MPC horizon: configured Np=%.0f, "
                "but generated library requires Np=%d",
                mpc_cfg_.Np,
                GENERATED_NP);

            return;
        }
        // =====================================================
        // 1. COPY DESIRED TRAJECTORY
        // =====================================================

        TrajectoryPointVector trajectory_data;

        {
            std::lock_guard<std::mutex> lock(trajectory_mutex_);

            if (TrajectoryPointVector_.desired_trajectory_.empty())
                return;

            trajectory_data = TrajectoryPointVector_;
        }

        // =====================================================
        // 2. COPY LATEST EKF STATE AND FIND TIMESTAMP MATCH
        // =====================================================

        nav_msgs::msg::Odometry latest_ekf_state;

        double min_diff_ms =
            std::numeric_limits<double>::max();

        double selected_ekf_time_ms = 0.0;
        double oldest_ekf_time_ms = 0.0;
        double newest_ekf_time_ms = 0.0;
        size_t buffer_size = 0;

        {
            std::lock_guard<std::mutex> lock(ekf_mutex_);

            if (ekf_buffer_.empty())
                return;

            buffer_size = ekf_buffer_.size();

            oldest_ekf_time_ms =
                rclcpp::Time(ekf_buffer_.front().header.stamp).nanoseconds() / 1e6;

            newest_ekf_time_ms =
                rclcpp::Time(ekf_buffer_.back().header.stamp).nanoseconds() / 1e6;

            latest_ekf_state = ekf_buffer_.back();

            for (const auto &state : ekf_buffer_)
            {
                const double ekf_time_ms =
                    static_cast<double>(
                        rclcpp::Time(state.header.stamp).nanoseconds()) /
                    1e6;

                const double diff_ms = std::abs(
                    ekf_time_ms - trajectory_data.timestamp_ms);

                if (diff_ms < min_diff_ms)
                {
                    min_diff_ms = diff_ms;
                    latest_ekf_state = state;
                    selected_ekf_time_ms = ekf_time_ms;
                }
            }
        }

        // =====================================================
        // MEASUREMENT AGES RELATIVE TO CURRENT SIMULATOR TIME
        // =====================================================

        const double sim_now_ms =
            static_cast<double>(
                this->get_clock()->now().nanoseconds()) /
            1e6;

        // Age of trajectory currently used by MPC
        const double trajectory_age_sim_ms =
            sim_now_ms - trajectory_data.timestamp_ms;

        // Age of the selected EKF measurement
        const double selected_ekf_age_sim_ms =
            sim_now_ms - selected_ekf_time_ms;

        // Age of the newest available EKF measurement
        const double newest_ekf_age_sim_ms =
            sim_now_ms - newest_ekf_time_ms;

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "SIM TIME=%.3f ms | "
            "Trajectory age=%.3f ms | "
            "Selected EKF age=%.3f ms | "
            "Newest EKF age=%.3f ms",
            sim_now_ms,
            trajectory_age_sim_ms,
            selected_ekf_age_sim_ms,
            newest_ekf_age_sim_ms);

        // Log the timestamp difference
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Trajectory/EKF timestamp difference: %.3f ms",
            min_diff_ms);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Trajectory=%.3f | EKF selected=%.3f | "
            "Signed difference=%.3f ms | "
            "Buffer oldest=%.3f | newest=%.3f | size=%zu",
            trajectory_data.timestamp_ms,
            selected_ekf_time_ms,
            selected_ekf_time_ms - trajectory_data.timestamp_ms,
            oldest_ekf_time_ms,
            newest_ekf_time_ms,
            buffer_size);

        // =====================================================
        // 2.1 safety guard for EKF pose and desired traejctory timestamp dismatch // VALIDATE TIMESTAMP SYNCHRONIZATION
        // =====================================================

        constexpr double MAX_SYNC_DIFF_MS = 50.0;

        if (min_diff_ms > MAX_SYNC_DIFF_MS)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Rejecting MPC update: trajectory/EKF timestamp "
                "difference too large (%.3f ms, maximum %.3f ms)",
                min_diff_ms,
                MAX_SYNC_DIFF_MS);

            return;
        }

        // =====================================================
        // 3. EXTRACT CURRENT VEHICLE STATE [E, N, psi, v]
        // =====================================================

        const auto &p = latest_ekf_state.pose.pose.position;
        const auto &q = latest_ekf_state.pose.pose.orientation;

        // Normalize quaternion
        const double q_norm = std::sqrt(
            q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);

        if (!std::isfinite(q_norm) || q_norm < 1e-12)
            return;

        const double qw = q.w / q_norm;
        const double qx = q.x / q_norm;
        const double qy = q.y / q_norm;
        const double qz = q.z / q_norm;

        // Extract yaw in ENU
        const double yaw = std::atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz));

        // TODO (EKF publisher):
        // Publish body-frame longitudinal velocity in
        // Odometry.twist.twist.linear.x.
        //
        // Assumption: twist is expressed in the vehicle body frame
        // and the body X axis points forward.

        const double v_current =
            latest_ekf_state.twist.twist.linear.x;

        if (!std::isfinite(v_current))
            return;

        // MATLAB bicycle model state: [E; N; yaw; velocity]
        double x_current[4] = {
            p.x,
            p.y,
            yaw,
            v_current};

        // =====================================================
        // 4. CALCULATE DYNAMIC MPC SAMPLING PERIOD
        // =====================================================

        const rclcpp::Time current_time =
            this->get_clock()->now();

        if (current_time.nanoseconds() <= 0)
            return;

        // Ts represents the elapsed simulation seconds between MPC executions.
        double Ts = mpc_cfg_.Ts;

        if (has_last_mpc_start_)
        {
            Ts = (current_time - last_mpc_start_).seconds();
        }

        if (!std::isfinite(Ts) || Ts <= 0.0)
            return;

        if (!std::isfinite(Ts) || Ts <= 0.0)
            return;

        // =====================================================
        // 5. GENERATE REFERENCE HORIZON
        // =====================================================

        constexpr int Np = 10;

        if (mpc_cfg_.Np != Np)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Generated MPC requires Np=10");

            return;
        }

        double refHorizon[4 * Np]{};

        // TODO: Replace with configured target velocity
        constexpr double v_reference = 0.5; // m/s

        // Reference horizon diagnostics
        ReferenceHorizonDebug ref_debug;

        if (!buildReferenceHorizon(
                trajectory_data.desired_trajectory_,
                x_current[0], // Current East
                x_current[1], // Current North
                x_current[2], // Current yaw
                v_reference,
                Ts,
                refHorizon,
                &ref_debug))
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Cannot construct MPC reference horizon");

            return;
        }

        // Log reference construction diagnostics
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Vehicle=(%.3f, %.3f) | "
            "Projection=(%.3f, %.3f) | "
            "Gap=%.3f m",
            ref_debug.vehicle_E,
            ref_debug.vehicle_N,
            ref_debug.projection_E,
            ref_debug.projection_N,
            ref_debug.projection_distance);

        // =====================================================
        // 6. PREPARE MATLAB-GENERATED MPC PARAMETERS
        // =====================================================

        struct0_T params{};

        params.Np = mpc_cfg_.Np;
        params.Ts = Ts;
        params.L = mpc_cfg_.L;

        params.deltaMax = mpc_cfg_.deltaMax;
        params.aMin = mpc_cfg_.aMin;
        params.aMax = mpc_cfg_.aMax;

        std::copy_n(&mpc_cfg_.Q[0], 16, params.Q);
        std::copy_n(&mpc_cfg_.R[0], 4, params.R);
        std::copy_n(&mpc_cfg_.S[0], 4, params.S);

        // =====================================================
        // 7. PREPARE OUTPUT ARGUMENTS
        // =====================================================

        double u_optimal[2]{};

        coder::array<double, 2U> U_optimal;

        // =====================================================
        // 8. EXECUTE GENERATED MPC
        // =====================================================

        last_mpc_start_ = current_time;
        has_last_mpc_start_ = true;

        const auto start = std::chrono::steady_clock::now();

        mpcController(
            x_current,
            refHorizon,
            u_last_applied_.data(),
            &params,
            u_optimal,
            U_optimal);

        // =====================================================
        // 8.1. LOGS
        // =====================================================
        const double execution_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start)
                .count();

        mpcLoggger::writeMPCLog(
            Ts,
            x_current,
            refHorizon,
            u_last_applied_,
            params,
            u_optimal,
            U_optimal,
            execution_ms);

        // =====================================================
        // 9. EXTRACT OPTIMAL CONTROL
        // =====================================================

        const double steering = u_optimal[0];
        const double acceleration = u_optimal[1];

        const double steering_normalized =
            steeringToBeamNG(steering);
        // =====================================================
        // 10. GENERATE TARGET VELOCITY FROM MPC ACCELERATION
        // =====================================================

        if (!speed_target_initialized_)
        {
            v_target_ = std::clamp(
                v_current, 0.0, v_reference);

            speed_target_initialized_ = true;
        }

        // Integrate desired acceleration
        v_target_ += acceleration * Ts;

        // Limit target velocity to forward-driving range
        v_target_ = std::clamp(
            v_target_,
            0.0,
            v_reference);

        // =====================================================
        // 11. PI VELOCITY CONTROLLER
        // =====================================================

        const PedalCommand pedals = speed_pi_.update(
            v_target_,
            v_current,
            Ts);

        // =====================================================
        // 12. PREPARE BEAMNG CONTROL MESSAGE
        // =====================================================

        beamng_msgs::msg::VehicleControl cmd{};

        // Steering conversion is handled separately.
        // Assign cmd.steering using your calibrated conversion
        // before publishing.

        cmd.gear = static_cast<int8_t>(1);
        cmd.steering = steering_normalized;
        cmd.throttle = pedals.throttle;
        cmd.brake = pedals.brake;

        cmd.parkingbrake = 0.0;
        cmd.clutch = 0.0;

        // =====================================================
        // 13. PUBLISH CONTROL
        // =====================================================

        // IMPORTANT: Set cmd.steering before enabling this
        // publish call, otherwise steering defaults to zero.
        control_pub_->publish(cmd);
        // Approximation: assume requested MPC controls are realized
        u_last_applied_[0] = steering;
        u_last_applied_[1] = acceleration;

        // =====================================================
        // 14. DEBUG
        // =====================================================

        /*
        RCLCPP_INFO(
            this->get_logger(),
            "MPC: a=%.3f m/s2 | "
            "v_target=%.3f m/s | "
            "v_current=%.3f m/s | "
            "throttle=%.3f | brake=%.3f",
            acceleration,
            v_target_,
            v_current,
            cmd.throttle,
            cmd.brake);

            */
        // TODO: Publish steering and acceleration commands.
        // TODO: Update u_last_applied_ when commands are applied.
    }

    // =================== privat attributes ==================

    // Latest received trajectory
    TrajectoryPointVector TrajectoryPointVector_;
    std::mutex trajectory_mutex_;

    // Timestamped EKF pose history
    std::deque<nav_msgs::msg::Odometry> ekf_buffer_;
    std::mutex ekf_mutex_;

    // MPC timer
    rclcpp::TimerBase::SharedPtr mpc_timer_;

    // Callback groups
    rclcpp::CallbackGroup::SharedPtr trajectory_group_;
    rclcpp::CallbackGroup::SharedPtr mpc_group_;
    rclcpp::CallbackGroup::SharedPtr ekf_group_;

    // MPC alg cfg
    MPC::mpc_cfg mpc_cfg_;
    bool has_last_mpc_start_{false};
    rclcpp::Time last_mpc_start_{0, 0, RCL_ROS_TIME};

    // Previous MPC-requested control [steering rad, acceleration m/s²]
    std::array<double, 2> u_last_applied_{0.0, 0.0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MPCSubscriber>();

    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 3);

    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

/*

reference: https://storage.googleapis.com/dixykwhqbouqre/kinematic-bicycle-model-equations.html?utm_source=chatgpt.com

*/
