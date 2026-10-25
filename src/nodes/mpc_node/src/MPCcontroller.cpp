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

#include <functional>

#include "mpcInterface.hpp"

struct TrajectoryPoint
{
    int32_t u;
    int32_t v;

    double E;
    double N;
    double U;
};

struct TrajectoryPointVector
{
    std::vector<TrajectoryPoint> desired_trajectory_;
    double timestamp_ms;
};

class MPCSubscriber : public rclcpp::Node
{
public:
    MPCSubscriber() : Node("mpc_subscriber")
    {
        mpcController_initialize();

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

        mpc_timer_ = this->create_wall_timer(
            std::chrono::nanoseconds(33'333'333), // 30 Hz
            std::bind(&MPCSubscriber::performMPC, this),
            mpc_group_);
    }

    ~MPCSubscriber()
    {
        mpcController_terminate();
    }

private:
    // ==================== subscriber callbacks ======================
    void trajectoryCallback(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
    {

        std::string frame = msg->header.frame_id; // "enu"

        // Create iterators for each field
        sensor_msgs::PointCloud2ConstIterator<int32_t> u(*msg, "u");
        sensor_msgs::PointCloud2ConstIterator<int32_t> v(*msg, "v");

        sensor_msgs::PointCloud2ConstIterator<double> x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<double> y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<double> z(*msg, "z");

        // Number of received trajectory points
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

        // Example: accessing the extracted trajectory
        for (const auto &point : trajectory)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Pixel (%d,%d) -> ENU (%.3f, %.3f, %.3f)",
                point.u, point.v,
                point.E, point.N, point.U);
        }

        {
            // Update shared trajectory
            std::lock_guard<std::mutex>
                lock(trajectory_mutex_);

            TrajectoryPointVector_.desired_trajectory_ = std::move(trajectory);
            TrajectoryPointVector_.timestamp_ms = static_cast<double>(
                                                      rclcpp::Time(msg->header.stamp).nanoseconds()) /
                                                  1e6;
        }
    }

    void EKFCallback(
        const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(ekf_mutex_);

        ekf_buffer_.push_back(*msg);

        if (ekf_buffer_.size() > 60)
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

        {
            std::lock_guard<std::mutex> lock(ekf_mutex_);

            if (ekf_buffer_.empty())
                return;

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
                    min_diff_ms = diff_ms;
            }
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

        const auto current_time =
            std::chrono::steady_clock::now();

        double Ts = mpc_cfg_.Ts;

        if (has_last_mpc_start_)
        {
            Ts = std::chrono::duration<double>(
                     current_time - last_mpc_start_)
                     .count();
        }

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
        constexpr double v_reference = 2.0;

        if (!buildReferenceHorizon(
                trajectory_data.desired_trajectory_,
                x_current[0],
                x_current[1],
                v_reference,
                Ts,
                refHorizon))
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Cannot construct MPC reference horizon");
            return;
        }

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

        const double execution_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start)
                .count();

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

        cmd.steering = steering_normalized;
        cmd.throttle = pedals.throttle;
        cmd.brake = pedals.brake;

        cmd.parkingbrake = 0.0;
        cmd.clutch = 0.0;
        cmd.gear = 1; // Verify this for your vehicle's gearbox

        control_pub_->publish(cmd);

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
    std::chrono::steady_clock::time_point last_mpc_start_;

    // Previous MPC-requested control [steering rad, acceleration m/s²]
    std::array<double, 2> u_last_applied_{0.0, 0.0};

    bool buildReferenceHorizon(
        const std::vector<TrajectoryPoint> &trajectory,
        double E_current,
        double N_current,
        double v_reference,
        double Ts,
        double refHorizon[40])
    {
        constexpr int Np = 10;

        struct Point2D
        {
            double E;
            double N;
        };

        // =====================================================
        // 1. PREPARE THE DESIRED PATH
        // =====================================================

        std::vector<Point2D> path;

        for (const auto &point : trajectory)
        {
            if (!std::isfinite(point.E) ||
                !std::isfinite(point.N))
                continue;

            // Remove consecutive duplicate points
            if (!path.empty() &&
                std::hypot(point.E - path.back().E,
                           point.N - path.back().N) < 1e-4)
                continue;

            path.push_back({point.E, point.N});
        }

        if (path.size() < 2 ||
            !std::isfinite(Ts) || Ts <= 0.0 ||
            !std::isfinite(v_reference) || v_reference < 0.0)
            return false;

        // =====================================================
        // 2. CALCULATE CUMULATIVE PATH DISTANCES
        // =====================================================

        std::vector<double> s(path.size(), 0.0);

        for (size_t i = 1; i < path.size(); ++i)
        {
            const double dE = path[i].E - path[i - 1].E;
            const double dN = path[i].N - path[i - 1].N;

            s[i] = s[i - 1] + std::hypot(dE, dN);
        }

        // =====================================================
        // 3. FIND VEHICLE'S CLOSEST PROJECTION
        // =====================================================

        double s_current = 0.0;

        double min_distance_sq =
            std::numeric_limits<double>::max();

        for (size_t i = 0; i + 1 < path.size(); ++i)
        {
            // Segment direction
            const double dE = path[i + 1].E - path[i].E;
            const double dN = path[i + 1].N - path[i].N;

            const double len_sq = dE * dE + dN * dN;

            // Projection parameter
            double t =
                ((E_current - path[i].E) * dE +
                 (N_current - path[i].N) * dN) /
                len_sq;

            // Restrict projection to this segment
            t = std::max(0.0, std::min(1.0, t));

            // Projected point coordinates
            const double E_proj = path[i].E + t * dE;
            const double N_proj = path[i].N + t * dN;

            // Squared distance to vehicle
            const double distance_sq =
                std::pow(E_current - E_proj, 2) +
                std::pow(N_current - N_proj, 2);

            if (distance_sq < min_distance_sq)
            {
                min_distance_sq = distance_sq;

                // Distance along the path to projection
                s_current = s[i] + t * (s[i + 1] - s[i]);
            }
        }

        // =====================================================
        // 4. GENERATE THE FUTURE REFERENCE STATES
        // =====================================================

        for (int k = 0; k < Np; ++k)
        {
            // Future reference distance along path
            const double s_ref = std::min(
                s_current + (k + 1) * v_reference * Ts,
                s.back());

            // Find the segment containing s_ref
            auto it = std::lower_bound(
                s.begin() + 1,
                s.end(),
                s_ref);

            size_t j = static_cast<size_t>(
                std::distance(s.begin(), it));

            if (j >= path.size())
                j = path.size() - 1;

            // Interpolation parameter
            const double alpha =
                (s_ref - s[j - 1]) / (s[j] - s[j - 1]);

            // Interpolated reference ENU coordinates
            const double E_ref =
                path[j - 1].E +
                alpha * (path[j].E - path[j - 1].E);

            const double N_ref =
                path[j - 1].N +
                alpha * (path[j].N - path[j - 1].N);

            // Reference heading from segment direction
            const double yaw_ref = std::atan2(
                path[j].N - path[j - 1].N,
                path[j].E - path[j - 1].E);

            // Stop reference at the end of the available path
            const double v_ref =
                (s_ref >= s.back()) ? 0.0 : v_reference;

            // Store MATLAB-compatible column-major matrix
            refHorizon[4 * k + 0] = E_ref;
            refHorizon[4 * k + 1] = N_ref;
            refHorizon[4 * k + 2] = yaw_ref;
            refHorizon[4 * k + 3] = v_ref;
        }

        return true;
    }
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
