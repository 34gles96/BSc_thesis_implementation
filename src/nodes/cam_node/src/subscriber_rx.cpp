#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <mutex>
#include "detectObstacles.hpp"
#include "ThreadJoinGuard.hpp"

#include <deque>
#include <limits>
#include <cstdint>
#include "BirdEyeViewVisualizer.hpp"
#include "extractLaneDepthRange.hpp"
//============ include from MATLAB coder lib ================

// debug
#include "debugWhiteLine.hpp"

#include <cmath>
// ===============================================
// New includes for synchronization
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/msg/image.hpp>

#include "readConfig.hpp"
#include "fitLanePoly.hpp"
#include "desiredTrajectory.hpp"
#include "pinholeBackProjection.hpp"
#include "config.hpp"
#include "initializeCameraMounting.hpp"
#include "trajectoryToENU.hpp"

// ================for transmitting desiredTrajectorENU as publisher ===============================
// needs to trsanmit (u,v) -> (X,Y,Z) layout
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

// for perfomance measurement
#include <chrono>

constexpr std::size_t MAX_EKF_BUFFER_SIZE = 500; // At roughly 100 Hz, this gives approximately 5 seconds of pose history.

// Inside your class members (e.g., in CameraSubscriber)
struct SmoothedPlane
{
    double A = 0.0;
    double B = -1.0; // Initial gravity vector approximation in camera frame
    double C = 0.0;
    double D = 1.05; // Your nominal camera height
} smoothed_plane_;

class CameraSubscriber : public rclcpp::Node
{
public:
    CameraSubscriber()
        : Node("camera_subscriber") // initialization list for parent class
    {

        // Enable ROS 2 simulation time
        this->set_parameter(
            rclcpp::Parameter("use_sim_time", true));

        // ======================== PUBLISHER FOR DESIREDTRAJECTORY_ENU ========================

        desired_trajectory_pub_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/adas/desired_trajectory/enu",
                rclcpp::QoS(1));

        // ========= subscriton options for async exetution using callback groups =========
        //  Create callback groups
        camera_callback_group_ =
            this->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);

        ekf_callback_group_ =
            this->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);

        // Subscription options
        rclcpp::SubscriptionOptions camera_options;
        camera_options.callback_group = camera_callback_group_;

        rclcpp::SubscriptionOptions ekf_options;
        ekf_options.callback_group = ekf_callback_group_;

        // MATLAB generated lib inti for transfromation cam2body:
        camera_transform_initialize();
        T_cam2body_ = initializeCameraMounting(
            "/home/base/ADC_27/configData/cam_config/CameraConfig.yaml");

        camera_params_ = loadCameraParams(
            "/home/base/ADC_27/configData/cam_config/CameraConfig.yaml");

        cv::Size img_size(camera_params_.width, camera_params_.height);

        point_cloud_3D_ = cv::Mat(
            img_size,
            CV_32FC3,
            cv::Scalar(0.0f, 0.0f, 0.0f)); // initialization of pointcloud:size:u x v, one element memlayout : CV_32FC3

        // Subscription for channel of left image stream
        // QoS configuration for camera image streams
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1));

        sub_left_.subscribe(
            this,
            "/vehicles/beamng_bridge/sensors/camera_L/colour",
            qos.get_rmw_qos_profile(),
            camera_options);

        sub_right_.subscribe(
            this,
            "/vehicles/beamng_bridge/sensors/camera_R/colour",
            qos.get_rmw_qos_profile(),
            camera_options);

        // Subscribe to the EKF-estimated vehicle pose
        ekf_pose_sub_ =
            this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/adas/ekf/pose",
                rclcpp::SensorDataQoS(),
                std::bind(
                    &CameraSubscriber::write2EKFbuff,
                    this,
                    std::placeholders::_1),
                ekf_options);

        // Keep queue size small (2) to prevent old frame accumulation
        // sync_ = std::make_shared<message_filters::Synchronizer<ExactSyncPolicy>>(
        // ExactSyncPolicy(100), sub_left_, sub_right_);
        using ApproxSyncPolicy = message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::Image,
            sensor_msgs::msg::Image>;

        // Pass the queue size (2) directly to Synchronizer, then set the 8 ms max interval via getPolicy()
        sync_ = std::make_shared<message_filters::Synchronizer<ApproxSyncPolicy>>(
            5, sub_left_, sub_right_);

        sync_->getPolicy()->setMaxIntervalDuration(
            rclcpp::Duration(std::chrono::milliseconds(20)));

        // Allow same-tick sensor polling offset (< 8 ms), block cross-frame pairing (~33 ms)

        sync_->registerCallback(std::bind(
            &CameraSubscriber::stereo_callback,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

        int minDisparity = 0;
        int numDisparities = 16 * 16; // 160 disparities
        int blockSize = 9;            // INCREASED: A large block size (15x15) grabs more road texture

        stereo_ = cv::StereoSGBM::create(
            0,
            // 16 * 8,
            numDisparities, // it constraints the Z_min (depth), minimum measurable depth Z_min is 2.23m
            5,
            8 * 3 * 5 * 5,
            32 * 3 * 5 * 5,
            1,
            63,
            10,
            50,
            2,
            cv::StereoSGBM::MODE_SGBM_3WAY);

        /*
                stereo_ = cv::StereoSGBM::create(
                    minDisparity,
                    numDisparities,
                    blockSize,
                    8 * 3 * blockSize * blockSize,
                    32 * 3 * blockSize * blockSize,
                    1,
                    63,
                    10,
                    50,
                    2,
                    cv::StereoSGBM::MODE_SGBM_3WAY);
        */

        config::updateTrajectoryBounds(
            camera_params_,
            numDisparities,
            smoothed_plane_.D);

        RCLCPP_INFO(
            this->get_logger(),
            "Updated trajectory bounds: MIN_TRAJECTORY_Z = %.2f m, PREDICTED_HORIZON = %.2f m",
            config::MIN_TRAJECTORY_Z,
            config::PREDICTED_HORIZON);
        if (!std::isfinite(config::MIN_TRAJECTORY_Z) ||
            !std::isfinite(config::PREDICTED_HORIZON) ||
            config::PREDICTED_HORIZON <= config::MIN_TRAJECTORY_Z)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Invalid trajectory depth range: min=%.2f, horizon=%.2f",
                config::MIN_TRAJECTORY_Z,
                config::PREDICTED_HORIZON);
        }
    }

    ~CameraSubscriber()
    {
        camera_transform_terminate();
    }

    // ================================================================
    // Members and internal functions due to event of subscripton
    // ================================================================
private:
    std::atomic<bool> is_cam_processing_{false};
    void stereo_callback( // introductiong threads for parallal exectution between: pinholeXYZ and line_detection
        const sensor_msgs::msg::Image::ConstSharedPtr msg_left,
        const sensor_msgs::msg::Image::ConstSharedPtr msg_right)
    {
        // Drop incoming pair if the previous frame is still computing
        if (is_cam_processing_.exchange(true))
        {
            return;
        }
        // RAII guard: guarantees is_processing_ resets to false even on early returns
        struct Guard
        {
            std::atomic<bool> &flag;
            ~Guard() { flag.store(false); }
        } guard{is_cam_processing_};

        // DEBUG
        double dt_ms = std::abs(
                           (rclcpp::Time(msg_left->header.stamp) - rclcpp::Time(msg_right->header.stamp)).nanoseconds()) *
                       1e-6;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Stereo sync delta: %.3f ms", dt_ms);

        // 1. Get the camera measurement timestamp
        const rclcpp::Time image_stamp(
            msg_left->header.stamp,
            RCL_ROS_TIME);

        // 2. Compare with the latest ROS simulation clock
        const double clock_difference_ms =
            (this->get_clock()->now() - image_stamp).seconds() * 1000.0;

        RCLCPP_INFO(
            this->get_logger(),
            "ROS clock - camera timestamp: %.3f ms",
            clock_difference_ms);

        const auto callback_start = std::chrono::steady_clock::now();

        // ============ SIMULATION TIME DIAGNOSTIC ============

        // Current ROS time.
        // When use_sim_time=true, this comes from /clock.
        const rclcpp::Time now = this->get_clock()->now();

        // Camera message timestamp, interpreted as ROS time.
        const rclcpp::Time camera_stamp(
            msg_left->header.stamp,
            RCL_ROS_TIME);

        // Simulation clock might not be initialized yet.
        if (now.nanoseconds() == 0)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Waiting for BeamNG simulation clock...");
            return;
        }

        // Age of the image in simulation time.
        const double camera_age_ms =
            (now - camera_stamp).seconds() * 1000.0;

        RCLCPP_INFO(
            this->get_logger(),
            "CAMERA CALLBACK START | Simulation image age: %.3f ms",
            camera_age_ms);

        // ================= CAMERA / EKF TIMESTAMP MATCHING =================

        const int64_t cam_time =
            rclcpp::Time(msg_left->header.stamp).nanoseconds();

        geometry_msgs::msg::PoseStamped closest_pose;

        int64_t min_diff = std::numeric_limits<int64_t>::max();
        bool found = false;

        {
            std::lock_guard<std::mutex> lock(ekf_mutex_);

            for (const auto &pose : ekf_buffer_)
            {
                const int64_t ekf_time =
                    rclcpp::Time(pose.header.stamp).nanoseconds();

                const int64_t diff =
                    std::abs(ekf_time - cam_time);

                if (diff < min_diff)
                {
                    min_diff = diff;
                    closest_pose = pose;
                    found = true;
                }
            }
        }

        const bool ekf_matched =
            found && min_diff <= 20'000'000LL;

        // =========== ORIGINAL CAMERA PROCESSING ===========

        try
        {
            // Convert ROS Image -> OpenCV image
            image_bgr8 = cv_bridge::toCvCopy(
                             msg_left,
                             "bgr8")
                             ->image;

            image_bgr8R_ = cv_bridge::toCvCopy(
                               msg_right,
                               "bgr8")
                               ->image;

            // ========= DEBUG ======
            // Refresh with the current camera frame even if later processing rejects it.
            // cv::imshow("Desired Lane Trajectory", image_bgr8);
            // cv::waitKey(1);
            // ===============
            using Clock_debug = std::chrono::steady_clock; // for timstamp measuring using cpu ticks in std::milli resolution storered in double tpye
            std::exception_ptr stereo_exception;           // for proagation exception from child thread in ordr to not call std::termiante() when exception occured in child thread
            cv::Mat output_cloud;                          // stadlon resources of child thread output
            double stereo_ms = 0.0;
            const auto parallel_start = Clock_debug::now();

            std::thread to_XYZ_transfrom_thread( // Constructing std::thread starts the new thread, to_XYZ_transfrom_thread start exectution from constructing
                [this,
                 left = image_bgr8,
                 right = image_bgr8R_, // coppy from soroounding scopr
                 &output_cloud,        // reference from sourinding scope
                 &stereo_exception,
                 &stereo_ms]() mutable
                {
                    try
                    {

                        const auto child_thread_start = Clock_debug::now();

                        cv::Mat cloud_buffer = cv::Mat::zeros( //...
                            left.size(),
                            CV_32FC3);

                        // invokint thread func

                        // // TRANSFORMING  (u,v) w coorinate into XYZ (camera framte)
                        output_cloud = pinHole2XYZ(
                            left,
                            right,
                            stereo_,
                            camera_params_,
                            cloud_buffer);

                        const auto finish = Clock_debug::now();
                        stereo_ms = std::chrono::duration<double, std::milli>(
                                        finish - child_thread_start)
                                        .count();
                    }
                    catch (...)
                    {

                        stereo_exception = std::current_exception();
                    }
                });

            // Ensures joining even on early return or exception
            ThreadJoinGuard thread_guard(to_XYZ_transfrom_thread);

            // ===================== Executed by the ROS executor thread =============================== // multiexclsuive way (no another instance of this ros2 exectutor thread)

            // transfroming the image into hsv color space
            cv::Mat hsv;
            cv::cvtColor(image_bgr8, hsv, cv::COLOR_BGR2HSV);

            // Split HSV into 3 separate single-channel images
            // std::vector<cv::Mat> hsv_channels;
            // cv::split(hsv, hsv_channels);

            // 3. Apply CLAHE to the V (Value/Brightness) channel
            // cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
            // clahe->apply(hsv_channels[2], hsv_channels[2]);

            // 4. Merge channels back into an HSV image
            // cv::merge(hsv_channels, hsv);

            // Define the HSV threshold range for WHITE
            // HSV ranges in OpenCV: H: 0-180, S: 0-255, V: 0-255
            // White = any color (H: 0-180), very low saturation (S: 0-50), very high brightness (V: 200-255)
            // cv::Scalar lower_white(0, 0, 230);
            // cv::Scalar upper_white(180, 50, 255);
            // Require higher brightness and a slightly stricter filter

            cv::Scalar lower_white(0, 0, 200);
            cv::Scalar upper_white(180, 50, 255);
            /*
            cv::Scalar lower_white(0, 0, 240);
            cv::Scalar upper_white(180, 30, 255);
            */

            //  Create a binary mask (white lines become pure white 255, everything else black 0)
            cv::Mat mask;
            cv::inRange(hsv, lower_white, upper_white, mask); // pixels in range of (lower_white, upper_hwhite) -> white, else black. stored: mask

            // ========================================== DEBUG ===================================================

            // =================================================== ==================================
            // defining region of interest (lower part of the streamed screen (ignoring ,trees and the vehicle hood)).
            // defining region of interest:

            int roi_top = static_cast<int>(mask.rows * config::img_in_stream_vertical_cut_top); // taking boundery for camputred img stream : vertical boundary from up
            int roi_bottom = static_cast<int>(mask.rows * config::img_in_stream_vertical_cut_bottom);

            cv::Rect roi_area(
                0,
                roi_top,
                mask.cols,
                roi_bottom - roi_top);

            // ============== DEBUG =======================

            // Create 3-channel debug image from binary mask
            cv::Mat debug_image;
            cv::cvtColor(mask, debug_image, cv::COLOR_GRAY2BGR);

            // Draw GREEN ROI rectangle
            cv::rectangle(
                debug_image,
                roi_area,
                cv::Scalar(0, 255, 0),
                3);

            // Add green label
            cv::putText(
                debug_image,
                "Region of Interest",
                cv::Point(10, roi_top - 10),
                cv::FONT_HERSHEY_SIMPLEX,
                0.6,
                cv::Scalar(0, 255, 0),
                2);

            cv::imshow("ROI Debug View", debug_image);
            // cv::waitKey(1); //TODO:recomment
            //  --------------------------------

            cv::Mat mask_roi = mask(roi_area); // applying boundary

            // Extracting the (u,v) pixel coordinates of the white line
            std::vector<cv::Point> white_line_pts;

            // Scan the masked region
            for (int y = 0; y < mask_roi.rows; ++y)
            {
                for (int x = 0; x < mask_roi.cols; ++x)
                {
                    // If the pixel is white (255)
                    if (mask_roi.at<uchar>(y, x) == 255)
                    {
                        // Transform ROI coordinates back into the original image coordinates
                        int original_y = y + roi_area.y;
                        int original_x = x + roi_area.x;

                        white_line_pts.push_back(
                            cv::Point(original_x, original_y));
                    }
                }
            }

            // Print out the total number of line points found
            std::cout << "Extracted " << white_line_pts.size() << " white lane pixels from Left Camer." << std::endl;

            // trasnroming the extracted (u,v) white coorinate into XYZ (camera framte)
            // creating a black img w intialization
            white_line_imgL = cv::Mat::zeros(image_bgr8.size(), CV_8UC1);

            for (const auto &pt : white_line_pts)
            {
                // checking boinds
                if (pt.x >= 0 && pt.x < white_line_imgL.cols && pt.y >= 0 && pt.y < white_line_imgL.rows)
                {
                    white_line_imgL.at<uchar>(pt.y, pt.x) = 255;
                }
                else
                {
                    std::cerr << "error in creating matrix of extracted white points" << std::endl;
                }
            }

            showExtractedWhitePixels(white_line_imgL);

            // Optional: Visual debug windows
            // cv::imshow("Original Frame", image_bgr8);
            // cv::imshow("White Lane Mask", mask);
            // cv::waitKey(0);

            // Lane detection has completed
            const auto lane_finished = Clock_debug::now();
            // wating for 'to_XYZ_transfrom_thread finish w .join()
            to_XYZ_transfrom_thread.join(); // TODO: checkin joinable ??
            // checking any exception occoured i child thread.
            if (stereo_exception)
            {
                std::rethrow_exception(stereo_exception); // who chacting?
            }
            // Take the result from the additional thread
            point_cloud_3D_ = std::move(output_cloud);

            // Validate both results
            if (point_cloud_3D_.empty() ||
                white_line_imgL.size() != point_cloud_3D_.size())
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Invalid reconstructed cloud or dimension mismatch");
                return;
            }

            const auto parallel_finished = Clock_debug::now();

            const double lane_ms =
                std::chrono::duration<double, std::milli>(
                    lane_finished - parallel_start)
                    .count();

            const double parallel_ms =
                std::chrono::duration<double, std::milli>(
                    parallel_finished - parallel_start)
                    .count();

            RCLCPP_INFO(
                this->get_logger(),
                "Lane: %.2f ms | Stereo: %.2f ms | Combined: %.2f ms",
                lane_ms,
                stereo_ms,
                parallel_ms);

            RoadPlane plane_for_lane_fit_tmp;

            plane_for_lane_fit_tmp.normal = cv::Vec3d(
                smoothed_plane_.A,
                smoothed_plane_.B,
                smoothed_plane_.C);

            plane_for_lane_fit_tmp.d = smoothed_plane_.D;

            std::vector<LanePolynomial> whiteCurveParams =
                fitLanePolynomialOpenCV(white_line_imgL, point_cloud_3D_, laneW, camera_params_, plane_for_lane_fit_tmp);

            // Use params_for_use for drawing/processing.
            // ============== DEBUG ================= ======== ========
            std::cout << "dinamically caluclated road witdh is: " << laneW << std::endl;
            // ======== ======== ======== ======== ======== ========

            // ============== ROAD PARAMS ================= ======== ========

            // 1. Convert the existing CV_32FC3 point cloud to CV_64FC3 for detectRoad()
            cv::Mat_<cv::Vec3d> XYZ_cam;
            point_cloud_3D_.convertTo(XYZ_cam, CV_64FC3);

            // 2. Declare an EMPTY RoadPlane object
            RoadPlane dynamic_road_plane;

            // 3. INVOKE THE FUNCTION: We pass dynamic_road_plane into detectRoad.
            // Because it is passed by reference, the RANSAC loop inside detectRoad
            // actively fills dynamic_road_plane with the A, B, C, and D values.
            bool road_found = detectRoad(dynamic_road_plane, XYZ_cam);

            // Apply a smoothing factor (alpha between 0.1 and 0.2 works best for 30Hz)
            double alpha = 0.05;
            // Create a temporary RoadPlane structure using the smoothed values
            RoadPlane final_plane;

            const auto &raw_n = dynamic_road_plane.normal;
            const double norm = cv::norm(raw_n);
            if (road_found && std::isfinite(norm) && norm > 1e-6 &&
                std::isfinite(dynamic_road_plane.d))
            {

                cv::Vec3d n = raw_n / norm;
                double d = dynamic_road_plane.d / norm;
                const cv::Vec3d previous(smoothed_plane_.A,
                                         smoothed_plane_.B,
                                         smoothed_plane_.C);

                if (n.dot(previous) < 0.0) // bcs we can describe the road w 2 diff sign convention, and diff frame produce diff math exőression
                {
                    n = -n;
                    d = -d;
                }
                smoothed_plane_.A = (1.0 - alpha) * smoothed_plane_.A + alpha * dynamic_road_plane.normal[0];
                smoothed_plane_.B = (1.0 - alpha) * smoothed_plane_.B + alpha * dynamic_road_plane.normal[1];
                smoothed_plane_.C = (1.0 - alpha) * smoothed_plane_.C + alpha * dynamic_road_plane.normal[2];
                smoothed_plane_.D = (1.0 - alpha) * smoothed_plane_.D + alpha * dynamic_road_plane.d;

                final_plane.normal = cv::Vec3d(smoothed_plane_.A, smoothed_plane_.B, smoothed_plane_.C);
                final_plane.d = smoothed_plane_.D;
            }
            else
            {
                final_plane.normal = cv::Vec3d(smoothed_plane_.A, smoothed_plane_.B, smoothed_plane_.C);
                final_plane.d = smoothed_plane_.D;
            }

            /*

   // ==================== obstacle detection params =================

// Convert the existing CV_32FC3 point cloud to CV_64FC3
// for detectRoad() and detectObstacles().
cv::Mat_<cv::Vec3d> XYZ_cam;
point_cloud_3D_.convertTo(XYZ_cam, CV_64FC3);

if (detectRoad(road_plane_, XYZ_cam))
{
   // ========== DETECT OBSTACLES ==========

   detectObstacles(
       XYZ_cam,
       road_plane_,
       obstacleTreshold_,
       heightMap_,
       ObstacleMask,
       upDirecton);

   // ========== CREATE INDEPENDENT COPIES ==========

   cv::Mat rawImage = image_bgr8.clone(); // deep copy
   filteredImage_ = image_bgr8.clone();

   cv::Mat1b filteredMask = ObstacleMask.clone();

   // ========== RAW OBSTACLE DETECTION ==========

   drawDetectedObstacles(
       ObstacleMask,
       rawImage);

   // ========== FILTER OBSTACLES TO LANE ==========

   filterObstaclesToLane(
       XYZ_cam,
       filteredMask,
       whiteCurveParams,
       laneW);

   // Draw bounding boxes using the filtered mask
   drawDetectedObstacles(
       filteredMask,
       filteredImage_);

   // ========== DISPLAY BOTH RESULTS ==========

   cv::imshow("Detected Obstacles", rawImage);
   cv::imshow("Detected Obstacles in Lane", filteredImage_);
   cv::imshow("Raw Mask", ObstacleMask);
   cv::imshow("Filtered Mask", filteredMask);

   cv::waitKey(1);
}

else
{
   RCLCPP_WARN(this->get_logger(), "Road plane not detected for this frame.");
}



   */

            if (!whiteCurveParams.empty() && whiteCurveParams[0].isValid)
            {
                // Existing trajectory generation, visualization, and publishing code.

                // Refresh again so the successful frame shows the trajectory overlay.
                cv::imshow("Desired Lane Trajectory", image_bgr8);
                cv::waitKey(1);
            }
            else
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Lane fit rejected; displaying the camera frame without a trajectory.");
            }

            // ONLY draw if the polynomial was successfully solved or buffered
            if (whiteCurveParams[0].isValid)
            {
                DepthRange lane_depths = extractLaneDepthRange(white_line_pts, point_cloud_3D_);

                // DEBUG
                std::cout << "whiteCurveParams is valid" << std::endl;

                DesiredTrajectory new_trajectory = drawDesiredTrajectoryOnImage(
                    image_bgr8,
                    final_plane, // using alpha for low pass filter
                    laneW,       //  lane width: 1st iteration: hardcoded, then calibrated
                    camera_params_,
                    whiteCurveParams,
                    lane_depths.max_z); // <--- CLAMPS TO ACTUAL DETECTED DATA NOTE: this is note used..

                // ================= COLLECT RAW WHITE 3D POINTS =================
                std::vector<cv::Vec3f> raw_white_3d;
                for (const auto &pt : white_line_pts) // white_line_pts from your pixel extraction loop
                {
                    if (pt.x >= 0 && pt.x < point_cloud_3D_.cols && pt.y >= 0 && pt.y < point_cloud_3D_.rows)
                    {
                        cv::Vec3f p3d = point_cloud_3D_.at<cv::Vec3f>(pt.y, pt.x);
                        // Ensure valid metric depth
                        if (std::isfinite(p3d[2]) && p3d[2] > 0.0 && p3d[2] <= 25.0)
                        {
                            raw_white_3d.push_back(p3d);
                        }
                    }
                }

                // Pass the raw points into the BEV visualizer
                const bool second_poly_valid =
                    whiteCurveParams.size() > 1 &&
                    whiteCurveParams[1].isValid;

                BirdEyeViewVisualizer::draw(
                    new_trajectory,
                    whiteCurveParams[0].a2,
                    whiteCurveParams[0].a1,
                    whiteCurveParams[0].a0,
                    raw_white_3d,
                    lane_depths.max_z,
                    second_poly_valid,
                    second_poly_valid ? whiteCurveParams[1].a2 : 0.0,
                    second_poly_valid ? whiteCurveParams[1].a1 : 0.0,
                    second_poly_valid ? whiteCurveParams[1].a0 : 0.0);

                // ============= PERFOMING TRANSFROMATION (CAM2ENU) DUE TO CLOSELY REALATED TIMESTAMP OF SENSOR =======

                // ================= DIAGNOSTIC LOGGING =================

                if (found)
                {
                    const int64_t closest_ekf_time =
                        rclcpp::Time(closest_pose.header.stamp).nanoseconds();

                    // Positive means camera timestamp is ahead of EKF
                    const double signed_diff_ms =
                        static_cast<double>(cam_time - closest_ekf_time) / 1e6;

                    RCLCPP_INFO(
                        this->get_logger(),
                        "Camera: %.3f ms | "
                        "Closest EKF: %.3f ms | "
                        "Signed difference: %.3f ms | ",
                        static_cast<double>(cam_time) / 1e6,
                        static_cast<double>(closest_ekf_time) / 1e6,
                        signed_diff_ms);
                }
                else
                {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "EKF buffer is empty. No poses received.");
                }

                // ================= ORIGINAL TRANSFORMATION LOGIC =================

                // Maximum allowed camera-EKF timestamp difference: 20 ms
                if (ekf_matched)
                {
                    auto desired_enu = transformTrajectoryToENU(
                        new_trajectory,
                        closest_pose,
                        T_cam2body_);

                    if (!desired_enu.xyz_enu.empty())
                    {
                        publishDesiredTrajectoryENU(
                            desired_enu,
                            msg_left->header.stamp);
                    }
                }
                else
                {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "No matching EKF pose. Minimum difference: %.2f ms",
                        static_cast<double>(min_diff) / 1e6);
                }

                // VISUAL DEBUG: Display the rendered trajectory on screen
                cv::imshow("Desired Lane Trajectory", image_bgr8);
                cv::waitKey(1); // Refresh window TODO!
            }
            if (!road_found)
            {
                RCLCPP_WARN(this->get_logger(), "Road plane not detected. Skipping trajectory projection for this frame to prevent jitter.");
            }
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "cv_bridge error: %s",
                e.what());
        }

        const auto callback_end = std::chrono::steady_clock::now();

        const double processing_ms =
            std::chrono::duration<double, std::milli>(
                callback_end - callback_start)
                .count();

        RCLCPP_INFO(
            this->get_logger(),
            "Camera processing duration: %.2f ms",
            processing_ms);
    }

    message_filters::Subscriber<sensor_msgs::msg::Image> sub_left_;  // left image subsripton
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_right_; // right image subscirpton

    using ApproxSyncPolicy_ = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,
        sensor_msgs::msg::Image>;
    std::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy_>> sync_;

    /*
    using ExactSyncPolicy = message_filters::sync_policies::ExactTime<
        sensor_msgs::msg::Image,
        sensor_msgs::msg::Image>;
    std::shared_ptr<message_filters::Synchronizer<ExactSyncPolicy>> sync_;
*/
    cv::Ptr<cv::StereoSGBM> stereo_;

    cv::Mat image_bgr8; // if i dont denote subscript i corresponds to left image. Left Camere is refenrece in computation logics
    cv::Mat image_bgr8R_;
    cv::Mat point_cloud_3D_;
    cv::Mat white_line_imgR;
    cv::Mat white_line_imgL;
    CameraParams camera_params_;
    CamType::Transform4x4_Cam2Body T_cam2body_;
    double laneW = 12;
    double Y_ground = 1.05; // hardcoded, but in real scenrio it depensd how i mount tha camera onto the vehicle

    // ================== CATCHED TRAJECTORY =========================
    std::vector<LanePolynomial> whiteCurveParams_prev; //  buffer for lastly valid polyfitting datas

    // ================================================

    // =========================== CALLBACK GROUPS ============================
    // Callback groups
    rclcpp::CallbackGroup::SharedPtr camera_callback_group_;
    rclcpp::CallbackGroup::SharedPtr ekf_callback_group_;

    // ==================== SYNC DETECTED TRAJECTORY DETERMINED HZ BY CAM STREAM AND EKF POSE ========================
    std::deque<geometry_msgs::msg::PoseStamped> ekf_buffer_;
    std::mutex ekf_mutex_;

    // ======================== SUBSCRIBER FOR TRANMITTED POSE BY EKF ========================
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ekf_pose_sub_;
    // tranmitted latest avaible ekf data

    // ======================== PUBLISHER FOR DESIREDTRAJECTORY_ENU ========================

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        desired_trajectory_pub_;

    void publishDesiredTrajectoryENU(
        const DesiredTrajectoryENU &trajectory,
        const builtin_interfaces::msg::Time &camera_stamp)
    {
        if (trajectory.uv_pixels.size() != trajectory.xyz_enu.size())
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Trajectory pixel/ENU size mismatch");
            return;
        }

        sensor_msgs::msg::PointCloud2 msg;

        msg.header.stamp = camera_stamp;
        msg.header.frame_id = "enu";
        msg.height = 1;
        msg.is_dense = true;

        sensor_msgs::PointCloud2Modifier modifier(msg);

        modifier.setPointCloud2Fields(
            5,
            "u", 1, sensor_msgs::msg::PointField::INT32,
            "v", 1, sensor_msgs::msg::PointField::INT32,
            "x", 1, sensor_msgs::msg::PointField::FLOAT64,
            "y", 1, sensor_msgs::msg::PointField::FLOAT64,
            "z", 1, sensor_msgs::msg::PointField::FLOAT64);

        modifier.resize(trajectory.xyz_enu.size());

        sensor_msgs::PointCloud2Iterator<int32_t> u(msg, "u");
        sensor_msgs::PointCloud2Iterator<int32_t> v(msg, "v");

        sensor_msgs::PointCloud2Iterator<double> x(msg, "x");
        sensor_msgs::PointCloud2Iterator<double> y(msg, "y");
        sensor_msgs::PointCloud2Iterator<double> z(msg, "z");

        for (size_t i = 0; i < trajectory.xyz_enu.size(); ++i)
        {
            *u = trajectory.uv_pixels[i].x;
            *v = trajectory.uv_pixels[i].y;

            *x = trajectory.xyz_enu[i][0];
            *y = trajectory.xyz_enu[i][1];
            *z = trajectory.xyz_enu[i][2];

            ++u;
            ++v;
            ++x;
            ++y;
            ++z;
        }

        desired_trajectory_pub_->publish(msg);
    }

    void write2EKFbuff(
        const geometry_msgs::msg::PoseStamped &msg)
    {

        std::lock_guard<std::mutex> lock(ekf_mutex_);

        ekf_buffer_.push_back(msg);

        // Keep maximum 30 poses (~1 second at 30 Hz)
        if (ekf_buffer_.size() > MAX_EKF_BUFFER_SIZE)
        {
            ekf_buffer_.pop_front();
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CameraSubscriber>();

    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(),
        4);

    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}