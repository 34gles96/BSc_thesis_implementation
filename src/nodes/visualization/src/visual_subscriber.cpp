
#include <memory>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/opencv.hpp>

#include <stdexcept>
#include <yaml-cpp/yaml.h>

#include <ctime>
#include <cstdlib>

constexpr int fps = 1;

class ImageSubscriber : public rclcpp::Node
{
public:
    ImageSubscriber()
        : Node("visualization_subscriber")
    {

        // =========================================
        // 0. INITIALIZATION OF VIDEO MAKER
        // =========================================

        // =========================================
        // CREATE RECORDING FOLDER WITH CURRENT DATE
        // =========================================

        std::time_t now = std::time(nullptr);
        std::tm *local_time = std::localtime(&now);

        char date[11];
        std::strftime(date, sizeof(date), "%Y-%m-%d", local_time);

        recording_folder_ = "/ADC_27/" + std::string(date);

        std::string command = "mkdir -p \"" + recording_folder_ + "\"";

        if (std::system(command.c_str()) != 0)
        {
            throw std::runtime_error("Failed to create recording folder!");
        }

        YAML::Node config = YAML::LoadFile("/home/base/ADC_27/configData/cam_config/CameraConfig.yaml");

        if (!config["image_width"] || !config["image_height"])
        {
            throw std::runtime_error("Invalid YAML format: Missing 'image width' or 'height' node.");
        }

        const int image_width = config["image_width"].as<int>();
        const int image_height = config["image_height"].as<int>();
        const cv::Size imgRes(image_width, image_height);

        birdview_writer_.open(
            recording_folder_ + "/birdview.avi",
            cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
            fps,
            imgRes,
            true);

        white_pixels_writer_.open(
            recording_folder_ + "/white_pixels.avi",
            cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
            fps,
            imgRes,
            false);

        trajectory_writer_.open(
            recording_folder_ + "/trajectory.avi",
            cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
            fps,
            imgRes,
            true);

        if (!birdview_writer_.isOpened() ||
            !white_pixels_writer_.isOpened() ||
            !trajectory_writer_.isOpened())
        {
            throw std::runtime_error("Failed to initialize one or more video writers!");
        }

        // =========================================
        // 1. BIRD'S-EYE VIEW IMAGE SUBSCRIBER
        // =========================================

        birdview_sub_ =
            this->create_subscription<sensor_msgs::msg::Image>(
                "/adas/debug/birdview_image",
                rclcpp::QoS(1),
                std::bind(
                    &ImageSubscriber::birdview_callback,
                    this,
                    std::placeholders::_1));

        // =========================================
        // 2. EXTRACTED WHITE PIXELS SUBSCRIBER
        // =========================================

        white_pixels_sub_ =
            this->create_subscription<sensor_msgs::msg::Image>(
                "/adas/debug/white_pixels_image",
                rclcpp::QoS(1),
                std::bind(
                    &ImageSubscriber::white_pixels_callback,
                    this,
                    std::placeholders::_1));

        // =========================================
        // 3. DESIRED TRAJECTORY IMAGE SUBSCRIBER
        // =========================================

        desired_trajectory_sub_ =
            this->create_subscription<sensor_msgs::msg::Image>(
                "/adas/debug/desired_trajectory_image",
                rclcpp::QoS(1),
                std::bind(
                    &ImageSubscriber::desired_trajectory_callback,
                    this,
                    std::placeholders::_1));

        // =========================================
        // CREATE DISPLAY WINDOWS
        // =========================================

        cv::namedWindow(
            "Bird's-Eye View",
            cv::WINDOW_NORMAL);

        cv::namedWindow(
            "Extracted White Pixels",
            cv::WINDOW_NORMAL);

        cv::namedWindow(
            "Desired Trajectory",
            cv::WINDOW_NORMAL);

        RCLCPP_INFO(
            this->get_logger(),
            "Three image subscribers initialized.");
    }

    ~ImageSubscriber()
    {
        birdview_writer_.release();
        white_pixels_writer_.release();
        trajectory_writer_.release();

        cv::destroyAllWindows();
    }

private:
    // =========================================
    // IMAGE 1: BIRD'S-EYE VIEW
    // =========================================

    void birdview_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        auto cv_image = cv_bridge::toCvCopy(
            msg,
            sensor_msgs::image_encodings::BGR8);

        // Initialize video writer using the first received image
        if (!birdview_initialized_)
        {
            cv::Size imgRes(
                cv_image->image.cols,
                cv_image->image.rows);

            birdview_writer_.open(
                recording_folder_ + "/birdview.avi",
                cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                fps,
                imgRes,
                true);

            if (!birdview_writer_.isOpened())
            {
                throw std::runtime_error("Failed to initialize birdview video writer!");
            }

            birdview_initialized_ = true;
        }

        show_image(
            cv_image->image,
            "Bird's-Eye View");

        birdview_writer_.write(cv_image->image);
    }

    // =========================================
    // IMAGE 2: EXTRACTED WHITE PIXELS
    // =========================================

    void white_pixels_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        auto cv_image = cv_bridge::toCvCopy(
            msg,
            sensor_msgs::image_encodings::MONO8);

        show_image(
            cv_image->image,
            "Extracted White Pixels");

        white_pixels_writer_.write(cv_image->image);
    }

    // =========================================
    // IMAGE 3: DESIRED TRAJECTORY
    // =========================================

    void desired_trajectory_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        auto cv_image = cv_bridge::toCvCopy(
            msg,
            sensor_msgs::image_encodings::BGR8);

        show_image(
            cv_image->image,
            "Desired Trajectory");

        trajectory_writer_.write(cv_image->image);
    }

    // =========================================
    // COMMON IMAGE DISPLAY FUNCTION
    // =========================================

    void show_image(
        const cv::Mat &image,
        const std::string &window_name)
    {
        cv::imshow(window_name, image);
        cv::waitKey(1);
    }

    // =========================================
    // SUBSCRIBER MEMBERS
    // =========================================

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr birdview_sub_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr white_pixels_sub_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr desired_trajectory_sub_;

    // =========================================
    // VIDEOWRITERS MEMBERS
    // =========================================

    cv::VideoWriter birdview_writer_;
    cv::VideoWriter white_pixels_writer_;
    cv::VideoWriter trajectory_writer_;

    std::string recording_folder_;

    bool birdview_initialized_ = false;
};

// =========================================
// MAIN
// =========================================

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<ImageSubscriber>();

    rclcpp::spin(node);

    cv::destroyAllWindows();

    rclcpp::shutdown();

    return 0;
}
