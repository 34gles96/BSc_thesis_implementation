#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
//============ include from MATLAB coder lib ================

// ===============================================
// New includes for synchronization
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include "readConfig.hpp"
#include "fitLanePoly.hpp"
#include "desiredTrajectory.hpp"
#include "pinholeBackProjection.hpp"

class CameraSubscriber : public rclcpp::Node
{
public:
    CameraSubscriber()
        : Node("camera_subscriber") // initialization list for parent class
    {

        camera_params_ = loadCameraParams("/workspace/data/CameraConfig.yaml");

        cv::Size img_size(camera_params_.width, camera_params_.height);
        point_cloud_3D_ = cv::Mat(img_size, CV_32FC3, cv::Scalar(0.0f, 0.0f, 0.0f)); // initialization of pointcloud:size:u x v, one element memlayout : CV_32FC3

        // subscripton for chanel of left img stream
        sub_left_.subscribe(this, "/camera/imageLeft");

        // subscripton for chanel of right img stream
        sub_right_.subscribe(this, "/camera/imageRight");

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), sub_left_, sub_right_);

        sync_->registerCallback(std::bind(
            &CameraSubscriber::stereo_callback, this,
            std::placeholders::_1, std::placeholders::_2));

        int minDisparity = 0;
        int numDisparities = 16 * 10; // 160 disparities
        int blockSize = 15;           // INCREASED: A large block size (15x15) grabs more road texture
        stereo_ = cv::StereoSGBM::create(
            0,
            16 * 8,
            5,
            8 * 3 * 5 * 5,
            32 * 3 * 5 * 5,
            1,
            63,
            10,
            50,
            2,
            cv::StereoSGBM::MODE_SGBM_3WAY);
    }

    // ================================================================
    // Members and internal functions due to event of subscripton
    // ================================================================
private:
    void stereo_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr msg_left,
        const sensor_msgs::msg::Image::ConstSharedPtr msg_right)
    {
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

            // transfroming the image into hsv color space
            cv::Mat hsv;
            cv::cvtColor(image_bgr8, hsv, cv::COLOR_BGR2HSV);

            // Split HSV into 3 separate single-channel images
            std::vector<cv::Mat> hsv_channels;
            cv::split(hsv, hsv_channels);

            // 3. Apply CLAHE to the V (Value/Brightness) channel
            cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
            clahe->apply(hsv_channels[2], hsv_channels[2]);

            // 4. Merge channels back into an HSV image
            cv::merge(hsv_channels, hsv);

            // Define the HSV threshold range for WHITE
            // HSV ranges in OpenCV: H: 0-180, S: 0-255, V: 0-255
            // White = any color (H: 0-180), very low saturation (S: 0-50), very high brightness (V: 200-255)
            cv::Scalar lower_white(0, 0, 230);
            cv::Scalar upper_white(180, 50, 255);

            //  Create a binary mask (white lines become pure white 255, everything else black 0)
            cv::Mat mask;
            cv::inRange(hsv, lower_white, upper_white, mask); // pixels in range of (lower_white, upper_hwhite) -> white, else black. stored: mask

            // defining region of interest (lower part of the streamed screen (ignoring tree s)).
            cv::Rect roi_area(0, mask.rows / 2, mask.cols, mask.rows / 2);
            cv::Mat mask_roi = mask(roi_area); // extracting the ROI part from mask

            // Extracting the (u, v ) pixel coordinates of the white line
            std::vector<cv::Point> white_line_pts;

            // Scan the masked region
            for (int y = 0; y < mask_roi.rows; ++y)
            {
                for (int x = 0; x < mask_roi.cols; ++x)
                {
                    // If the pixel is white (255)
                    if (mask_roi.at<uchar>(y, x) == 255)
                    {
                        // Adjust Y back to the original image ( captured input camera stream)) coordinate frame
                        int original_y = y + (image_bgr8.rows / 2);
                        white_line_pts.push_back(cv::Point(x, original_y));
                    }
                }
            }

            // Print out the total number of line points found
            std::cout << "Extracted " << white_line_pts.size() << " white lane pixels from Left Camer." << std::endl;

            // trasnroming the extracted (u,v) white coorinate into XYZ (camera framte)
            // creating a black img w intialization
            white_line_imgL = cv::Mat::zeros(image_bgr8.size(), CV_8UC1);

            // safity check
            if (white_line_imgL.size() != point_cloud_3D_.size())
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Mask and point cloud dimensions do not match");
                return;
            }

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

            // Optional: Visual debug windows
            // cv::imshow("Original Frame", image_bgr8);
            // cv::imshow("White Lane Mask", mask);
            // cv::waitKey(0);

            // TRANSFORMING  (u,v) w coorinate into XYZ (camera framte)
            point_cloud_3D_ = pinHole2XYZ(image_bgr8, image_bgr8R_, stereo_, camera_params_, point_cloud_3D_);

            struct LanePolynomial whiteCurveParams = fitLanePolynomialOpenCV(white_line_imgL, point_cloud_3D_, laneW);

            // ONLY draw if the polynomial was successfully solved
            if (whiteCurveParams.isValid)
            {
                // DEBUG
                std::cout << "whiteCurveParams is valid" << std::endl;
                drawDesiredTrajectoryOnImage(
                    image_bgr8,
                    whiteCurveParams.a2, whiteCurveParams.a1, whiteCurveParams.a0,
                    Y_ground,
                    laneW,                         //  lane width: 1st iteration: hardcoded, then calibrated
                    whiteCurveParams.is_left_lane, // Automatically determined!
                    camera_params_,
                    whiteCurveParams);

                // VISUAL DEBUG: Display the rendered trajectory on screen
                cv::imshow("Desired Lane Trajectory", image_bgr8);
                cv::waitKey(1); // Refresh window
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Lane polynomial fit is invalid for this frame.");
            }
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "cv_bridge error: %s",
                e.what());
        }
    }

    message_filters::Subscriber<sensor_msgs::msg::Image> sub_left_;  // left image subsripton
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_right_; // right image subscirpton

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    cv::Ptr<cv::StereoSGBM> stereo_;

    cv::Mat image_bgr8; // if i dont denote subscript i corresponds to left image. Left Camere is refenrece in computation logics
    cv::Mat image_bgr8R_;
    cv::Mat point_cloud_3D_;
    cv::Mat white_line_imgR;
    cv::Mat white_line_imgL;
    CameraParams camera_params_;
    double laneW = 3.5;
    double Y_ground = 1.65; // hardcoded, but in real scenrio it depensd how i mount tha camera onto the vehicle
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    //  Initialize MATLAB generated model states & persistent variables

    auto node = std::make_shared<CameraSubscriber>();

    rclcpp::spin(node);

    // Terminate/clean up MATLAB model resources safely

    rclcpp::shutdown();

    return 0;
}