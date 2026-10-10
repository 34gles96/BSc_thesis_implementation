#include "pinholeBackProjection.hpp"

cv::Mat &pinHole2XYZ(const cv::Mat &image_bgr8L, const cv::Mat &image_bgr8R, cv::Ptr<cv::StereoSGBM> stereo_, struct CameraParams config, cv::Mat &pointCloud)
{

    // extracting config datas of camera
    double fx_ = config.fx;
    double fy_ = config.fy;
    double baseline_ = config.baselink;
    double cx_ = config.cx; // focal center of the left camare
    double cy_ = config.cy;

    // ========================= DEBUG ==================================================
    /*
    std::cout << "fx: " << config.fx << std::endl;
    std::cout << "fy: " << config.fy << std::endl;
    std::cout << "cx: " << config.cx << std::endl;
    std::cout << "cy: " << config.cy << std::endl;
    std::cout << "baseline: " << config.baselink << std::endl;
    */
    // ====================================================================================================

    //  Reset pointcloud to 0 before populating new frame
    pointCloud = cv::Mat::zeros(image_bgr8L.size(), CV_32FC3);

    // ============================================================
    //  Compute disparity
    // ============================================================

    // Ensure input images are 1-channel grayscale (CV_8UC1) for StereoSGBM
    cv::Mat grayL, grayR;
    if (image_bgr8L.channels() == 3)
    {
        cv::cvtColor(image_bgr8L, grayL, cv::COLOR_BGR2GRAY);
    }
    else
    {
        grayL = image_bgr8L;
    }

    if (image_bgr8R.channels() == 3)
    {
        cv::cvtColor(image_bgr8R, grayR, cv::COLOR_BGR2GRAY);
    }
    else
    {
        grayR = image_bgr8R;
    }

    // Guard check: ensure dimensions match before passing to StereoSGBM
    if (grayL.empty() || grayR.empty() || grayL.size() != grayR.size())
    {
        std::cerr << "Stereo Warning: Image size mismatch or empty frame. Left: "
                  << grayL.cols << "x" << grayL.rows
                  << ", Right: " << grayR.cols << "x" << grayR.rows << std::endl;
        return pointCloud;
    }

    cv::Mat disparity_16;

    stereo_->compute(
        grayL,
        grayR,
        disparity_16);

    // OpenCV SGBM stores disparity multiplied by 16. aslo: extending the mem layout to 32bit floating

    cv::Mat disparity;

    disparity_16.convertTo(
        disparity,
        CV_32F,
        1.0 / 16.0);

    double minDisp, maxDisp;
    cv::minMaxLoc(disparity, &minDisp, &maxDisp);

    int validDisparities = cv::countNonZero(disparity > 0.0f);

    std::cout << "Disparity range: "
              << minDisp << " to " << maxDisp << std::endl;

    std::cout << "Valid disparity pixels: "
              << validDisparities << std::endl;

    /*
    // ============================================================
    // DEBUG: VISUALIZE DISPARITY MAP
    // ============================================================
    cv::Mat disp_vis;
    // Normalize the depth values to 0-255 for viewing
    cv::normalize(disparity, disp_vis, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::imshow("Stereo Disparity (Depth)", disp_vis);
    cv::waitKey(1); // Tells OpenCV to refresh the window

        */

    // ============================================================
    // Compute depth
    // ============================================================

    // Z = fx * B / disparity.      (triangular math)

    cv::Mat depth( // declaration of depth matrix
        disparity.size(),
        CV_32F);

    for (int v = 0; v < disparity.rows; ++v)
    {
        for (int u = 0; u < disparity.cols; ++u)
        {
            float d = disparity.at<float>(v, u);

            if (d > 0.0f)
            {
                depth.at<float>(v, u) =
                    fx_ * baseline_ / d;
            }
            else
            {
                depth.at<float>(v, u) = 0.0f;
            }
        }
    }

    // mapping (u,v) -> XYZ

    for (int v = 0; v < depth.rows; v++)
    {
        for (int u = 0; u < depth.cols; u++)
        {

            float Z = depth.at<float>(v, u);

            // Ignore invalid / very distant points
            if (Z <= 0.0f || Z > 30.0f)
            {
                pointCloud.at<cv::Vec3f>(v, u) = cv::Vec3f(0.0f, 0.0f, 0.0f);
                continue;
            }
            // pinhole model transfromation: (u,v) -> (X,Y) due to geometry

            float X = (u - cx_) * Z / fx_;
            float Y = (v - cy_) * Z / fy_;
            pointCloud.at<cv::Vec3f>(v, u) = cv::Vec3f(X, Y, Z);
        }
    }

    return pointCloud;
}