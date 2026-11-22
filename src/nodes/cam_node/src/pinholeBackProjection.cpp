
#include "pinholeBackProjection.hpp"

#include <iostream>
#include <chrono>

#include "config.hpp"

/*
cv::Mat &pinHole2XYZ(
    const cv::Mat &image_bgr8L,
    const cv::Mat &image_bgr8R,
    cv::Ptr<cv::StereoSGBM> stereo_,
    struct CameraParams config,
    cv::Mat &pointCloud)
{

    using Clock = std::chrono::steady_clock;

    const auto t0 = Clock::now();

    // Extract camera configuration
    const double fx_ = config.fx;
    const double fy_ = config.fy;
    const double baseline_ = config.baselink;
    const double cx_ = config.cx;
    const double cy_ = config.cy;

    // ============================================================
    // Convert images to grayscale
    // ============================================================

    cv::Mat grayL, grayR;

    if (image_bgr8L.channels() == 3)
    {
        cv::cvtColor(
            image_bgr8L,
            grayL,
            cv::COLOR_BGR2GRAY);
    }
    else
    {
        grayL = image_bgr8L;
    }

    if (image_bgr8R.channels() == 3)
    {
        cv::cvtColor(
            image_bgr8R,
            grayR,
            cv::COLOR_BGR2GRAY);
    }
    else
    {
        grayR = image_bgr8R;
    }

    const auto t1 = Clock::now();

    // ============================================================
    // Validate input images
    // ============================================================

    if (grayL.empty() ||
        grayR.empty() ||
        grayL.size() != grayR.size())
    {
        std::cerr
            << "Stereo Warning: Image size mismatch or empty frame. "
            << "Left: " << grayL.cols << "x" << grayL.rows
            << ", Right: " << grayR.cols << "x" << grayR.rows
            << std::endl;

        // Return an initialized empty point cloud for this frame.
        pointCloud.create(image_bgr8L.size(), CV_32FC3);
        pointCloud.setTo(cv::Scalar::all(0));

        return pointCloud;
    }

    // ============================================================
    // Compute disparity
    // ============================================================

    cv::Mat disparity_16;

    stereo_->compute(
        grayL,
        grayR,
        disparity_16);

    const auto t2 = Clock::now();
    // OpenCV StereoSGBM stores disparity multiplied by 16.
    // Convert to floating-point disparity values.

    cv::Mat disparity;

    disparity_16.convertTo(
        disparity,
        CV_32F,
        1.0 / 16.0);

    const auto t3 = Clock::now();

    // ============================================================
    // DEBUG: Disparity statistics
    // ============================================================

    double minDisp, maxDisp;

    cv::minMaxLoc(
        disparity,
        &minDisp,
        &maxDisp);

    int validDisparities =
        cv::countNonZero(disparity > 0.0f);

    std::cout
        << "Disparity range: "
        << minDisp << " to " << maxDisp
        << std::endl;

    std::cout
        << "Valid disparity pixels: "
        << validDisparities
        << std::endl;

    const auto t4 = Clock::now();

    // ============================================================
    // Calculate XYZ point cloud directly from disparity
    // ============================================================

    // Convert camera parameters to float to avoid unnecessary
    // double-precision arithmetic in the pixel-processing loop.

    const float fx = static_cast<float>(fx_);
    const float fy = static_cast<float>(fy_);
    const float cx = static_cast<float>(cx_);
    const float cy = static_cast<float>(cy_);
    const float baseline = static_cast<float>(baseline_);

    const float fx_baseline = fx * baseline;
    const float inv_fy = 1.0f / fy;

    // Allocate output storage without initializing every element.
    // Every pixel is explicitly assigned below.

    pointCloud.create(
        disparity.rows,
        disparity.cols,
        CV_32FC3);

    for (int v = 0; v < disparity.rows; ++v)
    {
        // Get pointers to the current rows.
        // This avoids repeated Mat::at() calls inside the loop.

        const float *disparity_row =
            disparity.ptr<float>(v);

        cv::Vec3f *cloud_row =
            pointCloud.ptr<cv::Vec3f>(v);

        // Precalculate the vertical factor for this row.

        const float y_factor =
            (static_cast<float>(v) - cy) * inv_fy;

        for (int u = 0; u < disparity.cols; ++u)
        {
            const float d = disparity_row[u];

            // Ignore invalid disparity values.

            if (d <= 0.0f)
            {
                cloud_row[u] =
                    cv::Vec3f(0.0f, 0.0f, 0.0f);

                continue;
            }

            // Calculate depth:
            //
            // Z = fx * baseline / disparity

            const float Z = fx_baseline / d;

            // Ignore invalid or very distant points.

            if (Z > 30.0f)
            {
                cloud_row[u] =
                    cv::Vec3f(0.0f, 0.0f, 0.0f);

                continue;
            }

            // Back-project pixel coordinates into 3D:
            //
            // X = (u - cx) * baseline / disparity
            // Y = (v - cy) * Z / fy

            const float X =
                (static_cast<float>(u) - cx) * baseline / d;

            const float Y = y_factor * Z;

            // Store 3D point.

            cloud_row[u] =
                cv::Vec3f(X, Y, Z);
        }
    }

    const auto t5 = Clock::now();

    const auto ms = [](Clock::time_point start, Clock::time_point end)
    {
        return std::chrono::duration<double, std::milli>(
                   end - start)
            .count();
    };

    std::cout << "[pinHole2XYZ timing] "
              << "Grayscale: " << ms(t0, t1) << " ms | "
              << "SGBM: " << ms(t1, t2) << " ms | "
              << "Disparity conversion: " << ms(t2, t3) << " ms | "
              << "Statistics: " << ms(t3, t4) << " ms | "
              << "XYZ reconstruction: " << ms(t4, t5) << " ms | "
              << "Total: " << ms(t0, t5) << " ms"
              << std::endl;

    return pointCloud;
}
*/

#include "pinholeBackProjection.hpp"

#include <iostream>
#include <chrono>
#include <cmath>

cv::Mat &pinHole2XYZ(
    const cv::Mat &image_bgr8L,
    const cv::Mat &image_bgr8R,
    cv::Ptr<cv::StereoSGBM> stereo_,
    struct CameraParams cameraConfig,
    cv::Mat &pointCloud)
{
    using Clock = std::chrono::steady_clock;

    const auto t0 = Clock::now();

    // ============================================================
    // Extract camera configuration
    // ============================================================

    const float fx = static_cast<float>(cameraConfig.fx);
    const float fy = static_cast<float>(cameraConfig.fy);
    const float baseline = static_cast<float>(cameraConfig.baselink);
    const float cx = static_cast<float>(cameraConfig.cx);
    const float cy = static_cast<float>(cameraConfig.cy);

    const float fx_baseline = fx * baseline;
    const float inv_fy = 1.0f / fy;

    // ============================================================
    // Validate input images
    // ============================================================

    if (image_bgr8L.empty() ||
        image_bgr8R.empty() ||
        image_bgr8L.size() != image_bgr8R.size())
    {
        std::cerr
            << "Stereo Warning: Image size mismatch or empty frame."
            << std::endl;

        pointCloud = cv::Mat::zeros(
            image_bgr8L.size(),
            CV_32FC3);

        return pointCloud;
    }

    if (stereo_.empty())
    {
        std::cerr << "Stereo Warning: StereoSGBM is not initialized."
                  << std::endl;

        pointCloud = cv::Mat::zeros(
            image_bgr8L.size(),
            CV_32FC3);

        return pointCloud;
    }

    // ============================================================
    // Define vertical Region of Interest (ROI)
    // ============================================================

    // Configuration values are normalized image-height coordinates:
    //
    // Example:
    // img_in_stream_vertical_cut_top    = 0.30
    // img_in_stream_vertical_cut_bottom = 1.00
    //
    // This processes the bottom 70% of the image.

    const int roi_top = static_cast<int>(
        image_bgr8L.rows *
        ::config::img_in_stream_vertical_cut_top);

    const int roi_bottom = static_cast<int>(
        image_bgr8L.rows *
        ::config::img_in_stream_vertical_cut_bottom);

    // Validate ROI boundaries.

    if (roi_top < 0 ||
        roi_bottom > image_bgr8L.rows ||
        roi_bottom <= roi_top)
    {
        std::cerr << "Stereo Warning: Invalid vertical ROI boundaries. "
                  << "Top: " << roi_top
                  << ", Bottom: " << roi_bottom
                  << ", Image height: " << image_bgr8L.rows
                  << std::endl;

        pointCloud = cv::Mat::zeros(
            image_bgr8L.size(),
            CV_32FC3);

        return pointCloud;
    }

    // Keep the entire horizontal image width.
    const cv::Rect roi_area(
        0,
        roi_top,
        image_bgr8L.cols,
        roi_bottom - roi_top);

    // Extract identical vertical ROIs from both stereo images.
    // These are OpenCV matrix views, not full-image copies.

    const cv::Mat leftROI = image_bgr8L(roi_area);
    const cv::Mat rightROI = image_bgr8R(roi_area);

    // ============================================================
    // Convert cropped images to grayscale
    // ============================================================

    cv::Mat grayL, grayR;

    if (leftROI.channels() == 3)
    {
        cv::cvtColor(
            leftROI,
            grayL,
            cv::COLOR_BGR2GRAY);
    }
    else if (leftROI.channels() == 1)
    {
        grayL = leftROI;
    }
    else
    {
        std::cerr << "Stereo Warning: Unsupported left image channels."
                  << std::endl;

        pointCloud = cv::Mat::zeros(
            image_bgr8L.size(),
            CV_32FC3);

        return pointCloud;
    }

    if (rightROI.channels() == 3)
    {
        cv::cvtColor(
            rightROI,
            grayR,
            cv::COLOR_BGR2GRAY);
    }
    else if (rightROI.channels() == 1)
    {
        grayR = rightROI;
    }
    else
    {
        std::cerr << "Stereo Warning: Unsupported right image channels."
                  << std::endl;

        pointCloud = cv::Mat::zeros(
            image_bgr8L.size(),
            CV_32FC3);

        return pointCloud;
    }

    if (grayL.type() != CV_8UC1 ||
        grayR.type() != CV_8UC1)
    {
        std::cerr << "Stereo Warning: SGBM requires CV_8UC1 images."
                  << std::endl;

        pointCloud = cv::Mat::zeros(
            image_bgr8L.size(),
            CV_32FC3);

        return pointCloud;
    }

    const auto t1 = Clock::now();

    // ============================================================
    // Compute disparity ONLY inside the ROI
    // ============================================================

    cv::Mat disparity_16;

    stereo_->compute(
        grayL,
        grayR,
        disparity_16);

    const auto t2 = Clock::now();

    // OpenCV StereoSGBM stores disparity multiplied by 16.
    // Convert to floating-point disparity values.

    cv::Mat disparityROI;

    disparity_16.convertTo(
        disparityROI,
        CV_32F,
        1.0 / 16.0);

    const auto t3 = Clock::now();

    // ============================================================
    // DEBUG: Disparity statistics (ROI only)
    // ============================================================

    double minDisp, maxDisp;

    cv::minMaxLoc(
        disparityROI,
        &minDisp,
        &maxDisp);

    const int validDisparities =
        cv::countNonZero(disparityROI > 0.0f);

    std::cout
        << "Disparity range (ROI): "
        << minDisp << " to " << maxDisp
        << std::endl;

    std::cout
        << "Valid disparity pixels (ROI): "
        << validDisparities
        << std::endl;

    const auto t4 = Clock::now();

    // ============================================================
    // Initialize FULL-resolution point cloud
    // ============================================================

    // Pixels outside the ROI must remain invalid (0, 0, 0).
    //
    // The point cloud retains the original image dimensions.

    pointCloud.create(
        image_bgr8L.rows,
        image_bgr8L.cols,
        CV_32FC3);

    pointCloud.setTo(cv::Scalar::all(0));

    const auto t5 = Clock::now();

    // ============================================================
    // Calculate XYZ ONLY inside the ROI
    // ============================================================

    for (int v_roi = 0; v_roi < disparityROI.rows; ++v_roi)
    {
        // Convert ROI-local row coordinate to original image row.

        const int v = v_roi + roi_area.y;

        // Access the corresponding disparity and point-cloud rows.

        const float *disparity_row =
            disparityROI.ptr<float>(v_roi);

        cv::Vec3f *cloud_row =
            pointCloud.ptr<cv::Vec3f>(v);

        // Calculate vertical factor using ORIGINAL image coordinates.

        const float y_factor =
            (static_cast<float>(v) - cy) * inv_fy;

        for (int u_roi = 0; u_roi < disparityROI.cols; ++u_roi)
        {
            // Convert ROI-local column coordinate to original image column.

            const int u = u_roi + roi_area.x;

            const float d = disparity_row[u_roi];

            // Ignore invalid disparity values.

            if (d <= 0.0f)
                continue;

            // Calculate depth:
            //
            // Z = fx * baseline / disparity

            const float Z = fx_baseline / d;

            // Ignore invalid or very distant points.

            if (Z > 30.0f)
                continue;

            // Back-project using ORIGINAL image coordinates:
            //
            // X = (u - cx) * baseline / disparity
            // Y = (v - cy) * Z / fy

            const float X =
                (static_cast<float>(u) - cx) * baseline / d;

            const float Y = y_factor * Z;

            // Store 3D point in its ORIGINAL image position.

            cloud_row[u] = cv::Vec3f(X, Y, Z);
        }
    }

    const auto t6 = Clock::now();

    // ============================================================
    // Execution-time measurements
    // ============================================================

    const auto ms = [](Clock::time_point start, Clock::time_point end)
    {
        return std::chrono::duration<double, std::milli>(
                   end - start)
            .count();
    };

    std::cout << "[pinHole2XYZ timing] "
              << "ROI + grayscale: " << ms(t0, t1) << " ms | "
              << "SGBM: " << ms(t1, t2) << " ms | "
              << "Disparity conversion: " << ms(t2, t3) << " ms | "
              << "Statistics: " << ms(t3, t4) << " ms | "
              << "Point-cloud initialization: " << ms(t4, t5) << " ms | "
              << "XYZ reconstruction: " << ms(t5, t6) << " ms | "
              << "Total: " << ms(t0, t6) << " ms"
              << std::endl;

    return pointCloud;
}
