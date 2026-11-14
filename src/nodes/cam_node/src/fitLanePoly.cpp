#include "fitLanePoly.hpp"
#include "road_boundary_fitting.hpp"
#include "config.hpp"
#include "roadLineValidator.hpp"

#include <cmath>
#include <iostream>

const double minDepth = 0.1;
const double maxDepth = 30.0;
const double binSize = 0.5;

LanePolynomial fitLanePolynomialOpenCV(
    const cv::Mat &uvIMG, const cv::Mat &XYZ_IMG, double &road_width) // first arg correspond to white images, 2nd is cv::Mat<cv::Vec3f>(u,v) = (X,Y,Z) mapping, 3rd arg is modifiyng road_witdh
{

    // region of interest for detected left or right lane using for desired trsajectory
    // Vertical ROI limits:
    // ignore upper part of image and bottom part containing vehicle hood
    int roi_top =
        static_cast<int>(uvIMG.rows * config::img_in_stream_vertical_cut_top);

    int roi_bottom =
        static_cast<int>(uvIMG.rows * config::img_in_stream_vertical_cut_bottom);

    int roi_height =
        roi_bottom - roi_top;

    // LEFT half of image
    cv::Rect roi_area_left(
        0,              // x
        roi_top,        // y
        uvIMG.cols / 2, // width
        roi_height      // height
    );

    cv::Mat mask_roi_left =
        uvIMG(roi_area_left);

    // RIGHT half of image
    cv::Rect roi_area_right(
        uvIMG.cols / 2,              // x
        roi_top,                     // y
        uvIMG.cols - uvIMG.cols / 2, // width
        roi_height                   // height
    );

    cv::Mat mask_roi_right =
        uvIMG(roi_area_right);

    // Extracting the (u, v ) pixel coordinates of the white line
    std::vector<cv::Point> white_line_ptsL; // 2D stream of point of (u,v)
    std::vector<cv::Point> white_line_ptsR;
    std::size_t nb_whitePX_left, nb_whitePX_right;

    // Scan the masked region for LEFT PART
    for (int y = 0; y < mask_roi_left.rows; ++y)
    {
        for (int x = 0; x < mask_roi_left.cols; ++x)
        {
            // If the pixel is white (255)
            if (mask_roi_left.at<uchar>(y, x) == 255)
            {
                // Adjust Y back to the original image ( captured input camera stream)) coordinate frame
                int original_y = y + roi_area_left.y;
                int original_x = x + roi_area_left.x;
                white_line_ptsL.push_back(cv::Point(original_x, original_y));
            }
        }
    }

    nb_whitePX_left = white_line_ptsL.size();
    // Scan the masked region for RIGHT PART
    for (int y = 0; y < mask_roi_right.rows; ++y)
    {
        for (int x = 0; x < mask_roi_right.cols; ++x)
        {
            // If the pixel is white (255)
            if (mask_roi_right.at<uchar>(y, x) == 255)
            {
                // Adjust Y back to the original image ( captured input camera stream)) coordinate frame
                int original_y = y + roi_area_right.y;
                int original_x = x + roi_area_right.x;
                white_line_ptsR.push_back(cv::Point(original_x, original_y));
            }
        }
    }
    nb_whitePX_right = white_line_ptsR.size();
    //*******//*******//*******//*******//*******//*******//*******//*******//******* */
    // =======================================================================
    // VALIDATE LEFT AND RIGHT ROAD-LINE CANDIDATES
    // =======================================================================

    const double maxAllowedWidth = 0.30;
    const double minValidBinRatio = 0.6;

    const bool leftValid = isValidRoadLine(
        XYZ_IMG, white_line_ptsL,
        minDepth, maxDepth, binSize,
        maxAllowedWidth, minValidBinRatio);

    const bool rightValid = isValidRoadLine(
        XYZ_IMG, white_line_ptsR,
        minDepth, maxDepth, binSize,
        maxAllowedWidth, minValidBinRatio);

    // =======================================================================
    // SELECT THE ROAD LINE
    // =======================================================================

    bool use_left_lane;

    if (leftValid && rightValid)
    {
        // Both candidates are valid: select the one with more white pixels
        use_left_lane =
            white_line_ptsL.size() >= white_line_ptsR.size();
    }
    else if (leftValid)
    {
        use_left_lane = true;
    }
    else if (rightValid)
    {
        use_left_lane = false;
    }
    else
    {
        use_left_lane =
            !(white_line_ptsL.size() >= white_line_ptsR.size()); // less white pixel is more accuray here..
    }

    const std::vector<cv::Point> &selected_lane =
        use_left_lane ? white_line_ptsL : white_line_ptsR;

    // const std::vector<cv::Point> &selected_lane_2ndBest_lane = !(use_left_lane) ? white_line_ptsL : white_line_ptsR;
    //*******//*******//*******//*******//*******//*******//*******//*******//******* */
    //  =======================================================================
    //  EXTRACT REPRESENTATIVE POINTS AND FIT SECOND-ORDER POLYNOMIAL
    //  =======================================================================

    int invalidDepth = 0;
    int validPoints = 0;

    // 1. Extract representative (Z, X) points
    std::vector<cv::Point2d> XZ =
        extractRepresentativePoints(
            XYZ_IMG,
            selected_lane,
            minDepth,
            maxDepth,
            binSize);

    // 2. Fit the second-order polynomial
    std::cout << "Representative points for polyfit: "
              << XZ.size() << std::endl;

    LanePolynomial result;

    if (XZ.size() < 3)
    {
        std::cerr << "Warning: Not enough representative points!"
                  << std::endl;
        return result;
    }

    std::vector<double> coeffs = secondOrderPolyFitting(XZ);

    if (coeffs.size() == 3)
    {
        result.a2 = coeffs[0];
        result.a1 = coeffs[1];
        result.a0 = coeffs[2];

        result.isValid = true;
        result.is_left_lane =
            use_left_lane;
    }
    else
    {
        std::cerr << "Warning: Polynomial fitting failed!"
                  << std::endl;
        return result;
    }
    // =======================================================================
    // CALCULATE ROAD WIDHT
    // =======================================================================
    // extracting white pixels of 2 masks is nearly equal (no distorsion)
    if (std::abs(
            static_cast<long long>(nb_whitePX_left) -
            static_cast<long long>(nb_whitePX_right)) < 500)
    {
        result.is_road_width_Valid = true;
        return result;
    }
    else
    {

        // =======================================================================
        // CALCULATE ROAD WIDTH USING 3D PIXEL EXTRACTION FROM LINES
        // =======================================================================
        std::vector<cv::Vec3f> left_3d_pts;
        std::vector<cv::Vec3f> right_3d_pts;

        // Extract 3D points from left line pixels
        for (const auto &pt : white_line_ptsL)
        {
            if (pt.x >= 0 && pt.x < XYZ_IMG.cols && pt.y >= 0 && pt.y < XYZ_IMG.rows)
            {
                cv::Vec3f pt3d = XYZ_IMG.at<cv::Vec3f>(pt.y, pt.x);
                if (pt3d[2] > 0.1f && pt3d[2] <= 30.0f)
                {
                    left_3d_pts.push_back(pt3d);
                }
            }
        }

        // Extract 3D points from right line pixels
        for (const auto &pt : white_line_ptsR)
        {
            if (pt.x >= 0 && pt.x < XYZ_IMG.cols && pt.y >= 0 && pt.y < XYZ_IMG.rows)
            {
                cv::Vec3f pt3d = XYZ_IMG.at<cv::Vec3f>(pt.y, pt.x);
                if (pt3d[2] > 0.1f && pt3d[2] <= 30.0f)
                {
                    right_3d_pts.push_back(pt3d);
                }
            }
        }

        double accumulated_width = 0.0;
        int valid_pairs = 0;

        // Match depths between left and right line points
        for (const auto &ptL : left_3d_pts)
        {
            float z_target = ptL[2];
            for (const auto &ptR : right_3d_pts)
            {
                if (std::abs(ptR[2] - z_target) < 0.1f) // Within 10cm depth match
                {
                    double instantaneous_width = std::abs(ptR[0] - ptL[0]);

                    // Filter realistic lane widths (e.g., between 2.0m and 4.5m)
                    if (instantaneous_width > 2.0 && instantaneous_width < 4.5)
                    {
                        accumulated_width += instantaneous_width;
                        valid_pairs++;
                    }
                    break;
                }
            }
        }

        if (valid_pairs > 5)
        {
            road_width = accumulated_width / valid_pairs;
            result.is_road_width_Valid = true;
            return result;
        }
        else
        {
            result.is_road_width_Valid = false;
            return result;
        }
    }
}