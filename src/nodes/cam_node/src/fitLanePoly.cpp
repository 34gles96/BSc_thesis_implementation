#include "fitLanePoly.hpp"
#include <cmath>

LanePolynomial fitLanePolynomialOpenCV(
    const cv::Mat &uvIMG, const cv::Mat &XYZ_IMG, double &road_width) // first arg correspond to white images, 2nd is cv::Mat<cv::Vec3f>(u,v) = (X,Y,Z) mapping, 3rd arg is modifiyng road_witdh
{

    // region of interest for detected left or right lane using for desired trsajectory
    // Vertical ROI limits:
    // ignore upper part of image and bottom part containing vehicle hood
    int roi_top =
        static_cast<int>(uvIMG.rows * 0.50);

    int roi_bottom =
        static_cast<int>(uvIMG.rows * 0.85);

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
                white_line_ptsL.push_back(cv::Point(original_x, original_y));
            }
        }
    }
    nb_whitePX_right = white_line_ptsR.size();

    //*******//*******//*******//*******//*******//*******//*******//*******//******* */
    int whitePixels = 0;
    int invalidDepth = 0;
    int rejectedByColumn = 0;
    int validPoints = 0;

    std::vector<double> X_pts;
    std::vector<double> Z_pts;

    for (int v = 0; v < uvIMG.rows; ++v)
    {
        for (int u = 0; u < uvIMG.cols; ++u)
        {

            if (255 == uvIMG.at<uchar>(v, u))
            {

                ++whitePixels;
                // Matrix access is (row, col) -> (v, u)
                cv::Vec3f pt3D = XYZ_IMG.at<cv::Vec3f>(v, u);
                float Z = pt3D[2];

                // Check that disparity gave a valid depth (> 0)
                if (Z > 0.1f && Z <= 30.0f)
                {
                    X_pts.push_back(static_cast<double>(pt3D[0]));
                    Z_pts.push_back(static_cast<double>(Z));
                    ++validPoints;
                }
                else
                {
                    ++invalidDepth;
                }
            }
        }
    }

    // DEBUG
    std::cout
        << "White pixels: " << whitePixels << '\n'
        << "Rejected by column: " << rejectedByColumn << '\n'
        << "Invalid depth: " << invalidDepth << '\n'
        << "Valid points: " << validPoints << '\n';

    LanePolynomial result;
    int N = X_pts.size();

    // Debug print to see how many 2D pixels actually have valid 3D stereo depth
    std::cout << "Valid 3D lane points for polyfit: " << N << std::endl;

    // Need at least 3 points to solve for 3 parameters
    if (N < 3)
    {
        std::cerr << "Warning: Not enough points to fit polynomial!" << std::endl;
        return result;
    }

    // 1. Construct matrix A (N x 3) and vector B (N x 1)
    cv::Mat A(N, 3, CV_64F);
    cv::Mat B(N, 1, CV_64F);

    for (int i = 0; i < N; ++i)
    {
        double z = Z_pts[i];
        A.at<double>(i, 0) = z * z; // Z^2 column
        A.at<double>(i, 1) = z;     // Z column
        A.at<double>(i, 2) = 1.0;   // Constant 1 column

        B.at<double>(i, 0) = X_pts[i]; // Target X column
    }

    // 2. Solve A * coeffs = B using SVD
    cv::Mat coeffs;
    bool success = cv::solve(A, B, coeffs, cv::DECOMP_SVD);

    if (success)
    {
        result.a2 = coeffs.at<double>(0, 0);
        result.a1 = coeffs.at<double>(1, 0);
        result.a0 = coeffs.at<double>(2, 0);
        result.isValid = true;
        result.is_left_lane = (nb_whitePX_left > nb_whitePX_right) ? true : false;
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