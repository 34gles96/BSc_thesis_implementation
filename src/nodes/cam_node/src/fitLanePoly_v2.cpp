#include "fitLanePoly.hpp"
#include "road_boundary_fitting.hpp"
#include "config.hpp"
#include "roadLineValidator.hpp"
#include "horizontConfig.hpp"

#include "selected_white_px_filter.hpp"
#include <cmath>
#include <iostream>

const double binSize = 0.1;

// first arg correspond to white images, 2nd is cv::Mat<cv::Vec3f>(u,v) = (X,Y,Z) mapping, 3rd arg is modifiyng road_witdh
std::vector<LanePolynomial> fitLanePolynomialOpenCV(
    const cv::Mat &uvIMG,
    const cv::Mat &XYZ_IMG, double &road_width,
    const CameraParams &cam,
    const RoadPlane &road_plane

)
{

    std::vector<LanePolynomial> result_2_bounday_road_line;
    result_2_bounday_road_line.reserve(2);

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

    // --- LEFT ROI: scan from road center (cols - 1) outward to left (0) ---
    for (int v_local = 0; v_local < mask_roi_left.rows; ++v_local)
    {
        const uchar *row_ptr = mask_roi_left.ptr<uchar>(v_local);
        int u_local = mask_roi_left.cols - 1;

        // 1. Skip white pixels sitting right on the center split
        while (u_local >= 0 && row_ptr[u_local] == 255)
            --u_local;

        // 2. Cross black asphalt until rising edge (0 -> 255)
        while (u_local >= 0 && row_ptr[u_local] == 0)
            --u_local;

        // 3. Collect white run until falling edge (255 -> 0)
        std::vector<cv::Point> row_candidates;
        while (u_local >= 0 && row_ptr[u_local] == 255)
        {
            row_candidates.emplace_back(u_local + roi_area_left.x, v_local + roi_area_left.y);
            --u_local;
        }

        // Keep only thin painted road lines (2..30 px), reject wide concrete barriers
        if (row_candidates.size() >= 2 && row_candidates.size() <= 30)
        {
            white_line_ptsL.insert(white_line_ptsL.end(), row_candidates.begin(), row_candidates.end());
        }
    }

    // --- RIGHT ROI: scan from road center (0) outward to right (cols - 1) ---
    for (int v_local = 0; v_local < mask_roi_right.rows; ++v_local)
    {
        const uchar *row_ptr = mask_roi_right.ptr<uchar>(v_local);
        int u_local = 0;

        // 1. Skip white pixels sitting right on the center split
        while (u_local < mask_roi_right.cols && row_ptr[u_local] == 255)
            ++u_local;

        // 2. Cross black asphalt until rising edge (0 -> 255)
        while (u_local < mask_roi_right.cols && row_ptr[u_local] == 0)
            ++u_local;

        // 3. Collect white run until falling edge (255 -> 0)
        std::vector<cv::Point> row_candidates;
        while (u_local < mask_roi_right.cols && row_ptr[u_local] == 255)
        {
            row_candidates.emplace_back(u_local + roi_area_right.x, v_local + roi_area_right.y);
            ++u_local;
        }

        // Keep only thin painted road lines (2..30 px), reject wide concrete barriers
        if (row_candidates.size() >= 2 && row_candidates.size() <= 30)
        {
            white_line_ptsR.insert(white_line_ptsR.end(), row_candidates.begin(), row_candidates.end());
        }
    }

    nb_whitePX_left = white_line_ptsL.size();
    nb_whitePX_right = white_line_ptsR.size();

    //*******//*******//*******//*******//*******//*******//*******//*******//******* */
    // =======================================================================
    // CALCULATE DEPTH BOUNDS FOR BOTH CANDIDATES FIRST
    // =======================================================================

    const double maxAllowedWidth = 0.30;
    const double minValidBinRatio = 0.6;

    auto left_bounds = calculateTrajectoryBoundsFromLane(
        XYZ_IMG, white_line_ptsL);

    auto right_bounds = calculateTrajectoryBoundsFromLane(
        XYZ_IMG, white_line_ptsR);

    // =======================================================================
    // VALIDATE LEFT AND RIGHT ROAD-LINE CANDIDATES
    // =======================================================================

    const bool leftValid =
        left_bounds &&
        isValidRoadLine(
            XYZ_IMG, white_line_ptsL,
            left_bounds->min_z, left_bounds->predicted_horizon, binSize,
            maxAllowedWidth, minValidBinRatio);

    const bool rightValid =
        right_bounds &&
        isValidRoadLine(
            XYZ_IMG, white_line_ptsR,
            right_bounds->min_z, right_bounds->predicted_horizon, binSize,
            maxAllowedWidth, minValidBinRatio);

    // const bool leftValid = true;
    // const bool rightValid = true;
    // =======================================================================
    // SELECT THE REFERENCE ROAD LINE
    // =======================================================================

    if (!leftValid && !rightValid)
    {
        std::cerr << "Skipping lane fit: neither lane passed validation\n";
        return {LanePolynomial{}, LanePolynomial{}};
    }

    // ============ FILTERING WHITE LINE PX's due to lateral extension of X  (/road_witdh)

    /*


    /*

    std::optional<std::vector<cv::Point2d>> filter_out_lat_extenison_px(
    const cv::Mat &XYZ_img,
    const std::vector<cv::Point> &white_px_line,
    const double depth_bin_size_meter,
    const CameraParams &cam,
    const RoadPlane &road_plane);*/

    const auto white_line_ptsL_filtered =
        filter_out_lat_extenison_px(
            XYZ_IMG,
            white_line_ptsL,
            binSize,
            cam,
            road_plane);

    const auto white_line_ptsR_filtered =
        filter_out_lat_extenison_px(
            XYZ_IMG,
            white_line_ptsR,
            binSize,
            cam,
            road_plane);

    bool use_left_lane = false;

    if (leftValid && rightValid && white_line_ptsL_filtered.has_value() && white_line_ptsR_filtered.has_value())
    {

        use_left_lane = white_line_ptsL_filtered.value().size() >= white_line_ptsR_filtered.value().size();
        // use_left_lane = white_line_ptsL.size() >= white_line_ptsR.size();
        //  use_left_lane = left_bounds->min_z < right_bounds->min_z;
    }
    else if (leftValid)
    {
        use_left_lane = true;
    }
    else if (rightValid)
    {
        use_left_lane = false;
    }

    const std::vector<cv::Point> &selected_lane =
        use_left_lane ? white_line_ptsL : white_line_ptsR;

    double Nb_px_selected_lane_ref = selected_lane.size();

    const std::vector<cv::Point> &selected_lane_2ndBest_lane =
        !(use_left_lane) ? white_line_ptsL : white_line_ptsR;

    // ======== design the min trajector due to Z .================
    /*

        std::vector<double> valid_depths;
        valid_depths.reserve(selected_lane.size());

        constexpr double MAX_DEPTH_CAP = 45.0;
        constexpr double MIN_DEPTH_CAP = 3.0;
        double farthest_z = -std::numeric_limits<double>::infinity();
        for (const auto &pt : selected_lane)
        {
            if (pt.x < 0 || pt.x >= XYZ_IMG.cols ||
                pt.y < 0 || pt.y >= XYZ_IMG.rows)
                continue;

            const cv::Vec3f xyz = XYZ_IMG.at<cv::Vec3f>(pt.y, pt.x);
            const float z = xyz[2];

            if (std::isfinite(xyz[0]) && std::isfinite(xyz[1]) &&
                std::isfinite(z) &&
                z >= MIN_DEPTH_CAP &&
                z <= MAX_DEPTH_CAP)
            {
                valid_depths.push_back(z);
                farthest_z = std::max(farthest_z, static_cast<double>(z));
            }
        }

        if (valid_depths.size() < 3)
        {
            std::cerr << "Not enough valid lane depths: "
                      << valid_depths.size() << '\n';
            return {LanePolynomial{}, LanePolynomial{}};
        }

        // Use the 2.5th percentile to avoid one spuriously near stereo point.
        // Use min_element instead if you specifically want the closest single point.
        const std::size_t index_min = (valid_depths.size() - 1) / 99;

        std::nth_element(
            valid_depths.begin(),
            valid_depths.begin() + index_min,
            valid_depths.end());

        config::MIN_TRAJECTORY_Z = static_cast<double>(valid_depths[index_min]);

        double tunning_farthest_z = 0.87;
        if (std::isfinite(farthest_z))
        {
            config::PREDICTED_HORIZON = farthest_z * tunning_farthest_z;
        }
            */

    // =======================================================================
    // DETERMINE THE FINAL DEPTH INTERVAL
    // =======================================================================

    const auto &selected_bounds =
        use_left_lane ? left_bounds : right_bounds;

    const auto &second_bounds =
        use_left_lane ? right_bounds : left_bounds;

    const bool secondValid =
        use_left_lane ? rightValid : leftValid;

    if (!selected_bounds)
    {
        return {LanePolynomial{}, LanePolynomial{}};
    }

    // double predictedHorizon_tunning = 0.5;
    double minTrajectoryZ = selected_bounds->min_z;
    double predictedHorizon = std::max((selected_bounds->predicted_horizon), minTrajectoryZ + 1);

    if (!(std::abs(
              static_cast<long long>(nb_whitePX_left) -
              static_cast<long long>(nb_whitePX_right)) > Nb_px_selected_lane_ref * 0.55))
    {
        if (secondValid && second_bounds)
        {
            minTrajectoryZ = std::max(
                second_bounds->min_z,
                minTrajectoryZ);

            predictedHorizon = std::min(
                second_bounds->predicted_horizon,
                predictedHorizon);
        }
    }

    config::MIN_TRAJECTORY_Z = minTrajectoryZ;
    config::PREDICTED_HORIZON = predictedHorizon;

    //*******//*******//*******//*******//*******//*******//*******//*******//******* */
    //  =======================================================================
    //  EXTRACT REPRESENTATIVE POINTS AND FIT SECOND-ORDER POLYNOMIAL
    //  =======================================================================

    // 1. Extract representative (Z, X) points : transforming the lateral extension of X-coordinates of road line into 1 line by taking avg
    std::vector<cv::Point2d> XZ =
        extractRepresentativePoints(
            XYZ_IMG,
            selected_lane,
            minTrajectoryZ,
            predictedHorizon,
            binSize);

    std::vector<cv::Point2d> XZ_2ndBest =
        extractRepresentativePoints(
            XYZ_IMG,
            selected_lane_2ndBest_lane,
            minTrajectoryZ,
            predictedHorizon,
            binSize);

    bool valid_XZ_2ndBest = false;
    if (XZ_2ndBest.size() >= 3)
    {
        valid_XZ_2ndBest = true;
    }

    // 2. Fit the second-order polynomial
    std::cout << "Representative points for polyfit: "
              << XZ.size() << std::endl;

    LanePolynomial result; // initializated w default comstrucotr, boolen expression gets value of false
    LanePolynomial result_2ndBest;

    constexpr std::size_t min_fit_bins = 5;

    if (XZ.size() < min_fit_bins)
    {
        std::cerr << "Skipping lane fit: only "
                  << XZ.size() << " representative bins\n";
        return {LanePolynomial{}, LanePolynomial{}};
    }

    std::vector<double> coeffs = secondOrderPolyFitting(XZ);
    std::vector<double> coeffs_2ndBest;
    if (valid_XZ_2ndBest)
    {
        coeffs_2ndBest = secondOrderPolyFitting(XZ_2ndBest);
    }

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
        return {result, result_2ndBest}; // doingin implicit intruction of vec.emplace_back();
    }

    if (coeffs_2ndBest.size() == 3)
    {
        result_2ndBest.a2 = coeffs_2ndBest[0];
        result_2ndBest.a1 = coeffs_2ndBest[1];
        result_2ndBest.a0 = coeffs_2ndBest[2];

        result_2ndBest.isValid = true;
    }

    // =======================================================================
    // CALCULATE ROAD WIDHT
    // =======================================================================
    // extracting white pixels of 2 masks is nearly equal (no distorsion)

    if (std::abs(
            static_cast<long long>(nb_whitePX_left) -
            static_cast<long long>(nb_whitePX_right)) > Nb_px_selected_lane_ref * 0.30) // 30% of ref_lane pixel difference -> 2ndBest lane no valid
    {
        result.is_road_width_Valid = false;

        std::cout << "[Lane PX] Left: " << nb_whitePX_left
                  << " | Right: " << nb_whitePX_right << std::endl;

        return {result, LanePolynomial()};
    }
    else
    {

        // =======================================================================
        // CALCULATE ROAD WIDTH USING 3D PIXEL EXTRACTION FROM LINES
        // =======================================================================
        // =======================================================================
        // CALCULATE ROAD WIDTH IN O(N) USING BINNED REPRESENTATIVE POINTS
        // =======================================================================
        result.is_road_width_Valid = false;

        // Require both polynomials to be valid (at least 3 binned points each)
        if (result_2ndBest.isValid)
        {
            // Ensure both vectors are sorted by depth Z (stored in .x)
            auto by_depth = [](const cv::Point2d &a, const cv::Point2d &b)
            {
                return a.x < b.x;
            };
            std::sort(XZ.begin(), XZ.end(), by_depth);
            std::sort(XZ_2ndBest.begin(), XZ_2ndBest.end(), by_depth);

            double accumulated_width = 0.0;
            int valid_pairs = 0;
            const double depth_tolerance = binSize * 0.5; // 0.25m for 0.5m bins

            // Two-pointer linear scan over depth-sorted binned points
            std::size_t j = 0;
            for (const auto &pt1 : XZ)
            {
                // Advance j until XZ_2ndBest[j].x is within tolerance of pt1.x
                while (j < XZ_2ndBest.size() && XZ_2ndBest[j].x < pt1.x - depth_tolerance)
                {
                    ++j;
                }

                if (j < XZ_2ndBest.size() && std::abs(XZ_2ndBest[j].x - pt1.x) <= depth_tolerance)
                {
                    // .y stores lateral X coordinate in meters
                    double instantaneous_width = std::abs(XZ_2ndBest[j].y - pt1.y);

                    // Filter realistic lane widths (2.0m to 6.0m)
                    if (instantaneous_width >= 9.0 && instantaneous_width <= 12.0)
                    {
                        accumulated_width += instantaneous_width;
                        ++valid_pairs;
                    }
                }
            }

            if (valid_pairs >= 4)
            {
                road_width = accumulated_width / valid_pairs;
                result.is_road_width_Valid = true;
            }
        }

        return {result, result_2ndBest};
    }
}