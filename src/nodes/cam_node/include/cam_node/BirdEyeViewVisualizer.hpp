#include <opencv2/opencv.hpp>
#include <vector>
#include "desiredTrajectory.hpp"
#include "config.hpp"

class BirdEyeViewVisualizer
{
public:
    static void draw(
        const DesiredTrajectory &trajectory,
        double a2 = 0.0, double a1 = 0.0, double a0 = 0.0,
        const std::vector<cv::Vec3f> &raw_white_points = {},
        const double predictedHorizont_m = static_cast<double>(config::PREDICTED_HORIZON),
        bool second_poly_valid = false,
        double a2_2nd = 0.0,
        double a1_2nd = 0.0,
        double a0_2nd = 0.0)
    {

        // 1. Canvas Configuration
        int width = 600;
        int height = 800;
        cv::Mat bev_img = cv::Mat::zeros(height, width, CV_8UC3);

        // Scale: 25 pixels = 1 meter
        double scale = 25.0;

        // Ego vehicle (origin) positioned at the bottom-center of the map
        int origin_x = width / 2;
        int origin_y = height - 60;

        // 2. Draw Distance Grid Lines (Every 2 meters up to detected avaible depth meters ahead)
        for (double d = config::MIN_TRAJECTORY_Z; d <= config::PREDICTED_HORIZON; d += 0.25)
        {
            int y_pos = origin_y - static_cast<int>(d * scale);
            cv::line(bev_img, cv::Point(0, y_pos), cv::Point(width, y_pos), cv::Scalar(40, 40, 40), 1);
            cv::putText(bev_img, std::to_string(static_cast<int>(d)) + "m",
                        cv::Point(origin_x + 15, y_pos + 4),
                        cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(120, 120, 120), 1);
        }

        // Draw centerline reference (X = 0)
        cv::line(bev_img, cv::Point(origin_x, 0), cv::Point(origin_x, height), cv::Scalar(50, 50, 50), 1, cv::LINE_AA);

        // 3. Draw Ego Vehicle Representation (Blue rectangle at bottom-center)
        cv::Rect ego_box(origin_x - 15, origin_y - 25, 30, 50);
        cv::rectangle(bev_img, ego_box, cv::Scalar(255, 150, 0), cv::FILLED);
        cv::putText(bev_img, "EGO", cv::Point(origin_x - 12, origin_y + 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);

        // 4. DRAW RAW DETECTED WHITE PIXELS (Scatter points in Light Gray)
        for (const auto &pt : raw_white_points)
        {
            double X = pt[0]; // Lateral meters
            double Z = pt[2]; // Forward meters

            int px = origin_x - static_cast<int>(X * scale);
            int pz = origin_y - static_cast<int>(Z * scale);

            if (px >= 0 && px < width && pz >= 0 && pz < height)
            {
                cv::circle(bev_img, cv::Point(px, pz), 1, cv::Scalar(180, 180, 180), -1); // Gray dots
            }
        }

        // 5. Draw Fitted Polynomial Lane Line (Red line)
        if (a2 != 0.0 || a1 != 0.0 || a0 != 0.0)
        {
            std::vector<cv::Point> lane_pts;
            for (double Z = config::MIN_TRAJECTORY_Z; Z <= config::PREDICTED_HORIZON; Z += 0.25)
            {
                double X = a2 * Z * Z + a1 * Z + a0;
                int px = origin_x - static_cast<int>(X * scale);
                int pz = origin_y - static_cast<int>(Z * scale);

                if (px >= 0 && px < width && pz >= 0 && pz < height)
                {
                    lane_pts.push_back(cv::Point(px, pz));
                }
            }

            for (size_t i = 1; i < lane_pts.size(); ++i)
            {
                cv::line(bev_img, lane_pts[i - 1], lane_pts[i], cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
            }
        }

        // 5.1 DRAW SECOND FITTED POLYNOMIAL (Dark red line)

        if (second_poly_valid)
        {
            std::vector<cv::Point> lane_pts_2nd;

            for (double Z = config::MIN_TRAJECTORY_Z;
                 Z <= config::PREDICTED_HORIZON;
                 Z += 0.25)
            {
                double X = a2_2nd * Z * Z + a1_2nd * Z + a0_2nd;

                int px = origin_x - static_cast<int>(X * scale);
                int pz = origin_y - static_cast<int>(Z * scale);

                if (px >= 0 && px < width && pz >= 0 && pz < height)
                {
                    lane_pts_2nd.push_back(cv::Point(px, pz));
                }
            }

            for (size_t i = 1; i < lane_pts_2nd.size(); ++i)
            {
                cv::line(
                    bev_img,
                    lane_pts_2nd[i - 1],
                    lane_pts_2nd[i],
                    cv::Scalar(0, 0, 180),
                    2,
                    cv::LINE_AA);
            }
        }

        // 6. Draw Desired Centerline Trajectory (Green line & Yellow waypoints)
        std::vector<cv::Point> traj_pts;
        for (const auto &pt : trajectory.xyz_points)
        {
            double X = pt[0];
            double Z = pt[2];

            int px = origin_x - static_cast<int>(X * scale);
            int pz = origin_y - static_cast<int>(Z * scale);

            if (px >= 0 && px < width && pz >= 0 && pz < height)
            {
                traj_pts.push_back(cv::Point(px, pz));
                cv::circle(bev_img, cv::Point(px, pz), 3, cv::Scalar(0, 255, 255), -1);
            }
        }

        for (size_t i = 1; i < traj_pts.size(); ++i)
        {
            cv::line(bev_img, traj_pts[i - 1], traj_pts[i], cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
        }

        // 7. Display the BEV window
        cv::imshow("Bird's-Eye View Metric Map", bev_img);
        // cv::waitKey(1);
    }
};