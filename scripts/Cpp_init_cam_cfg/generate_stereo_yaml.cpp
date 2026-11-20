#include <opencv2/opencv.hpp>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char **argv)
{
    const std::string scenario_path = (argc > 1) ? argv[1] : "/ADC_27/launch/scenario_launch/silesia_scenario.json";
    const std::string output_path = (argc > 2) ? argv[2] : "/ADC_27/configData/cam_config/CameraConfig.yaml";

    // 1. Open BeamNG scenario JSON directly with OpenCV FileStorage
    cv::FileStorage fs_in(scenario_path, cv::FileStorage::READ);
    if (!fs_in.isOpened())
    {
        std::cerr << "Error: Could not open scenario JSON: " << scenario_path << std::endl;
        return 1;
    }

    cv::FileNode vehicles = fs_in["vehicles"];
    if (vehicles.empty() || !vehicles.isSeq())
    {
        std::cerr << "Error: 'vehicles' array not found in JSON." << std::endl;
        return 1;
    }

    cv::FileNode ego_vehicle = vehicles[0];
    for (const auto &veh : vehicles)
    {
        if (static_cast<std::string>(veh["name"]) == "ego")
        {
            ego_vehicle = veh;
            break;
        }
    }

    cv::FileNode sensors = ego_vehicle["sensors"];
    cv::FileNode cam_l, cam_r;
    for (const auto &sensor : sensors)
    {
        const std::string name = static_cast<std::string>(sensor["name"]);
        if (name == "camera_L")
            cam_l = sensor;
        else if (name == "camera_R")
            cam_r = sensor;
    }

    if (cam_l.empty() || cam_r.empty())
    {
        std::cerr << "Error: Could not find both 'camera_L' and 'camera_R' in scenario JSON." << std::endl;
        return 1;
    }

    // 2. Compute Intrinsics (K1, K2) from resolution & FOV
    const int width = static_cast<int>(cam_l["resolution"][0]);
    const int height = static_cast<int>(cam_l["resolution"][1]);
    const double cx = static_cast<double>(width) / 2.0;
    const double cy = static_cast<double>(height) / 2.0;

    double f_px = 0.0;
    if (!cam_l["field_of_view_y"].empty())
    {
        const double fov_y_rad = static_cast<double>(cam_l["field_of_view_y"]) * M_PI / 180.0;
        f_px = (static_cast<double>(height) / 2.0) / std::tan(fov_y_rad / 2.0);
    }
    else if (!cam_l["field_of_view_x"].empty())
    {
        const double fov_x_rad = static_cast<double>(cam_l["field_of_view_x"]) * M_PI / 180.0;
        f_px = (static_cast<double>(width) / 2.0) / std::tan(fov_x_rad / 2.0);
    }
    else
    {
        std::cerr << "Error: Neither 'field_of_view_y' nor 'field_of_view_x' found in camera_L." << std::endl;
        return 1;
    }

    // 3. Extract BeamNG Vehicle-Local Vectors (V: X=left, Y=backward, Z=up)
    auto read_vec3 = [](const cv::FileNode &node) -> cv::Vec3d
    {
        return cv::Vec3d(static_cast<double>(node[0]),
                         static_cast<double>(node[1]),
                         static_cast<double>(node[2]));
    };

    const cv::Vec3d pos_l_V = read_vec3(cam_l["pos"]);
    const cv::Vec3d dir_l_V = read_vec3(cam_l["dir"]);
    const cv::Vec3d up_l_V = read_vec3(cam_l["up"]);

    const cv::Vec3d pos_r_V = read_vec3(cam_r["pos"]);
    const cv::Vec3d dir_r_V = read_vec3(cam_r["dir"]);
    const cv::Vec3d up_r_V = read_vec3(cam_r["up"]);

    fs_in.release();

    // 4. Convert BeamNG Vehicle Frame (V) -> EKF Body Frame (X=forward, Y=left, Z=up)
    //    x_body = -y_beamng,  y_body = x_beamng,  z_body = z_beamng
    const cv::Vec3d pos_l_body(-pos_l_V[1], pos_l_V[0], pos_l_V[2]);
    const cv::Vec3d pos_r_body(-pos_r_V[1], pos_r_V[0], pos_r_V[2]);
    const cv::Vec3d baseline_body = pos_r_body - pos_l_body;
    const double baseline_m = cv::norm(baseline_body);

    // 5. Build OpenCV Matrices (K1, D1, K2, D2, R, T, Q)
    cv::Mat K1 = (cv::Mat_<double>(3, 3) << f_px, 0.0, cx,
                  0.0, f_px, cy,
                  0.0, 0.0, 1.0);
    cv::Mat K2 = K1.clone();

    cv::Mat D1 = cv::Mat::zeros(1, 5, CV_64F);
    cv::Mat D2 = cv::Mat::zeros(1, 5, CV_64F);

    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat T = (cv::Mat_<double>(3, 1) << -baseline_m, 0.0, 0.0);

    cv::Mat Q = (cv::Mat_<double>(4, 4) << 1.0, 0.0, 0.0, -cx,
                 0.0, 1.0, 0.0, -cy,
                 0.0, 0.0, 0.0, f_px,
                 0.0, 0.0, 1.0 / baseline_m, 0.0);

    auto vec_to_mat3x1 = [](const cv::Vec3d &v) -> cv::Mat
    {
        return (cv::Mat_<double>(3, 1) << v[0], v[1], v[2]);
    };

    // 6. Write OpenCV YAML (%YAML:1.0 with !!opencv-matrix tags)
    cv::FileStorage fs_out(output_path, cv::FileStorage::WRITE);
    if (!fs_out.isOpened())
    {
        std::cerr << "Error: Failed to open output YAML file: " << output_path << std::endl;
        return 1;
    }

    fs_out << "image_width" << width;
    fs_out << "image_height" << height;

    fs_out << "K1" << K1;
    fs_out << "D1" << D1;
    fs_out << "K2" << K2;
    fs_out << "D2" << D2;

    fs_out << "R" << R;
    fs_out << "T" << T;
    fs_out << "Q" << Q;

    fs_out << "stereo_baseline_body" << vec_to_mat3x1(baseline_body);
    fs_out << "cog_pos_V" << vec_to_mat3x1(cv::Vec3d(0.0, 0.0, 0.0));

    fs_out << "camera_L_pos_V" << vec_to_mat3x1(pos_l_V);
    fs_out << "camera_L_dir_V" << vec_to_mat3x1(dir_l_V);
    fs_out << "camera_L_up_V" << vec_to_mat3x1(up_l_V);

    fs_out << "camera_R_pos_V" << vec_to_mat3x1(pos_r_V);
    fs_out << "camera_R_dir_V" << vec_to_mat3x1(dir_r_V);
    fs_out << "camera_R_up_V" << vec_to_mat3x1(up_r_V);

    fs_out.release();

    std::cout << "Generated " << output_path
              << " | Resolution: " << width << "x" << height
              << " | f: " << f_px << " px"
              << " | Baseline: " << baseline_m << " m" << std::endl;

    return 0;
}