#include "initializeCameraToCoGTransform.h"
#include "transformCameraToCoG.h"
#include "transformBodyToENU.h"
#include "camera_transform_initialize.h"
#include "camera_transform_terminate.h"
#include <iostream>
#include <array>

/*
xtern void transformBodyToENU(const double p_body[3],
                               const double p_cog_enu[3], double q[4],
                               double p_enu[3]);


*/

namespace CamType
{

    using CamPos_3d_V = std::array<double, 3>;
    using camera_dir_3d_V = std::array<double, 3>;
    using camera_up_3d_V = std::array<double, 3>;
    using cog_pos_3d_V = std::array<double, 3>;

    // 4x4 matrix stored in MATLAB column-major order
    using Transform4x4_Cam2Body = std::array<double, 16>;

    using p_body = std::array<double, 3>;
    using p_cog_enu = std::array<double, 3>;
    using quat = std::array<double, 4>;
    using p_enu = std::array<double, 3>;
}

CamType::Transform4x4_Cam2Body initializeCameraMounting(
    const std::string &yamlPath);