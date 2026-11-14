#ifndef DEBUG_LANE_DETECTION_HPP
#define DEBUG_LANE_DETECTION_HPP

#include <opencv2/opencv.hpp>

inline void showExtractedWhitePixels(const cv::Mat &white_line_img)
{
    cv::imshow("Extracted White Pixels", white_line_img);
    cv::waitKey(1);
}

#endif