#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "readConfig.hpp"

cv::Mat &pinHole2XYZ(const cv::Mat &image_bgr8L, const cv::Mat &image_bgr8R, cv::Ptr<cv::StereoSGBM> stereo_, struct CameraParams config, cv::Mat &pointCloud);