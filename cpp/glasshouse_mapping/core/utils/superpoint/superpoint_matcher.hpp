// ----------------------------------------------------------------------------
// NOTE: This file has been adapted from the following project, but copyright
// still belongs to them. All rights reserved
// ----------------------------------------------------------------------------
// -            https://github.com/christian-rauch/super_point_inference -
// ----------------------------------------------------------------------------
#pragma once

#include "superpoint.hpp"

std::vector<std::tuple<int, int, float>> pairwise_matches(
    const Eigen::MatrixX2f &, const Eigen::MatrixX2f &, const Eigen::MatrixXf &,
    const Eigen::MatrixXf &, const cv::Mat &, const cv::Mat &);

cv::Mat draw_matches(const Eigen::MatrixX2f &, const Eigen::MatrixX2f &,
                     const std::vector<std::tuple<int, int, float>> &,
                     const cv::Mat &, const cv::Mat &);
