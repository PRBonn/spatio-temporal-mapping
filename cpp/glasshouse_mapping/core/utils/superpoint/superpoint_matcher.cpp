// ----------------------------------------------------------------------------
// NOTE: This file has been adapted from the following project, but copyright
// still belongs to them. All rights reserved
// ----------------------------------------------------------------------------
// -            https://github.com/christian-rauch/super_point_inference -
// ----------------------------------------------------------------------------
#include "superpoint_matcher.hpp"

#include <algorithm>
#include <cstdlib>
#include <eigen3/Eigen/Dense>
#include <execution>
#include <opencv4/opencv2/core/eigen.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/viz/types.hpp>
#include <vector>

std::vector<std::tuple<int, int, float>> pairwise_matches(
    const Eigen::MatrixX2f &kps1, const Eigen::MatrixX2f &kps2,
    const Eigen::MatrixXf &dsc1, const Eigen::MatrixXf &dsc2,
    const cv::Mat &img1, const cv::Mat &img2) {
  // Vector containing the correspondances (kp1_id, kp2_id, distance)
  const size_t max_size = kps1.rows() ? kps1.rows() < kps2.rows() : kps2.rows();
  std::vector<std::tuple<int, int, float>> match_ids;
  match_ids.reserve(max_size);

  if (dsc1.rows() > 0) {
    // Convert the descriptors from Eigen to OpenCV
    cv::Mat dsc1_cv;
    cv::eigen2cv(dsc1, dsc1_cv);
    cv::Mat dsc2_cv;
    cv::eigen2cv(dsc2, dsc2_cv);

    // Brute force matching
    std::vector<cv::DMatch> matches;
    cv::Ptr<cv::BFMatcher> matcher = cv::BFMatcher::create(cv::NORM_L2, true);
    matcher->match(dsc1_cv, dsc2_cv, matches);

    // Filter out matches computing homography
    cv::Mat inliers_mask;
    std::vector<cv::Point2f> matched_kps1(matches.size()),
        matched_kps2(matches.size());
    std::vector<int> matches_ids(matched_kps1.size());
    std::iota(matches_ids.begin(), matches_ids.end(), 0);
    std::for_each(std::execution::par, matches_ids.cbegin(), matches_ids.cend(),
                  [&](const int matches_idx) {
                    auto &kp1 = matched_kps1[matches_idx];
                    auto &kp2 = matched_kps2[matches_idx];
                    const auto &match = matches[matches_idx];
                    kp1 = cv::Point2f(kps1.row(match.queryIdx)[0] * img1.cols,
                                      kps1.row(match.queryIdx)[1] * img1.rows);
                    kp2 = cv::Point2f(kps2.row(match.trainIdx)[0] * img2.cols,
                                      kps2.row(match.trainIdx)[1] * img2.rows);
                  });

    cv::findHomography(matched_kps1, matched_kps2, inliers_mask,
                       cv::USAC_ACCURATE, 30);  // 0.05

    for (int i = 0; i < matches.size(); ++i) {
      if (inliers_mask.row(i).at<bool>(0)) {
        match_ids.emplace_back(matches[i].queryIdx, matches[i].trainIdx,
                               matches[i].distance);
      }
    }
  }
  match_ids.shrink_to_fit();
  return match_ids;
}

cv::Mat draw_matches(
    const Eigen::MatrixX2f &kps1, const Eigen::MatrixX2f &kps2,
    const std::vector<std::tuple<int, int, float>> &correspondences,
    const cv::Mat &img1, const cv::Mat &img2) {
  std::vector<cv::DMatch> matches;
  for (const auto &match : correspondences)
    matches.emplace_back(std::get<0>(match), std::get<1>(match),
                         std::get<2>(match));

  // Rotate the images
  cv::Mat img1_rotated, img2_rotated;
  cv::rotate(img1, img1_rotated, cv::ROTATE_90_CLOCKWISE);
  cv::rotate(img2, img2_rotated, cv::ROTATE_90_CLOCKWISE);

  std::vector<cv::KeyPoint> kps1_cv(kps1.rows());
  for (size_t i = 0; i < kps1.rows(); i++)
    kps1_cv[i].pt = cv::Point(img1.rows - (kps1.row(i)[1] * img1.rows),
                              kps1.row(i)[0] * img1.cols);

  std::vector<cv::KeyPoint> kps2_cv(kps2.rows());
  for (size_t i = 0; i < kps2.rows(); i++)
    kps2_cv[i].pt = cv::Point(img2.rows - (kps2.row(i)[1] * img1.rows),
                              kps2.row(i)[0] * img1.cols);

  cv::Mat img_matches;
  cv::drawMatches(img1_rotated, kps1_cv, img2_rotated, kps2_cv, matches,
                  img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                  std::vector<char>(),
                  cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  return img_matches;
}
