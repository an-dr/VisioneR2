// *************************************************************************
//
// Copyright (c) 2024 Andrei Gramakov. All rights reserved.
//
// site:    https://agramakov.me
// e-mail:  mail@agramakov.me
//
// *************************************************************************

#pragma once

#include <string>
#include <vector>
#include <opencv2/core/core.hpp>

#include "Quadrilateral.hpp"

class ObjectFinder {
 public:
    Quadrilateral Find(Mat &objectImg);
    const cv::Mat &GetScene() const { return m_sceneImg; }
    bool SetScene(cv::Mat &sceneImg);

 private:
    bool _DetectKeypoints(std::string image_name, cv::Mat &img,
                          std::vector<cv::KeyPoint> &keypoints,
                          bool show = false);

    bool _ComputeDescriptors(std::string image_name, cv::Mat &img,
                             std::vector<cv::KeyPoint> &keypoints,
                             cv::Mat &descriptors);

    bool _MatchDescriptors(cv::Mat &results, cv::Mat &dists,
                           std::vector<std::vector<cv::DMatch>> &matches,
                           bool useBFMatcher = false);

    bool _FindGoodMatches(cv::Mat &results, cv::Mat &dists,
                          std::vector<std::vector<cv::DMatch>> &matches,
                          std::vector<cv::Point2f> &src_points,
                          std::vector<cv::Point2f> &dst_points,
                          std::vector<int> &src_point_idxs,
                          std::vector<int> &dst_point_idxs,
                          std::vector<uchar> &outlier_mask,
                          bool useBFMatcher = false);

    bool _FindHomography(std::vector<cv::Point2f> &src_points,
                         std::vector<cv::Point2f> &dst_points,
                         std::vector<uchar> &outlier_mask, cv::Mat &H,
                         unsigned int minInliers = 8);

    Quadrilateral _GetObjectRectangle(cv::Mat &objectImg, cv::Mat &sceneImg,
                                      cv::Mat &H);

    bool
    _ValidateObjectQuadrilateral(Quadrilateral &sceneObjectQuadrilateral,
                                 cv::Mat &origObjectImg,
                                 int a2p_ratio = VALIDATION_OBJA2P_VS_SCENEA2P);

    static const int VALIDATION_OBJA2P_VS_SCENEA2P = 5;
    std::vector<cv::KeyPoint> m_objectKeypoints;
    std::vector<cv::KeyPoint> m_sceneKeypoints;

    cv::Mat m_objectDescriptors;
    cv::Mat m_sceneDescriptors;

    cv::Mat m_objectImg;
    cv::Mat m_sceneImg;
};
