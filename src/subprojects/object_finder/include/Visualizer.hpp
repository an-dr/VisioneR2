// *************************************************************************
//
// Copyright (c) 2024 Andrei Gramakov. All rights reserved.
//
// site:    https://agramakov.me
// e-mail:  mail@agramakov.me
//
// *************************************************************************

#pragma once

#include <opencv2/opencv.hpp>
#include "Quadrilateral.hpp"

class Visualizer {
 public:
    Visualizer() = default;
    void SetImg(cv::Mat img);
    cv::Mat SelectAndDismiss(Quadrilateral selection);
    cv::Mat &GetSceneWithSelection();
    cv::Mat &GetSceneWithoutSelectedObjects();

 private:
    cv::Mat m_img_orig;
    cv::Mat m_img_selection;
    cv::Mat m_img_dismissed;
    static void DrawQuadrilateral(cv::Mat &img, Quadrilateral selection,
                                  bool crossed,
                                  cv::Scalar color = Scalar(0, 255, 0),
                                  int thickness = 2);
    static void ClearQuadrilateral(cv::Mat &img, Quadrilateral selection);
};
