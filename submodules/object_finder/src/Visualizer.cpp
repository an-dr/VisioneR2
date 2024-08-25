// *************************************************************************
//
// Copyright (c) 2024 Andrei Gramakov. All rights reserved.
//
// site:    https://agramakov.me
// e-mail:  mail@agramakov.me
//
// *************************************************************************

#include "Visualizer.hpp"
#include <opencv2/opencv.hpp>

void Visualizer::SetImg(cv::Mat img) {
    m_img_orig = img.clone();
    m_img_selection = img.clone();
    m_img_dismissed = img.clone();
}

cv::Mat Visualizer::SelectAndDismiss(Quadrilateral selection) {
    DrawQuadrilateral(m_img_selection, selection, true);
    ClearQuadrilateral(m_img_dismissed, selection);
    return m_img_dismissed;
}

cv::Mat &Visualizer::GetSceneWithSelection() { return m_img_selection; }

cv::Mat &Visualizer::GetSceneWithoutSelectedObjects() {
    return m_img_dismissed;
}

void Visualizer::DrawQuadrilateral(cv::Mat &img, Quadrilateral selection,
                                   bool crossed, cv::Scalar color,
                                   int thickness) {
    cv::line(img, selection[0], selection[1], color, thickness);
    cv::line(img, selection[1], selection[2], color, thickness);
    cv::line(img, selection[2], selection[3], color, thickness);
    cv::line(img, selection[3], selection[0], color, thickness);

    if (crossed) {
        cv::line(img, selection[0], selection[2], color, thickness);
        cv::line(img, selection[1], selection[3], color, thickness);
    }
}

void Visualizer::ClearQuadrilateral(cv::Mat &img, Quadrilateral selection) {
    cv::fillPoly(img, selection.GetIntegerPoints(), Scalar(128));
}
