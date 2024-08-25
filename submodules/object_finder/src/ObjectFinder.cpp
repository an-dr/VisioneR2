// *************************************************************************
//
// Copyright (c) 2024 Andrei Gramakov. All rights reserved.
//
// This file is licensed under the terms of the MIT license.
// For a copy, see: https://opensource.org/licenses/MIT
//
// site:    https://agramakov.me
// e-mail:  mail@agramakov.me
//
// *************************************************************************

#include <opencv2/calib3d/calib3d.hpp>  // for homography
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>  // for wrapPerspective
#include <opencv2/opencv_modules.hpp>
#include <stdio.h>

#include "Quadrilateral.hpp"
#include "opencv_tools.hpp"


// #include "ulog.h"
#define log_debug printf
#define log_error printf
#define log_trace printf

#include "ObjectFinder.hpp"

using namespace cv;
using namespace std;

bool ObjectFinder::_ValidateObjectQuadrilateral(
    Quadrilateral &sceneObjectQuadrilateral, cv::Mat &origObjectImg,
    int a2p_ratio) {
    // Validate object quadrilateral is in the scene
    for (auto p : sceneObjectQuadrilateral.arr) {
        if (p.x < 0 || p.y < 0 || p.x > m_sceneImg.cols ||
            p.y > m_sceneImg.rows) {
            return false;
        }
    }

    // Object A2P
    float origObjArea = origObjectImg.rows * origObjectImg.cols;
    float origObjPerimeter = 2 * (origObjectImg.rows + origObjectImg.cols);
    float origA2P = origObjArea / origObjPerimeter;

    auto sceneObjPerimeter = sceneObjectQuadrilateral.GetPerimeter();
    sceneObjPerimeter = isnan(sceneObjPerimeter) ? 0 : sceneObjPerimeter;
    auto obj_area = sceneObjectQuadrilateral.GetArea();
    obj_area = isnan(obj_area) ? 0 : obj_area;

    // Scene A2P
    float sceneObjA2P =
        sceneObjPerimeter == 0 ? 0 : obj_area / sceneObjPerimeter;

    // log_debug("Validation orig A2P vs scene A2P: %f | %f", origA2P,
    // sceneObjA2P);
    log_debug("Validation orig A2P / scene A2P: %f", origA2P / sceneObjA2P);
    if (origA2P / sceneObjA2P > a2p_ratio) {
        return false;
    }

    // Obj size
    auto scene_area = m_sceneImg.rows * m_sceneImg.cols;
    auto obj_area_ratio = scene_area / obj_area;
    if (obj_area_ratio > 10) {
        return false;
    }

    return true;
}

Quadrilateral ObjectFinder::Find(Mat &objectImg) {
    m_objectImg = objectImg;
    if (m_objectImg.empty() || m_sceneImg.empty()) {
        log_error("Empty images");
        return Quadrilateral();
    }

    _DetectKeypoints("Object", m_objectImg, m_objectKeypoints, false);
    _ComputeDescriptors("Object", m_objectImg, m_objectKeypoints,
                        m_objectDescriptors);

    _DetectKeypoints("Scene", m_sceneImg, m_sceneKeypoints);
    _ComputeDescriptors("Scene", m_sceneImg, m_sceneKeypoints,
                        m_sceneDescriptors);

    bool useBFMatcher = true;

    // Match descriptors
    Mat results, dists;
    vector<vector<DMatch>> matches;
    _MatchDescriptors(results, dists, matches, useBFMatcher);

    // Find good matches
    vector<Point2f> src_points, dst_points;
    vector<int> src_point_idxs, dst_point_idxs;
    vector<uchar> outlier_mask;
    _FindGoodMatches(results, dists, matches, src_points, dst_points,
                     src_point_idxs, dst_point_idxs, outlier_mask,
                     useBFMatcher);

    // Find homography
    Mat H;
    bool result = false;
    result = _FindHomography(src_points, dst_points, outlier_mask, H, 20);
    if (!result) {
        log_debug("homography not found");
        return Quadrilateral();
    }

    // Get result
    Quadrilateral found_object =
        _GetObjectRectangle(m_objectImg, m_sceneImg, H);

    // Check
    if (!_ValidateObjectQuadrilateral(
            found_object, m_objectImg,
            ObjectFinder::VALIDATION_OBJA2P_VS_SCENEA2P)) {
        return Quadrilateral();
    }
    return found_object;
}

bool ObjectFinder::SetScene(cv::Mat &sceneImg) {
    if (sceneImg.empty()) {
        log_error("Empty/no image");
        return false;
    }
    m_sceneImg = sceneImg;
    return true;
}

bool ObjectFinder::_DetectKeypoints(string image_name, Mat &img,
                                    vector<KeyPoint> &keypoints, bool show) {
    Ptr<FeatureDetector> detector = getFeatureDetector();
    detector->detect(img, keypoints);

    log_debug("[%s] %d keypoints detected", image_name.c_str(),
              (int)keypoints.size());

    if (show) {
        // Draw keypoints
        Mat img_keypoints = img.clone();
        drawKeypoints(img_keypoints, keypoints, img_keypoints, Scalar::all(-1),
                      DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        // show
        imshow("Keypoints", img_keypoints);
        waitKey(1500);
    }
    return true;
}

bool ObjectFinder::_ComputeDescriptors(string image_name, Mat &img,
                                       vector<KeyPoint> &keypoints,
                                       Mat &descriptors) {
    if (keypoints.size() == 0) {
        log_error("[%s]: no keypoints", image_name.c_str());
        return false;
    }

    Ptr<DescriptorExtractor> extractor = getDescriptorExtractor();
    extractor->compute(img, keypoints, descriptors);
    log_debug("[%s] %d descriptors extracted", image_name.c_str(),
              descriptors.rows);
    return true;
}

bool ObjectFinder::_MatchDescriptors(Mat &results, Mat &dists,
                                     vector<vector<DMatch>> &matches,
                                     bool useBFMatcher) {
    int k = 2;  // find the 2 nearest neighbors
    if (m_objectDescriptors.type() == CV_8U) {
        // Binary descriptors detected (from ORB, Brief, BRISK, FREAK)
        log_trace("Binary descriptors detected...");
        if (useBFMatcher) {
            // use NORM_HAMMING2 for ORB descriptor with WTA_K == 3 or 4 (see
            // ORB constructor)
            BFMatcher matcher(NORM_HAMMING);
            matcher.knnMatch(m_objectDescriptors, m_sceneDescriptors, matches,
                             k);
        } else {
            // Create Flann LSH index
            flann::Index flannIndex(m_sceneDescriptors,
                                    flann::LshIndexParams(12, 20, 2),
                                    cvflann::FLANN_DIST_HAMMING);
            log_debug("Creating FLANN LSH index is done");

            // search (nearest neighbor)
            flannIndex.knnSearch(m_objectDescriptors, results, dists, k,
                                 flann::SearchParams());
        }
    } else {
        // assume it is CV_32F
        log_trace("Float descriptors detected...");
        if (useBFMatcher) {
            BFMatcher matcher(NORM_L2);
            matcher.knnMatch(m_objectDescriptors, m_sceneDescriptors, matches,
                             k);
        } else {
            // Create Flann KDTree index
            flann::Index flannIndex(m_sceneDescriptors,
                                    flann::KDTreeIndexParams(),
                                    cvflann::FLANN_DIST_EUCLIDEAN);
            log_trace("Time creating FLANN KDTree index = %lld ms", 0LL);

            // search (nearest neighbor)
            flannIndex.knnSearch(m_objectDescriptors, results, dists, k,
                                 flann::SearchParams());
        }
    }
    log_trace("Time nearest neighbor search = %lld ms", 0LL);

    // Conversion to CV_32F if needed
    if (dists.type() == CV_32S) {
        Mat temp;
        dists.convertTo(temp, CV_32F);
        dists = temp;
    }
    return true;
}

bool ObjectFinder::_FindGoodMatches(
    Mat &results, Mat &dists, vector<vector<DMatch>> &matches,
    vector<Point2f> &src_points, vector<Point2f> &dst_points,
    vector<int> &src_point_idxs, vector<int> &dst_point_idxs,
    vector<uchar> &outlier_mask, bool useBFMatcher) {
    // Find correspondences by NNDR (Nearest Neighbor Distance Ratio)
    float nndrRatio = 0.8f;
    // Check if this descriptor matches with those of the objects
    if (!useBFMatcher) {
        for (int i = 0; i < m_objectDescriptors.rows; ++i) {
            // Apply NNDR
            log_trace("q=%d dist1=%f dist2=%f", i, dists.at<float>(i, 0),
                      dists.at<float>(i, 1));
            if (results.at<int>(i, 0) >= 0 && results.at<int>(i, 1) >= 0 &&
                dists.at<float>(i, 0) <= nndrRatio * dists.at<float>(i, 1)) {
                src_points.push_back(m_objectKeypoints.at(i).pt);
                src_point_idxs.push_back(i);

                dst_points.push_back(
                    m_sceneKeypoints.at(results.at<int>(i, 0)).pt);
                dst_point_idxs.push_back(results.at<int>(i, 0));
            }
        }
    } else {
        for (unsigned int i = 0; i < matches.size(); ++i) {
            // Apply NNDR
            log_trace("q=%d dist1=%f dist2=%f", matches.at(i).at(0).queryIdx,
                      matches.at(i).at(0).distance,
                      matches.at(i).at(1).distance);
            if (matches.at(i).size() == 2 &&
                matches.at(i).at(0).distance <=
                    nndrRatio * matches.at(i).at(1).distance) {
                src_points.push_back(
                    m_objectKeypoints.at(matches.at(i).at(0).queryIdx).pt);
                src_point_idxs.push_back(matches.at(i).at(0).queryIdx);

                dst_points.push_back(
                    m_sceneKeypoints.at(matches.at(i).at(0).trainIdx).pt);
                dst_point_idxs.push_back(matches.at(i).at(0).trainIdx);
            }
        }
    }
    log_debug("%d good matches found", (int)src_points.size());
    return true;
}

bool ObjectFinder::_FindHomography(vector<Point2f> &src_points,
                                   vector<Point2f> &dst_points,
                                   vector<uchar> &outlier_mask, Mat &H,
                                   unsigned int minInliers) {
    bool result = false;

    if (src_points.size() >= minInliers) {
        H = findHomography(src_points, dst_points, RANSAC, 1.0, outlier_mask);

        int inliers = 0, outliers = 0;
        for (unsigned int k = 0; k < src_points.size(); ++k) {
            if (outlier_mask.at(k)) {
                ++inliers;
            } else {
                ++outliers;
            }
        }
        log_debug("Inliers=%d Outliers=%d", inliers, outliers);
        // TODO: only if there is H
        // log_trace("H[0][0]=%f \tH[0][1]=%f \tH[0][2]=%f", H.at<double>(0, 0),
        // H.at<double>(0, 1), H.at<double>(0, 2)); log_trace("H[1][0]=%f
        // \tH[1][1]=%f \tH[1][2]=%f", H.at<double>(1, 0), H.at<double>(1, 1),
        // H.at<double>(1, 2));
        // log_trace("H[2][0]=%f \tH[2][1]=%f \tH[2][2]=%f", H.at<double>(2, 0),
        // H.at<double>(2, 1), H.at<double>(2, 2));
        result = true;
    } else {
        log_debug("Not enough matches (%d) for homography...",
                  (int)src_points.size());
    }
    return result;
}

Quadrilateral ObjectFinder::_GetObjectRectangle(cv::Mat &objectImg,
                                                cv::Mat &sceneImg, cv::Mat &H) {
    //-- Get the corners from the image_1 ( the object to be "detected" )
    Quadrilateral obj_corners(
        Point2f(0, 0), Point2f((float)objectImg.cols, 0),
        Point2f((float)objectImg.cols, (float)objectImg.rows),
        Point2f(0, (float)objectImg.rows));
    Quadrilateral scene_corners;

    // Catch exception if transformation is not possible
    try {
        perspectiveTransform(obj_corners.arr, scene_corners.arr, H);
    } catch (...) {
        return Quadrilateral();
    }

    return scene_corners;
}
