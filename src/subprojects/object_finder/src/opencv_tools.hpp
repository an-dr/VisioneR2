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

using namespace cv;

Ptr<DescriptorExtractor> getDescriptorExtractor();
Ptr<FeatureDetector> getFeatureDetector();
