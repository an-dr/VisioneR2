// *************************************************************************
//
// Copyright (c) 2024 Andrei Gramakov. All rights reserved.
//
// site:    https://agramakov.me
// e-mail:  mail@agramakov.me
//
// *************************************************************************
#pragma once

#include <array>
#include <vector>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

class Quadrilateral {
 public:
    Quadrilateral() = default;
    Quadrilateral(Point2f p1, Point2f p2, Point2f p3, Point2f p4);
    Point2f GetCenter();
    array<float, 4> GetSides();
    float GetArea();
    float GetPerimeter();
    vector<Point2i> GetIntegerPoints();

    // overload operator []
    Point2f &operator[](int i);
    const Point2f &operator[](int i) const;

    array<Point2f, 4> arr;

 private:
    float _GetDistance(Point2f &p_a, Point2f &p_b);
};
