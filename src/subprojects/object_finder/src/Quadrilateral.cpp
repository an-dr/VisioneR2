#include "Quadrilateral.hpp"
#include <cmath>

Quadrilateral::Quadrilateral(Point2f p1, Point2f p2, Point2f p3, Point2f p4)
    : arr({p1, p2, p3, p4}) {}

Point2f Quadrilateral::GetCenter() {
    return (arr[0] + arr[1] + arr[2] + arr[3]) / 4;
}

array<float, 4> Quadrilateral::GetSides() {

    return {_GetDistance(arr[0], arr[1]), _GetDistance(arr[1], arr[2]),
            _GetDistance(arr[2], arr[3]), _GetDistance(arr[3], arr[0])};
}

float Quadrilateral::GetArea() {
    array<float, 4> s = GetSides();
    float semip = (s[0] + s[1] + s[2] + s[3]) / 2;
    float result =
        sqrt((semip - s[0]) * (semip - s[1]) * (semip - s[2]) * (semip - s[3]));
    return isnan(result) ? 0 : result;
}

Point2f &Quadrilateral::operator[](int i) { return arr[i]; }

const Point2f &Quadrilateral::operator[](int i) const { return arr[i]; }

float Quadrilateral::GetPerimeter() {
    array<float, 4> sides = GetSides();
    return sides[0] + sides[1] + sides[2] + sides[3];
}

vector<Point2i> Quadrilateral::GetIntegerPoints() {
    vector<Point2i> res;
    for (auto p : arr) {
        res.push_back(p);
    }
    return res;
}

float Quadrilateral::_GetDistance(Point2f &p_a, Point2f &p_b) {
    return sqrt((p_a.x - p_b.x) * (p_a.x - p_b.x) +
                (p_a.y - p_b.y) * (p_a.y - p_b.y));
}
