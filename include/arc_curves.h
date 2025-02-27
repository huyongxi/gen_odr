#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <tuple>
#include <vector>

using std::vector;
using namespace Eigen;

using PointVec = vector<Vector3d>;

extern double epsilon;

inline double distance(double x1, double y1, double x2, double y2)
{
    return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}
double getPositiveHeading(double hdg);
double getDeltaHdg(double hdg1, double hdg2);
double giveHeading(double x1, double y1, double x2, double y2);
std::tuple<double, double, double, double> schnittpunkt(double x1, double y1, double hdg1, double x2, double y2,
                                                        double hdg2);
std::tuple<double, double, double, double, double, double> getArcCurvatureAndLength(
    double xstart, double ystart, double x_end, double y_end, double x_curveMid, double y_curveMid,
    double maxerror = 0.1, double minradius = 0.5, int iterations = 1000);
std::tuple<double, double, double> getArcEndPosition(double curvature, double length, double xstart, double ystart,
                                                     double hdg_start);
Vector4d fitCubicSpline(const vector<Vector2d>& points);
inline double computeDistance(const Vector2d& point, const Vector4d& coeff);
void fitLaneWidth(const vector<Vector2d>& points, double threshold, vector<std::pair<double, Vector4d>>& widths);
Vector2d fitLinear(PointVec::const_iterator begin, PointVec::const_iterator end);
