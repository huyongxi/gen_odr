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

// 计算两点之间的距离
inline double distance(double x1, double y1, double x2, double y2)
{
    return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}
// 获取正向航向角
double getPositiveHeading(double hdg);
// 获取两个航向角之间的差值
double getDeltaHdg(double hdg1, double hdg2);
// 计算从点(x1, y1)到点(x2, y2)的航向角
double giveHeading(double x1, double y1, double x2, double y2);
// 计算两条线的交点
std::tuple<double, double, double, double> schnittpunkt(double x1, double y1, double hdg1, double x2, double y2,
                                                        double hdg2);
// 计算圆弧的曲率和长度
std::tuple<double, double, double, double, double, double> getArcCurvatureAndLength(
    double xstart, double ystart, double x_end, double y_end, double x_curveMid, double y_curveMid,
    double maxerror = 0.1, double minradius = 0.5, int iterations = 1000);
// 计算圆弧的终点位置
std::tuple<double, double, double> getArcEndPosition(double curvature, double length, double xstart, double ystart,
                                                     double hdg_start);
// 拟合三次曲线
Vector4d fitCubicSpline(const vector<Vector2d>& points);
// 计算点到曲线的距离
inline double computeDistance(const Vector2d& point, const Vector4d& coeff);
// 拟合车道宽度
void fitLaneWidth(const vector<Vector2d>& points, double threshold, vector<std::pair<double, Vector4d>>& widths);
// 拟合线性曲线
Vector2d fitLinear(PointVec::const_iterator begin, PointVec::const_iterator end);
