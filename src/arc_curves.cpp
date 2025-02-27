#include "arc_curves.h"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <utility>
#include <vector>

double epsilon = 0.00001;

double getPositiveHeading(double hdg)
{
    while (hdg < 0.0)
    {
        hdg += 2.0 * M_PI;
    }
    return fmod(hdg, 2.0 * M_PI);  // fmod 用于计算浮动数的模
}

double getDeltaHdg(double hdg1, double hdg2)
{
    hdg1 = getPositiveHeading(hdg1);
    hdg2 = getPositiveHeading(hdg2);

    double deltaHdg = hdg1 - hdg2;

    if (int(std::abs(deltaHdg) / M_PI) == 1)
    {
        deltaHdg = std::fmod(std::abs(deltaHdg), M_PI) * std::copysign(1.0, deltaHdg);
        if (deltaHdg < 0.0)
        {
            deltaHdg = M_PI + deltaHdg;
        } else
        {
            deltaHdg = -M_PI + deltaHdg;
        }
    } else
    {
        deltaHdg = std::fmod(std::abs(deltaHdg), M_PI) * std::copysign(1.0, deltaHdg);
    }
    return deltaHdg;
}

double giveHeading(double x1, double y1, double x2, double y2)
{
    assert(!(std::abs(x1 - x2) > epsilon && std::abs(y1 - y2) < epsilon));  // 确保 x1, y1 和 x2, y2 不相同

    double x_arr[2] = {x1, x2};
    double y_arr[2] = {y1, y2};

    x_arr[1] -= x_arr[0];
    y_arr[1] -= y_arr[0];

    double phi;
    if (x_arr[1] > 0)
    {
        phi = std::atan(y_arr[1] / x_arr[1]);
    } else if (std::abs(x_arr[1]) <= epsilon)
    {
        if (y_arr[1] > 0)
        {
            phi = M_PI / 2;
        } else
        {
            phi = -M_PI / 2;
        }
    } else
    {
        if (y_arr[1] >= 0)
        {
            phi = std::atan(y_arr[1] / x_arr[1]) + M_PI;
        } else
        {
            phi = std::atan(y_arr[1] / x_arr[1]) - M_PI;
        }
    }

    return getPositiveHeading(phi);
}

std::tuple<double, double, double, double> schnittpunkt(double x1, double y1, double hdg1, double x2, double y2,
                                                        double hdg2)
{
    const double EPSILON = 0.02;
    const double PI_2 = M_PI / 2.0;
    const double PI_2_THRESHOLD = 0.2;

    double r1{};
    double r2{};

    if (std::abs(std::sin(hdg1) * std::cos(hdg2) - std::sin(hdg2) * std::cos(hdg1)) < EPSILON)
    {
        r2 = (y1 * std::cos(hdg1) + std::sin(hdg1) * (x2 - x1) - y2 * std::cos(hdg1)) /
             (std::sin(hdg2) * std::cos(hdg1) - std::sin(hdg1) * std::cos(hdg2));

        if (std::abs(std::abs(hdg1) - PI_2) < PI_2_THRESHOLD)
        {
            r1 = (y2 - y1 + std::sin(hdg2) * r2) / std::sin(hdg1);
        } else
        {
            r1 = (x2 - x1 + std::cos(hdg2) * r2) / std::cos(hdg1);
        }
    } else
    {
        r1 = (-y1 * std::cos(hdg2) + y2 * std::cos(hdg2) + std::sin(hdg2) * x1 - std::sin(hdg2) * x2) /
             (std::sin(hdg1) * std::cos(hdg2) - std::sin(hdg2) * std::cos(hdg1));

        if (std::abs(std::abs(hdg2) - PI_2) < PI_2_THRESHOLD)
        {
            r2 = (y1 - y2 + std::sin(hdg1) * r1) / std::sin(hdg2);
        } else
        {
            r2 = (x1 - x2 + std::cos(hdg1) * r1) / std::cos(hdg2);
        }
    }

    double x_s = x1 + std::cos(hdg1) * r1;
    double y_s = y1 + std::sin(hdg1) * r1;

    return {x_s, y_s, r1, r2};
}

std::tuple<double, double, double, double, double, double> getArcCurvatureAndLength(double xstart, double ystart,
                                                                                    double x_end, double y_end,
                                                                                    double x_curveMid,
                                                                                    double y_curveMid, double maxerror,
                                                                                    double minradius, int iterations)
{
    double hdg_start = giveHeading(xstart, ystart, x_curveMid, y_curveMid);
    double hdg_mid2end = giveHeading(x_curveMid, y_curveMid, x_end, y_end);

    double deltaHdg = getDeltaHdg(hdg_start, hdg_mid2end);
    double winkelHalbHdg = deltaHdg / 2.0 + hdg_start;

    double maxDist =
        std::min(distance(x_curveMid, y_curveMid, xstart, ystart), distance(x_curveMid, y_curveMid, x_end, y_end));

    if (std::abs(deltaHdg) < epsilon)
    {
        return {xstart, ystart, x_end, y_end, 0.0, distance(xstart, ystart, x_end, y_end)};
    }

    double hdg_90_a = hdg_start - M_PI / 2.0;
    double hdg_90_b = hdg_mid2end - M_PI / 2.0;

    double dist = maxDist;
    double bestDist = dist;
    double notWorkingDist = dist;
    double x1, y1, x2, y2, r1, r2, x_s, y_s;

    x1 = x_curveMid + dist * std::cos(hdg_start - M_PI);
    y1 = y_curveMid + dist * std::sin(hdg_start - M_PI);
    x2 = x_curveMid + dist * std::cos(hdg_mid2end);
    y2 = y_curveMid + dist * std::sin(hdg_mid2end);
    std::tie(x_s, y_s, r1, r2) = schnittpunkt(x1, y1, hdg_90_a, x2, y2, hdg_90_b);

    double error = distance(x_s, y_s, x_curveMid, y_curveMid) - std::abs(r1);

    if (error < maxerror)
    {
        // Do nothing, pass if the error is acceptable
    } else
    {
        for (int i = 0; i < iterations; ++i)
        {
            dist = (bestDist + notWorkingDist) / 2.0;
            x1 = x_curveMid + dist * std::cos(hdg_start - M_PI);
            y1 = y_curveMid + dist * std::sin(hdg_start - M_PI);
            x2 = x_curveMid + dist * std::cos(hdg_mid2end);
            y2 = y_curveMid + dist * std::sin(hdg_mid2end);
            std::tie(x_s, y_s, r1, r2) = schnittpunkt(x1, y1, hdg_90_a, x2, y2, hdg_90_b);

            error = distance(x_s, y_s, x_curveMid, y_curveMid) - std::abs(r1);
            if (error < maxerror)
            {
                bestDist = dist;
            } else
            {
                notWorkingDist = dist;
            }
        }
    }

    x1 = x_curveMid + bestDist * std::cos(hdg_start - M_PI);
    y1 = y_curveMid + bestDist * std::sin(hdg_start - M_PI);
    x2 = x_curveMid + bestDist * std::cos(hdg_mid2end);
    y2 = y_curveMid + bestDist * std::sin(hdg_mid2end);

    std::tie(x_s, y_s, r1, r2) = schnittpunkt(x1, y1, hdg_90_a, x2, y2, hdg_90_b);

    double length = std::abs(r1) * std::abs(deltaHdg);
    double curvature = -deltaHdg / length;

    return {x1, y1, x2, y2, curvature, length};
}

std::tuple<double, double, double> getArcEndPosition(double curvature, double length, double xstart, double ystart,
                                                     double hdg_start)
{
    if (std::abs(length) < epsilon)
    {
        return {xstart, ystart, hdg_start};
    }
    double deltaHdg = curvature * length;
    double hdg_end = deltaHdg + hdg_start;

    double x_end, y_end;

    if (std::abs(curvature) > epsilon)
    {
        double radius = length / deltaHdg;
        double x_M, y_M;

        if (curvature < 0.0)
        {
            x_M = std::cos(hdg_start + M_PI / 2.0) * radius + xstart;
            y_M = std::sin(hdg_start + M_PI / 2.0) * radius + ystart;
            x_end = std::cos(hdg_start - M_PI / 2.0 + deltaHdg) * radius + x_M;
            y_end = std::sin(hdg_start - M_PI / 2.0 + deltaHdg) * radius + y_M;
        } else
        {
            x_M = std::cos(hdg_start + M_PI / 2.0) * radius + xstart;
            y_M = std::sin(hdg_start + M_PI / 2.0) * radius + ystart;
            x_end = std::cos(hdg_start - M_PI / 2.0 + deltaHdg) * radius + x_M;
            y_end = std::sin(hdg_start - M_PI / 2.0 + deltaHdg) * radius + y_M;
        }
    } else
    {
        x_end = std::cos(hdg_start) * length + xstart;
        y_end = std::sin(hdg_start) * length + ystart;
    }

    return {x_end, y_end, getPositiveHeading(hdg_end)};
}

// 一元线性曲线拟合函数
Vector2d fitLinear(PointVec::const_iterator begin, PointVec::const_iterator end)
{
    int n = end - begin;
    MatrixXd A(n, 2);
    VectorXd b(n);

    int i = 0;
    for (auto iter = begin; iter != end; ++iter, ++i)
    {
        double x = (*iter)[0];
        A(i, 0) = 1;
        A(i, 1) = x;
        b(i) = (*iter)[1];
    }

    Vector2d coeff = A.colPivHouseholderQr().solve(b);
    return coeff;
}

// 一元三次曲线拟合函数
Vector4d fitCubicSpline(const vector<Vector2d>& points)
{
    int n = points.size();
    MatrixXd A(n, 4);
    VectorXd b(n);

    // 构造方程 Ax = b，其中A为设计矩阵，b为结果向量
    for (int i = 0; i < n; ++i)
    {
        double x = points[i][0];
        A(i, 0) = 1;
        A(i, 1) = x;
        A(i, 2) = x * x;
        A(i, 3) = x * x * x;
        b(i) = points[i][1];
    }

    // 使用最小二乘法解线性方程 Ax = b，得到拟合的系数
    Vector4d coeff = A.colPivHouseholderQr().solve(b);
    return coeff;
}

// 计算点到拟合曲线的距离
double computeDistance(const Vector2d& point, const Vector4d& coeff)
{
    double x = point[0];
    // 曲线方程 y = a + b*x + c*x^2 + d*x^3
    double y_fit = coeff[0] + coeff[1] * x + coeff[2] * x * x + coeff[3] * x * x * x;
    return std::abs(y_fit - point[1]);
}

// 拟合车道宽度
void fitLaneWidth(const vector<Vector2d>& points, double threshold, vector<std::pair<double, Vector4d>>& widths)
{
    double s_start = 0.0;
    int base = 0;
    std::pair<double, Vector4d> best_fit;
    while (base < points.size() - 1)
    {
        int i = 1;
        for (; i < points.size() - base; ++i)
        {
            vector<Vector2d> sub_points(points.begin() + base, points.begin() + base + i + 1);
            std::for_each(sub_points.begin(), sub_points.end(), [=](Vector2d& p) { p[0] -= s_start; });
            Vector4d cubic_coeff = fitCubicSpline(sub_points);
            double max_cubic_error = 0.0;
            for (const auto& sub_point : sub_points)
            {
                auto dis = computeDistance(sub_point, cubic_coeff);
                if (dis > max_cubic_error)
                {
                    max_cubic_error = dis;
                }
            }

            if (max_cubic_error <= threshold)
            {
                best_fit = {s_start, cubic_coeff};
                continue;
            }
            break;
        }
        base += i;
        s_start = (points.begin() + base)->x();
        widths.push_back(best_fit);
    }
}
