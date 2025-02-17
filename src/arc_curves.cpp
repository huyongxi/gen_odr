#include "arc_curves.h"

#include <algorithm>
#include <cassert>
#include <tuple>

double distance(double x1, double y1, double x2, double y2)
{
    return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

//
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

    return deltaHdg;
}

double giveHeading(double x1, double y1, double x2, double y2)
{
    // assert(!(x1 == x2 && y1 == y2));  // 确保 x1, y1 和 x2, y2 不相同

    double x_arr[2] = {x1, x2};
    double y_arr[2] = {y1, y2};

    x_arr[1] -= x_arr[0];
    y_arr[1] -= y_arr[0];

    double phi;
    if (x_arr[1] > 0)
    {
        phi = std::atan(y_arr[1] / x_arr[1]);
    } else if (x_arr[1] == 0)
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
        // r2 calculation
        r2 = (y1 * std::cos(hdg1) + std::sin(hdg1) * (x2 - x1) - y2 * std::cos(hdg1)) /
             (std::sin(hdg2) * std::cos(hdg1) - std::sin(hdg1) * std::cos(hdg2));

        if (std::abs(std::abs(hdg1) - PI_2) < PI_2_THRESHOLD)
        {
            // r1 calculation when hdg1 is near 90° or -90°
            r1 = (y2 - y1 + std::sin(hdg2) * r2) / std::sin(hdg1);
        } else
        {
            // r1 calculation for general case
            r1 = (x2 - x1 + std::cos(hdg2) * r2) / std::cos(hdg1);
        }
    } else
    {
        // r1 calculation
        r1 = (-y1 * std::cos(hdg2) + y2 * std::cos(hdg2) + std::sin(hdg2) * x1 - std::sin(hdg2) * x2) /
             (std::sin(hdg1) * std::cos(hdg2) - std::sin(hdg2) * std::cos(hdg1));

        if (std::abs(std::abs(hdg2) - PI_2) < PI_2_THRESHOLD)
        {
            // r2 calculation when hdg2 is near 90° or -90°
            r2 = (y1 - y2 + std::sin(hdg1) * r1) / std::sin(hdg2);
        } else
        {
            // r2 calculation for general case
            r2 = (x1 - x2 + std::cos(hdg1) * r1) / std::cos(hdg2);
        }
    }

    // Calculate intersection point (x_s, y_s)
    double x_s = x1 + std::cos(hdg1) * r1;
    double y_s = y1 + std::sin(hdg1) * r1;

    return {x_s, y_s, r1, r2};  // Return intersection point and distances
}

std::tuple<double, double, double, double, double, double> getArcCurvatureAndLength(double xstart, double ystart,
                                                                                    double x_end, double y_end,
                                                                                    double x_curveMid,
                                                                                    double y_curveMid, double maxerror,
                                                                                    double minradius, int iterations)
{
    double hdg_start = giveHeading(xstart, ystart, x_curveMid, y_curveMid);
    double hdg_mid2end = giveHeading(x_curveMid, y_curveMid, x_end, y_end);

    // assert(hdg_start != hdg_mid2end && "The directions have to be different!");

    double deltaHdg = getDeltaHdg(hdg_start, hdg_mid2end);
    double winkelHalbHdg = deltaHdg / 2.0 + hdg_start;

    double maxDist =
        std::min(distance(x_curveMid, y_curveMid, xstart, ystart), distance(x_curveMid, y_curveMid, x_end, y_end));

    if (std::abs(deltaHdg) < 0.0001)
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
