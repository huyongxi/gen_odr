#pragma once
#include <cmath>
#include <tuple>
#include <vector>

struct blPoint
{
    double x;
    double y;
    double z;
};

using PointVec = std::vector<blPoint>;

double distance(double x1, double y1, double x2, double y2);
double getPositiveHeading(double hdg);
double getDeltaHdg(double hdg1, double hdg2);
double getDeltaHdg(double hdg1, double hdg2);
double giveHeading(double x1, double y1, double x2, double y2);
std::tuple<double, double, double, double> schnittpunkt(double x1, double y1, double hdg1, double x2, double y2,
                                                        double hdg2);
std::tuple<double, double, double, double, double, double> getArcCurvatureAndLength(
    double xstart, double ystart, double x_end, double y_end, double x_curveMid, double y_curveMid,
    double maxerror = 0.8, double minradius = 0.5, int iterations = 10);