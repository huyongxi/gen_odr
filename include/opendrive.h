#pragma once
#include <tinyxml2.h>

#include <cstdint>
#include <ios>
#include <memory>
#include <string>
#include <vector>

#include "arc_curves.h"

using std::shared_ptr;
using std::string;
using std::vector;

using ID = uint64_t;

struct BaseLine
{
    double s;
    double x;
    double y;
    double hdg;  //航向角
    double length;
    virtual ~BaseLine() = default;
    virtual tinyxml2::XMLElement* to_geometry_xml(tinyxml2::XMLDocument&) { return nullptr; }
    static tinyxml2::XMLDocument doc;
};

//直线
struct StraightLine : public BaseLine
{
    virtual tinyxml2::XMLElement* to_geometry_xml(tinyxml2::XMLDocument& doc) override;
};

//圆弧
struct ArcLine : public BaseLine
{
    double curvature;  //曲率
    virtual tinyxml2::XMLElement* to_geometry_xml(tinyxml2::XMLDocument& doc) override;
};

//参考线
struct RefLine
{
    vector<shared_ptr<BaseLine>> reflines;
    tinyxml2::XMLElement* to_planView_xml(tinyxml2::XMLDocument& doc);
};

//车道
class Lane
{
   public:
    ID id_;
    string type_;
    shared_ptr<RefLine> refline_ptr_;
    double width_;

   public:
    tinyxml2::XMLElement* to_lane_xml(tinyxml2::XMLDocument& doc);
};

//道路
class Road
{
   private:
    ID id_;
    ID junction_id_;
    string name_;
    string rule_;
    double length_;
    shared_ptr<RefLine> refline_ptr_;
    vector<Lane> left_lanes_;
    vector<Lane> right_lanes_;

   public:
    Road(const PointVec& refline_points);
    tinyxml2::XMLElement* to_road_xml(tinyxml2::XMLDocument& doc);
    void fit(const PointVec& refline_points);
    void add_lane(double width);

   public:
};

struct Header
{
    string revMajor;
    string revMinor;
    string name = "Gen OpenDRIVE";
    string version;
    string date;
    double north;
    double south;
    double east;
    double west;
    string vendor = "Gen OpenDRIVE";
};

class OpenDrive
{
   private:
    tinyxml2::XMLDocument doc_;
    Header header_;
    vector<Road> roads_;

   public:
    void to_xml(const string& filename);
    void add_road(const PointVec& refline_points);
};
