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
    virtual tinyxml2::XMLElement* to_geometry_xml(tinyxml2::XMLDocument&) { return nullptr; }  //生成xml节点
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

struct RefLinePoint
{
    double x;
    double y;
    double s;
    bool operator< (double other) const
    {
        return s < other;
    }
};

//参考线
struct RefLine
{
    vector<shared_ptr<BaseLine>> reflines;
    double fit(const PointVec& refline_points);
    tinyxml2::XMLElement* to_planView_xml(tinyxml2::XMLDocument& doc);
    void sample(vector<RefLinePoint>& points, double step = 1);
};


struct LaneWidth
{
    double sOffset;
    double a;
    double b;
    double c;
    double d;
};

//车道
class Lane
{
   private:
    ID id_;
    string type_;
    vector<LaneWidth> lane_widths_;
    shared_ptr<RefLine> refline_ptr_;
    shared_ptr<RefLine> left_border_ptr_;
    shared_ptr<RefLine> right_border_ptr_;

   public:
    Lane(ID id, string type, shared_ptr<RefLine> refline_ptr) : id_(id), type_(type), refline_ptr_(refline_ptr)
    {
        left_border_ptr_ = std::make_shared<RefLine>();
        right_border_ptr_ = std::make_shared<RefLine>();
    }

    ID get_id() { return id_; }

    void set_left_border(const PointVec& points) { left_border_ptr_->fit(points); }

    // void set_right_border(const PointVec& points)
    // {
    //     right_border_ptr_->fit(points);
    // }

    //拟合车道宽度
    void fit_lane_width();

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
    void add_lane(const PointVec& left_border, ID id = 1, string type = "driving");

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
    Road& add_road(const PointVec& refline_points);
};
