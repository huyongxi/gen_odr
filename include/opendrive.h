#pragma once
#include <tinyxml2.h>

#include <cstdint>
#include <ios>
#include <list>
#include <memory>
#include <string>
#include <vector>

#include "arc_curves.h"

using std::list;
using std::shared_ptr;
using std::string;
using std::vector;

using ID = int64_t;

namespace ODR
{

ID GetId();

struct BaseLine
{
    double s;
    double x;
    double y;
    double hdg;  //航向角
    double length;
    virtual ~BaseLine() = default;
    virtual tinyxml2::XMLElement* to_geometry_xml(tinyxml2::XMLDocument&)
    {
        return nullptr;
    }  //生成xml节点
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
    double                        curvature;  //曲率
    virtual tinyxml2::XMLElement* to_geometry_xml(tinyxml2::XMLDocument& doc) override;
};

struct RefLinePoint
{
    double x;
    double y;
    double s;
    double hdg;
};

//参考线
struct RefLine
{
    vector<shared_ptr<BaseLine>> reflines;
    double                       fit(const PointVec& refline_points);
    // 转换为XML元素
    tinyxml2::XMLElement* to_planView_xml(tinyxml2::XMLDocument& doc);
    // 采样参考线点
    void sample(vector<RefLinePoint>& points, double step = 0.2);
};

struct LaneWidth
{
    double sOffset;
    double a;
    double b;
    double c;
    double d;
};

struct LaneLink
{
    ID predecessor_id{};
    ID successor_id{};
};

//车道
class Lane
{
   private:
    ID                  id_;
    string              type_;
    vector<LaneWidth>   lane_widths_;
    shared_ptr<RefLine> refline_ptr_;
    shared_ptr<RefLine> left_border_ptr_;
    shared_ptr<RefLine> right_border_ptr_;
    LaneLink            link_;
    friend class Road;

   public:
    Lane(ID id, string type, shared_ptr<RefLine> refline_ptr) : id_(id), type_(type), refline_ptr_(refline_ptr)
    {
        left_border_ptr_  = std::make_shared<RefLine>();
        right_border_ptr_ = std::make_shared<RefLine>();
    }

    ID get_id()
    {
        return id_;
    }

    // 设置左边界，使用提供的点进行拟合
    void set_left_border(const PointVec& points)
    {
        left_border_ptr_->fit(points);
    }

    // void set_right_border(const PointVec& points)
    // {
    //     right_border_ptr_->fit(points);
    // }

    //拟合车道宽度
    void fit_lane_width();
    // 设置车道宽度为指定值
    void set_lane_width(double width);
    // 将当前车道连接到另一个车道
    void connect_to(Lane& lane);
    // 将当前车道转换为XML元素
    tinyxml2::XMLElement* to_lane_xml(tinyxml2::XMLDocument& doc);
};

struct RoadLink
{
    ID     predecessor_id{};
    string predecessor_type;
    string predecessor_contact_point;
    ID     successor_id{};
    string successor_type;
    string successor_contact_point;
};

//道路
class Road
{
   private:
    ID                  id_;
    ID                  junction_id_{-1};
    string              name_;
    string              rule_{"LHT"};
    double              length_;
    shared_ptr<RefLine> refline_ptr_;
    vector<Lane>        left_lanes_;
    vector<Lane>        right_lanes_;
    RoadLink            link_;
    friend class Junction;

   public:
    Road(const PointVec& refline_points);
    // 将道路转换为XML元素
    tinyxml2::XMLElement* to_road_xml(tinyxml2::XMLDocument& doc);
    // 添加车道，使用左边界点和类型
    void add_lane(const PointVec& left_border, string type = "driving");
    // 添加车道，使用宽度和类型
    void add_lane(double width, string type = "driving");
    // 将当前道路连接到另一个道路
    Road& connect_to(Road& road);
    Road& operator>>(Road& road)
    {
        return connect_to(road);
    }
    // 通过交叉口连接道路
    void connect_by_junction(Road& road, ID junction_id);
};

class Junction
{
   private:
    ID     id;
    string name;
    struct Link
    {
        ID     incoming_road;
        ID     connecting_road;
        string contact_point{"start"};
        ID     from;
        ID     to;
    };
    vector<Link> links;

   public:
    Junction() : id(GetId()), name("Junction" + std::to_string(id)) {}
    // 将交叉口转换为XML元素
    tinyxml2::XMLElement* to_junction_xml(tinyxml2::XMLDocument& doc);
    // 添加连接道路
    void add_connect_road(Road& incoming_road, Road& connecting_road);
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
    Header                header_;
    list<Road>            roads_;
    list<Junction>        junctions_;

   public:
    OpenDrive() {}
    // 将OpenDrive对象转换为XML文件
    void to_xml(const string& filename);
    // 添加道路
    Road& add_road(const PointVec& refline_points);
    // 添加交叉口
    Junction& add_junction();
};

}  // namespace ODR