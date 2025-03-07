#include "opendrive.h"

#include <tinyxml2.h>

#include <atomic>
#include <iostream>
#include <limits>
#include <tuple>
#include <utility>
#include <vector>

#include "arc_curves.h"

namespace ODR
{

ID GetId()
{
    static std::atomic<ID> id{0};
    return ++id;
}

tinyxml2::XMLElement* StraightLine::to_geometry_xml(tinyxml2::XMLDocument& doc)
{
    tinyxml2::XMLElement* geometry = doc.NewElement("geometry");
    geometry->SetAttribute("s", s);
    geometry->SetAttribute("x", x);
    geometry->SetAttribute("y", y);
    geometry->SetAttribute("hdg", hdg);
    geometry->SetAttribute("length", length);
    tinyxml2::XMLElement* line = doc.NewElement("line");
    geometry->InsertEndChild(line);
    return geometry;
}

tinyxml2::XMLElement* ArcLine::to_geometry_xml(tinyxml2::XMLDocument& doc)
{
    tinyxml2::XMLElement* geometry = doc.NewElement("geometry");
    geometry->SetAttribute("s", s);
    geometry->SetAttribute("x", x);
    geometry->SetAttribute("y", y);
    geometry->SetAttribute("hdg", hdg);
    geometry->SetAttribute("length", length);
    tinyxml2::XMLElement* arc = doc.NewElement("arc");
    arc->SetAttribute("curvature", curvature);
    geometry->InsertEndChild(arc);
    return geometry;
}

tinyxml2::XMLElement* RefLine::to_planView_xml(tinyxml2::XMLDocument& doc)
{
    tinyxml2::XMLElement* planView = doc.NewElement("planView");
    for (auto& line : reflines)
    {
        planView->InsertEndChild(line->to_geometry_xml(doc));
    }
    return planView;
}

double RefLine::fit(const PointVec& refline_points)
{
    assert(refline_points.size() > 1);

    if (refline_points.size() == 2)
    {
        auto line = std::make_shared<StraightLine>();
        line->s   = 0.0;
        line->x   = refline_points[0].x();
        line->y   = refline_points[0].y();
        line->hdg =
            giveHeading(refline_points[0].x(), refline_points[0].y(), refline_points[1].x(), refline_points[1].y());
        line->length =
            distance(refline_points[0].x(), refline_points[0].y(), refline_points[1].x(), refline_points[1].y());
        reflines.push_back(line);
        return line->length;
    }

    double s = 0.0;

    for (int i = 0; i < refline_points.size() - 2; ++i)
    {
        double x1  = refline_points[i].x();
        double y1  = refline_points[i].y();
        double x2  = refline_points[i + 1].x();
        double y2  = refline_points[i + 1].y();
        double x3  = refline_points[i + 2].x();
        double y3  = refline_points[i + 2].y();
        double hdg = giveHeading(x1, y1, x2, y2);

        if (i != 0)
        {
            x1 = (x1 + x2) / 2.0;
            y1 = (y1 + y2) / 2.0;
        }

        if (i != refline_points.size() - 3)
        {
            x3 = (x3 + x2) / 2.0;
            y3 = (y3 + y2) / 2.0;
        }

        double xarc, yarc, xendline, yendline, curvature, length;
        std::tie(xarc, yarc, xendline, yendline, curvature, length) = getArcCurvatureAndLength(x1, y1, x3, y3, x2, y2);

        if (distance(x1, y1, xarc, yarc) > 0.1)
        {
            // std::cout << "xstart: " << x1 << " ystart: " << y1 << " length: " << distance(x1, y1, xarc, yarc)
            //           << " heading: " << hdg << " curvature: " << 0.0 << std::endl;

            auto line    = std::make_shared<StraightLine>();
            line->s      = s;
            line->x      = x1;
            line->y      = y1;
            line->hdg    = hdg;
            line->length = distance(x1, y1, xarc, yarc);
            s += line->length;
            reflines.push_back(line);
        }

        // std::cout << "xstart: " << xarc << " ystart: " << yarc << " length: " << length << " heading: " << hdg
        //           << " curvature: " << curvature << std::endl;

        if (std::abs(curvature) < epsilon)
        {
            auto line    = std::make_shared<StraightLine>();
            line->s      = s;
            line->x      = xarc;
            line->y      = yarc;
            line->hdg    = hdg;
            line->length = length;
            s += line->length;
            reflines.push_back(line);
        } else
        {
            auto arc       = std::make_shared<ArcLine>();
            arc->s         = s;
            arc->x         = xarc;
            arc->y         = yarc;
            arc->hdg       = hdg;
            arc->length    = length;
            arc->curvature = curvature;
            s += arc->length;
            reflines.push_back(arc);
        }

        if (distance(xendline, yendline, x3, y3) > 0.1)
        {
            // std::cout << "xstart: " << xendline << " ystart: " << yendline
            //           << " length: " << distance(xendline, yendline, x3, y3)
            //           << " heading: " << giveHeading(xendline, yendline, x3, y3) << " curvature: " << 0.0 <<
            //           std::endl;

            auto line    = std::make_shared<StraightLine>();
            line->s      = s;
            line->x      = xendline;
            line->y      = yendline;
            line->hdg    = giveHeading(xendline, yendline, x3, y3);
            line->length = distance(xendline, yendline, x3, y3);
            s += line->length;
            reflines.push_back(line);
        }
    }

    return s;
}

void RefLine::sample(vector<RefLinePoint>& points, double step)
{
    double s_sum   = 0.0;
    double s_start = 0.0;
    for (auto& line : reflines)
    {
        if (auto straight_line = std::dynamic_pointer_cast<StraightLine>(line))
        {
            double s = s_start;
            for (; s < straight_line->length; s += step)
            {
                RefLinePoint point;
                double       hdg;
                std::tie(point.x, point.y, hdg) =
                    getArcEndPosition(0.0, s, straight_line->x, straight_line->y, straight_line->hdg);
                point.s   = s_sum;
                point.hdg = hdg;
                s_sum += step;
                points.push_back(point);
            }
            s_start = s - straight_line->length;

        } else if (auto arc = std::dynamic_pointer_cast<ArcLine>(line))
        {
            double s = s_start;
            for (; s < arc->length; s += step)
            {
                RefLinePoint point;
                double       hdg;
                std::tie(point.x, point.y, hdg) = getArcEndPosition(arc->curvature, s, arc->x, arc->y, arc->hdg);
                point.s                         = s_sum;
                point.hdg                       = hdg;
                s_sum += step;
                points.push_back(point);
            }
            s_start = s - arc->length;
        }
    }
}

tinyxml2::XMLElement* Lane::to_lane_xml(tinyxml2::XMLDocument& doc)
{
    tinyxml2::XMLElement* lane = doc.NewElement("lane");
    lane->SetAttribute("id", id_);
    lane->SetAttribute("type", type_.c_str());
    lane->SetAttribute("level", "false");
    auto link = doc.NewElement("link");
    if (link_.predecessor_id > 0)
    {
        auto predecessor = doc.NewElement("predecessor");
        predecessor->SetAttribute("id", link_.predecessor_id);
        link->InsertEndChild(predecessor);
    }
    if (link_.successor_id > 0)
    {
        auto successor = doc.NewElement("successor");
        successor->SetAttribute("id", link_.successor_id);
        link->InsertEndChild(successor);
    }
    lane->InsertEndChild(link);
    for (auto& width : lane_widths_)
    {
        auto width_xml = doc.NewElement("width");
        width_xml->SetAttribute("sOffset", width.sOffset);
        width_xml->SetAttribute("a", width.a);
        width_xml->SetAttribute("b", width.b);
        width_xml->SetAttribute("c", width.c);
        width_xml->SetAttribute("d", width.d);
        lane->InsertEndChild(width_xml);
    }
    return lane;
}

void Lane::connect_to(Lane& lane)
{
    link_.successor_id        = lane.id_;
    lane.link_.predecessor_id = id_;
}

inline double calc_distance(const Vector2d& s, double shdg, const Vector2d& p)
{
    double dx = std::cos(shdg);
    double dy = std::sin(shdg);
    return std::abs(dx * (p.x() - s.x()) + dy * (p.y() - s.y())) / std::sqrt(dx * dx + dy * dy);
}

void calc_width(const vector<RefLinePoint>& ref, const vector<RefLinePoint>& border, vector<Vector2d>& s_width_pairs)
{
    for (const auto& rp : ref)
    {
        double   min_distance = 10000.0;
        Vector2d min_p;

        for (const auto& p : border)
        {
            double dis = calc_distance({rp.x, rp.y}, rp.hdg, {p.x, p.y});
            if (dis < min_distance)
            {
                min_distance = dis;
                min_p        = {p.x, p.y};
            }
        }

        if (min_distance > 0.2)
        {
            min_distance = 0;
        } else
        {
            min_distance = distance(rp.x, rp.y, min_p.x(), min_p.y());
        }

        s_width_pairs.push_back({rp.s, min_distance});
    }
}

void Lane::fit_lane_width()
{
    vector<RefLinePoint> refline_sample_points;
    refline_ptr_->sample(refline_sample_points);

    vector<RefLinePoint> left_border_sample_points;
    left_border_ptr_->sample(left_border_sample_points);

    vector<Vector2d> s_width_pairs;

    calc_width(refline_sample_points, left_border_sample_points, s_width_pairs);
    auto pred  = [](const Vector2d& v) { return std::abs(v[1]) > epsilon ? true : false; };
    auto iter1 = std::find_if(s_width_pairs.begin(), s_width_pairs.end(), pred);
    auto iter2 = std::find_if(s_width_pairs.rbegin(), s_width_pairs.rend(), pred);

    for (auto iter = s_width_pairs.begin(); iter != iter1; ++iter)
    {
        iter->y() = iter1->y();
    }

    for (auto iter = s_width_pairs.rbegin(); iter != iter2; ++iter)
    {
        iter->y() = iter2->y();
    }

    vector<std::pair<double, Vector4d>> widths;
    fitLaneWidth(s_width_pairs, 0.1, widths);

    for (const auto& width : widths)
    {
        LaneWidth lane_width;
        lane_width.sOffset = width.first;
        lane_width.a       = width.second[0];
        lane_width.b       = width.second[1];
        lane_width.c       = width.second[2];
        lane_width.d       = width.second[3];
        lane_widths_.push_back(lane_width);
    }
}

void Lane::set_lane_width(double width)
{
    LaneWidth lane_width;
    lane_width.sOffset = 0.0;
    lane_width.a       = width;
    lane_width.b       = 0.0;
    lane_width.c       = 0.0;
    lane_width.d       = 0.0;
    lane_widths_.push_back(lane_width);
}

Road::Road(const PointVec& refline_points)
{
    id_          = GetId();
    name_        = "road" + std::to_string(id_);
    refline_ptr_ = std::make_shared<RefLine>();
    length_      = refline_ptr_->fit(refline_points);
}

void Road::add_lane(const PointVec& left_border, string type)
{
    left_lanes_.emplace_back(left_lanes_.size() + 1, type, refline_ptr_);
    left_lanes_.back().set_left_border(left_border);
    left_lanes_.back().fit_lane_width();
}

void Road::add_lane(double width, string type)
{
    left_lanes_.emplace_back(left_lanes_.size() + 1, type, refline_ptr_);
    left_lanes_.back().set_lane_width(width);
}

Road& Road::connect_to(Road& road)
{
    link_.successor_id            = road.id_;
    link_.successor_type          = "road";
    link_.successor_contact_point = "start";

    road.link_.predecessor_id            = id_;
    road.link_.predecessor_type          = "road";
    road.link_.predecessor_contact_point = "end";

    for (int i = 0; i < left_lanes_.size(); ++i)
    {
        if (i < road.left_lanes_.size())
        {
            left_lanes_[i].connect_to(road.left_lanes_[i]);
        }
    }
    return road;
}

tinyxml2::XMLElement* Road::to_road_xml(tinyxml2::XMLDocument& doc)
{
    tinyxml2::XMLElement* road = doc.NewElement("road");

    auto type = doc.NewElement("type");
    type->SetAttribute("s", 0.0);
    type->SetAttribute("type", "town");
    auto speed = doc.NewElement("speed");
    speed->SetAttribute("max", 30);
    speed->SetAttribute("unit", "km/h");
    type->InsertEndChild(speed);
    road->InsertEndChild(type);

    auto link = doc.NewElement("link");
    if (link_.predecessor_id > 0)
    {
        auto predecessor = doc.NewElement("predecessor");
        predecessor->SetAttribute("elementId", link_.predecessor_id);
        predecessor->SetAttribute("elementType", link_.predecessor_type.c_str());
        if (link_.predecessor_type == "road")
        {
            predecessor->SetAttribute("contactPoint", link_.predecessor_contact_point.c_str());
        }
        link->InsertEndChild(predecessor);
    }
    if (link_.successor_id > 0)
    {
        auto successor = doc.NewElement("successor");
        successor->SetAttribute("elementId", link_.successor_id);
        successor->SetAttribute("elementType", link_.successor_type.c_str());
        if (link_.successor_type == "road")
        {
            successor->SetAttribute("contactPoint", link_.successor_contact_point.c_str());
        }
        link->InsertEndChild(successor);
    }
    road->InsertEndChild(link);

    auto planView = refline_ptr_->to_planView_xml(doc);
    road->SetAttribute("id", id_);
    road->SetAttribute("name", name_.c_str());
    road->SetAttribute("length", length_);
    road->SetAttribute("junction", junction_id_);
    road->SetAttribute("rule", rule_.c_str());
    road->InsertEndChild(planView);

    auto lanes       = doc.NewElement("lanes");
    auto lanesection = doc.NewElement("laneSection");
    lanesection->SetAttribute("s", 0.0);
    auto left   = doc.NewElement("left");
    auto center = doc.NewElement("center");
    auto right  = doc.NewElement("right");

    for (auto& lane : left_lanes_)
    {
        left->InsertEndChild(lane.to_lane_xml(doc));
    }

    auto lane = doc.NewElement("lane");
    lane->SetAttribute("id", 0);
    lane->SetAttribute("type", "driving");
    lane->SetAttribute("level", "false");
    center->InsertEndChild(lane);

    for (auto& lane : right_lanes_)
    {
        right->InsertEndChild(lane.to_lane_xml(doc));
    }
    lanesection->InsertEndChild(left);
    lanesection->InsertEndChild(center);
    lanesection->InsertEndChild(right);

    lanes->InsertEndChild(lanesection);
    road->InsertEndChild(lanes);

    return road;
}

void Road::connect_by_junction(Road& road, ID junction_id)
{
    link_.successor_id   = junction_id;
    link_.successor_type = "junction";

    for (auto& lane : left_lanes_)
    {
        lane.link_.successor_id = {};
    }

    road.link_.predecessor_type = "junction";
    road.link_.predecessor_id   = junction_id;

    for (auto& lane : road.left_lanes_)
    {
        lane.link_.predecessor_id = {};
    }
}

tinyxml2::XMLElement* Junction::to_junction_xml(tinyxml2::XMLDocument& doc)
{
    tinyxml2::XMLElement* junction = doc.NewElement("junction");
    junction->SetAttribute("id", id);
    junction->SetAttribute("name", name.c_str());
    for (auto& link : links)
    {
        auto connection = doc.NewElement("connection");
        connection->SetAttribute("incomingRoad", link.incoming_road);
        connection->SetAttribute("connectingRoad", link.connecting_road);
        connection->SetAttribute("contactPoint", link.contact_point.c_str());
        auto laneLink = doc.NewElement("laneLink");
        laneLink->SetAttribute("from", link.from);
        laneLink->SetAttribute("to", link.to);
        connection->InsertEndChild(laneLink);
        junction->InsertEndChild(connection);
    }
    return junction;
}

void Junction::add_connect_road(Road& incoming_road, Road& connecting_road)
{
    Junction::Link link;
    link.incoming_road   = incoming_road.id_;
    link.connecting_road = connecting_road.id_;
    link.contact_point   = "start";
    for (int i = 0; i < incoming_road.left_lanes_.size() && i < connecting_road.left_lanes_.size(); ++i)
    {
        link.from = incoming_road.left_lanes_[i].get_id();
        link.to   = connecting_road.left_lanes_[i].get_id();
        links.push_back(link);
    }

    incoming_road.link_.successor_type = "junction";
    incoming_road.link_.successor_id   = id;

    connecting_road.link_.predecessor_type = "junction";
    connecting_road.link_.predecessor_id   = id;

    connecting_road.junction_id_ = id;
}

void OpenDrive::to_xml(const string& filename)
{
    tinyxml2::XMLElement* openDrive = doc_.NewElement("OpenDRIVE");
    doc_.InsertEndChild(openDrive);
    Header header;
    header.revMajor = "1";
    header.revMinor = "4";
    header.version  = "1.0.0";
    header.date     = "2025-02-14";
    header.north    = 0.0;
    header.south    = 0.0;
    header.east     = 0.0;
    header.west     = 0.0;
    header.vendor   = "Gen OpenDRIVE";
    auto header_xml = doc_.NewElement("header");
    header_xml->SetAttribute("revMajor", header.revMajor.c_str());
    header_xml->SetAttribute("revMinor", header.revMinor.c_str());
    header_xml->SetAttribute("name", header.name.c_str());
    header_xml->SetAttribute("version", header.version.c_str());
    header_xml->SetAttribute("date", header.date.c_str());
    header_xml->SetAttribute("north", header.north);
    header_xml->SetAttribute("south", header.south);
    header_xml->SetAttribute("east", header.east);
    header_xml->SetAttribute("west", header.west);
    header_xml->SetAttribute("vendor", header.vendor.c_str());
    openDrive->InsertEndChild(header_xml);

    for (auto& road : roads_)
    {
        openDrive->InsertEndChild(road.to_road_xml(doc_));
    }
    doc_.SaveFile(filename.c_str());
}

Road& OpenDrive::add_road(const PointVec& refline_points)
{
    roads_.emplace_back(refline_points);
    return roads_.back();
}

Junction& OpenDrive::add_junction()
{
    junctions_.emplace_back();
    return junctions_.back();
}

}  // namespace ODR