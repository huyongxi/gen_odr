#include "opendrive.h"

#include <tinyxml2.h>

#include <iostream>

#include "arc_curves.h"

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

tinyxml2::XMLElement* Lane::to_lane_xml(tinyxml2::XMLDocument& doc)
{
    tinyxml2::XMLElement* lane = doc.NewElement("lane");
    lane->SetAttribute("id", id_);
    lane->SetAttribute("type", type_.c_str());
    lane->SetAttribute("level", "false");
    auto link = doc.NewElement("link");
    lane->InsertEndChild(link);
    auto width = doc.NewElement("width");
    width->SetAttribute("sOffset", 0.0);
    width->SetAttribute("a", width_);
    width->SetAttribute("b", 0.0);
    width->SetAttribute("c", 0.0);
    width->SetAttribute("d", 0.0);
    lane->InsertEndChild(width);
    return lane;
}

Road::Road(const PointVec& refline_points)
{
    refline_ptr_ = std::make_shared<RefLine>();
    fit(refline_points);
}

void Road::fit(const PointVec& refline_points)
{
    double s = 0.0;

    for (int i = 0; i < refline_points.size() - 2; ++i)
    {
        double x1 = refline_points[i].x;
        double y1 = refline_points[i].y;
        double x2 = refline_points[i + 1].x;
        double y2 = refline_points[i + 1].y;
        double x3 = refline_points[i + 2].x;
        double y3 = refline_points[i + 2].y;
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
        std::tie(xarc, yarc, xendline, yendline, curvature, length) =
            getArcCurvatureAndLength(x1, y1, x3, y3, x2, y2, 0.8, 0.5, 100);

        if (distance(x1, y1, xarc, yarc) > 0.1)
        {
            std::cout << "xstart: " << x1 << " ystart: " << y1 << " length: " << distance(x1, y1, xarc, yarc)
                      << " heading: " << hdg << " curvature: " << 0.0 << std::endl;

            auto line = std::make_shared<StraightLine>();
            line->s = s;
            line->x = x1;
            line->y = y1;
            line->hdg = hdg;
            line->length = distance(x1, y1, xarc, yarc);
            s += line->length;
            refline_ptr_->reflines.push_back(line);
        }

        std::cout << "xstart: " << xarc << " ystart: " << yarc << " length: " << length << " heading: " << hdg
                  << " curvature: " << curvature << std::endl;

        auto arc = std::make_shared<ArcLine>();
        arc->s = s;
        arc->x = xarc;
        arc->y = yarc;
        arc->hdg = hdg;
        arc->length = length;
        arc->curvature = curvature;
        s += arc->length;
        refline_ptr_->reflines.push_back(arc);

        if (distance(xendline, yendline, x3, y3) > 0.1)
        {
            std::cout << "xstart: " << xendline << " ystart: " << yendline
                      << " length: " << distance(xendline, yendline, x3, y3)
                      << " heading: " << giveHeading(xendline, yendline, x3, y3) << " curvature: " << 0.0 << std::endl;

            auto line = std::make_shared<StraightLine>();
            line->s = s;
            line->x = x1;
            line->y = y1;
            line->hdg = giveHeading(xendline, yendline, x3, y3);
            line->length = distance(xendline, yendline, x3, y3);
            s += line->length;
            refline_ptr_->reflines.push_back(line);
        }
    }

    length_ = s;
}

void Road::add_lane(double width)
{
    Lane lane;
    lane.id_ = 1;
    lane.type_ = "driving";
    lane.width_ = width;
    lane.refline_ptr_ = refline_ptr_;
    left_lanes_.push_back(lane);
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
    road->InsertEndChild(link);

    auto planView = refline_ptr_->to_planView_xml(doc);
    road->SetAttribute("id", id_);
    road->SetAttribute("name", name_.c_str());
    road->SetAttribute("length", length_);
    road->SetAttribute("junction", junction_id_);
    road->InsertEndChild(planView);

    auto lanes = doc.NewElement("lanes");
    auto lanesection = doc.NewElement("laneSection");
    lanesection->SetAttribute("s", 0.0);
    auto left = doc.NewElement("left");
    auto center = doc.NewElement("center");
    auto right = doc.NewElement("right");

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

void OpenDrive::to_xml(const string& filename)
{
    tinyxml2::XMLElement* openDrive = doc_.NewElement("OpenDRIVE");
    doc_.InsertEndChild(openDrive);
    Header header;
    header.revMajor = "1";
    header.revMinor = "4";
    header.version = "1.0.0";
    header.date = "2025-02-14";
    header.north = 0.0;
    header.south = 0.0;
    header.east = 0.0;
    header.west = 0.0;
    header.vendor = "Gen OpenDRIVE";
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

    std::cout << doc_.ErrorStr() << std::endl;
    std::cout << doc_.SaveFile(filename.c_str()) << std::endl;
}

void OpenDrive::add_road(const PointVec& refline_points)
{
    roads_.emplace_back(refline_points);
    roads_.back().add_lane(3);
}