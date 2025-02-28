#include "json_map.h"
#include <unordered_map>
#include <fstream>
#include <nlohmann/json.hpp>

using std::ifstream;
using json = nlohmann::json;

namespace JsonMap
{

bool Map::from_json(string file_name)
{
    ifstream file(file_name);
    if (file)
    {
        json data = json::parse(file, nullptr, false);
        if (data.is_discarded())
        {
            return false;
        }

        try
        {
            for (auto& rd : data)
            {
                Road road;
                road.start_node = rd["startNode"]["id"];
                road.end_node = rd["endNode"]["id"];

                for (auto& point : rd["geometry"]["coordinates"])
                {
                    road.ref_line.emplace_back(point["x"], point["y"], point["z"]);
                }

                for (auto& border : rd["laneBoundary"])
                {
                    Lane lane;
                    lane.start_node = border["startNode"]["id"];
                    lane.end_node = border["endNode"]["id"];
                    for (auto& point : border["geometry"]["coordinates"])
                    {
                        lane.line.emplace_back(point["x"], point["y"], point["z"]);
                    }
                    road.Lanes.push_back(lane);
                }
                roads_.push_back(road);
            }

        } catch (std::exception& e)
        {
            return false;
        }

        return true;
    }
    return false;
}


struct LinkNode
{
    //ID id;
    vector<Road*> from;
    vector<Road*> to;
};

void Map::to_xodr(string file_name) 
{   
    
    std::unordered_map<ID, LinkNode> links;
    for (auto& road : roads_)
    {
        auto& r = opendrive_.add_road(road.ref_line);
        for (const auto& lane : road.Lanes)
        {
            r.add_lane(lane.line);
        }
        road.odr_road = &r;

        links[road.end_node].from.push_back(&road);
        links[road.start_node].to.push_back(&road);
    }

    for (const auto& link : links)
    {
        if (link.second.from.size() == 1)
        {
            for (const auto& to : link.second.to)
            {
                *(link.second.from[0]->odr_road) >> *(to->odr_road);
            }
            continue;
        }

        if (link.second.to.size() == 1)
        {
            for (const auto& from : link.second.from)
            {
                *(from->odr_road) >> *(link.second.to[0]->odr_road);
            }
        }
    }


    opendrive_.to_xml(file_name);
}
}  // namespace JsonMap
