#include "json_map.h"

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

void Map::to_xodr(string file_name) {}
}  // namespace JsonMap
