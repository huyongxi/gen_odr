#include <iostream>

#include "arc_curves.h"
#include "json_map.h"
#include "opendrive.h"
#include "test_data.h"

int main()
{
    ODR::OpenDrive opendrive;
    auto& road1 = opendrive.add_road(road1_right);
    road1.add_lane(road1_left, "driving");

    auto& road2 = opendrive.add_road(road2_right);
    road2.add_lane(road2_left, "driving");

    auto& road3 = opendrive.add_road(road3_right);
    road3.add_lane(road3_left, "driving");

    auto& road4 = opendrive.add_road(road4_right);
    road4.add_lane(road4_left, "driving");

    auto& road5 = opendrive.add_road(road5_right);
    road5.add_lane(3);

    road1 >> road2 >> road3;
    opendrive.to_xml("test.xodr");

    JsonMap::Map map;
    if (map.from_json("map.json"))
    {
        map.to_xodr("map.xodr");
    }
    
}