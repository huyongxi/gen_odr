#include "arc_curves.h"
#include "opendrive.h"
#include "test_data.h"

#include <iostream>


int main()
{
    OpenDrive opendrive;
    auto road1 = opendrive.add_road(test_points4);
    road1.add_lane(test_points5);
    opendrive.to_xml("test.xml");
}