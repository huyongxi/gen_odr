#include "opendrive.h"
#include "test_data.h"


int main()
{
    OpenDrive opendrive;
    opendrive.add_road(test_points);
    opendrive.to_xml("test.xml");
    return 0;
}