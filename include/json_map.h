#pragma once
#include "opendrive.h"

namespace JsonMap
{

struct Lane
{
    ID start_node;
    ID end_node;
    PointVec line;
};

struct Road
{
    ID start_node;
    ID end_node;
    PointVec ref_line;
    vector<Lane> Lanes;
};

class Map
{
   private:
    vector<Road> roads_;

   public:
    bool from_json(string file_name);
    void to_xodr(string file_name);
};

}  // namespace JsonMap
