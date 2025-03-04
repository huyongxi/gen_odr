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
    ODR::Road* odr_road;
};

class Map
{
   private:
    vector<Road> roads_;
    ODR::OpenDrive opendrive_;

   public:
    // 从JSON文件加载地图
    bool from_json(string file_name);
    // 将地图转换为OpenDRIVE格式并保存为文件
    void to_xodr(string file_name);
};

}  // namespace JsonMap
