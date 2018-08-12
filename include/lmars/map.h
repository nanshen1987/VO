#ifndef MAP_H
#define MAP_H
#include"common_include.h"
#include"mappoint.h"
#include"frame.h"
namespace lmars{
  class Map
  {
  public:
    typedef shared_ptr<Map> Ptr;
    unordered_map<unsigned long,MapPoint::Ptr> map_points_;
    unordered_map<unsigned long,Frame::Ptr> key_frames_;
    
    Map(){};
    void insertKeyFrame(Frame::Ptr frame);
    void insertMapPoint(MapPoint::Ptr mapPoint);
  };
}

#endif // MAP_H
