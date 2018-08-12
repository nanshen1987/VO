#ifndef MAPPOINT_H
#define MAPPOINT_H
#include"common_include.h"
namespace lmars{
  class MapPoint
  {
  public:
    typedef shared_ptr<MapPoint> Ptr;
    unsigned long id_;
    Vector3d pos_;
    Vector3d norm_;
    Mat descriptor_;
    int observe_times_;
    int correct_times_;
    MapPoint();
    MapPoint(long id,Vector3d position,Vector3d norm);
    static MapPoint::Ptr createMapPoint();
  };
}

#endif // MAPPOINT_H
