#include "lmars/map.h"
namespace lmars {
void Map::insertKeyFrame(Frame::Ptr frame)
{
  cout<<"key_frames size "<<key_frames_.size()<<endl;
  if(key_frames_.find(frame->id_)==key_frames_.end())
  {
    key_frames_.insert(make_pair(frame->id_,frame));
  }
  else{
	key_frames_[frame->id_]=frame;
  }
}
void Map::insertMapPoint(MapPoint::Ptr mapPoint)
{
  if(map_points_.find(mapPoint->id_)==map_points_.end())
  {
    map_points_.insert(make_pair(mapPoint->id_,mapPoint));
  }else{
    map_points_[mapPoint->id_]=mapPoint;
  }
}

}
