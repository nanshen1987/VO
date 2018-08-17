#ifndef MAPPOINT_H
#define MAPPOINT_H
#include"common_include.h"
namespace lmars{
	class Frame;
  class MapPoint
  {
  public:
    typedef shared_ptr<MapPoint> Ptr;
    unsigned long id_;
	static unsigned long factory_id_;
	bool good_;
    Vector3d pos_;
    Vector3d norm_;
    Mat descriptor_;
    int observe_times_;
    int correct_times_;
	List<Frame*> observed_frames;
	
    MapPoint();
    MapPoint(
	unsigned long id,
	const Vector3d& position,
	const Vector3d& norm,
	Frame* frame=nullptr,
	const Mat& descriptor=Mat()
	);
	inline cv::Point3f getPositionCV() const{
		return cv::Point3f(pos_(0,0),pos_(1,0),pos_(2,0));
	}
    static MapPoint::Ptr createMapPoint();
	static MapPoint::Ptr createMapPoint(
		const Vector3d& pos_world,
		const Vector3d& norm,
		const Mat& descriptor,
		Frame* frame
	);
  };
}

#endif // MAPPOINT_H
