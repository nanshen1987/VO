#ifndef FRAME_H
#define FRAME_H
#include"common_include.h"
#include"camera.h"
namespace lmars{
class Frame
{
  public:
    typedef shared_ptr<Frame> Ptr;
    unsigned long id_;
    double time_stamp_;
    SE3 T_c_w_;
       
    Camera::Ptr camera_;
    Mat color_,depth_;
  public:
    Frame();
    Frame(long id,double time_stamp=0,SE3 T_c_w=SE3(),Camera::Ptr camera=nullptr,Mat color=Mat(),Mat depth=Mat());
    ~Frame();
    
    static Frame::Ptr createFrame();
    
    double findDepth(const cv::KeyPoint& kp);
    
    Vector3d getCamCenter() const;
    bool isInFrame(const Vector3d& pt_world);
  
};
  
}


#endif // FRAME_H
