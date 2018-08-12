#include "lmars/visualodometry.h"
#include"lmars/config.h"
#include<opencv2/calib3d/calib3d.hpp>
namespace lmars {
VisualOdometry::VisualOdometry()
:state_(INITIALIZING),ref_(nullptr),curr_(nullptr),map_(new Map),num_lost_(0),num_inliners_(0)
{
    num_of_features_    = Config::get<int> ( "number_of_features" );
    scale_factor_       = Config::get<double> ( "scale_factor" );
    level_pyramid_      = Config::get<int> ( "level_pyramid" );
    match_ratio_        = Config::get<float> ( "match_ratio" );
    max_num_lost_       = Config::get<float> ( "max_num_lost" );
    min_inliers_        = Config::get<int> ( "min_inliers" );
    key_frame_min_rot   = Config::get<double> ( "keyframe_rotation" ); 
    key_frame_min_trans = Config::get<double> ( "keyframe_translation" );
    orb_ = cv::ORB::create ( num_of_features_, scale_factor_, level_pyramid_ );

}
VisualOdometry::~VisualOdometry()
{

}
bool VisualOdometry::addFrame(Frame::ptr frame)
{
  switch(state_){
    case INITIALIZING:
    {
      state_=OK;
      curr_=ref_=frame;
      map_->insertKeyFrame(frame);
      extractKeyPoints();
      computeDescriptors();
      setRef3DPoints();
      break;
    }
    case OK:
    {
      curr_=frame;
      extractKeyPoints();
      computeDescriptors();
      featureMatching();
      poseEstimationPnp();
      if(checkEstimatedPose()==true)
      {
	curr_->T_c_w_=T_c_r_estimated_*ref_->T_c_w_;
	ref_=curr_;
	setRef3DPoints();
	num_lost_=0;
	if(checkKeyFrame()==true){
	  addKeyFrame();
	}
      }
      else
      {
	num_lost_++;
	if(num_lost_>max_num_lost_){
	  state_=LOST;
	}
	return false;
      }
      
    }
    case LOST:
    {
      cout<<"VO has lost"<<endl;
      break;
    }
  }
  return true;
}

void VisualOdometry::extractKeyPoints()
{
  orb_->detect(curr_->color_,Keypoints_curr_);
}
void VisualOdometry::computeDescriptors()
{
  orb_->compute(curr_->color_,Keypoints_curr_,descriptors_curr_);
}
void VisualOdometry::featureMatching()
{
  vector<cv::DMatch> matches;
  cv::BFMatcher matcher(cv::NORM_HAMMING);
  matcher.match(descriptors_ref_,descriptors_curr_,matches);
  float min_dis=min_element(matches.begin(),matches.end(),
			    [](const cv::DMatch m1,const cv::DMatch m2){
			      return m1.distance<m2.distance;
			    }
			   ).distance;

  feature_matches_.clear();
  for(cv::DMatch match:matches){
    if(match.distance<std::max<float>(min_dis*match_ratio_,30.0)){
	feature_matches_.push_back(match);
    }
  }
  cout<<"good matches: "<<feature_matches_.size()<<endl;
}

void VisualOdometry::setRef3DPoints()
{
  pts_3d_ref_.clear();
  descriptors_ref_=Mat();
  for(size_t i=0;i<Keypoints_curr_.size();i++){
    double d=ref_->findDepth(Keypoints_curr_[i]);
    if(d>0){
      Vector3d p_cam=ref_->camera_->pixel2camera(Vector2d(Keypoints_curr_[i].pt.x,Keypoints_curr_[i].pt.y),d);
      pts_3d_ref_.push_back(cv::Point3f(p_cam(0,0),p_cam(1,0),p_cam(2,0));
      descriptors_ref_.push_back(descriptors_curr_.row(i));
    }
  }
}

void VisualOdometry::poseEstimationPnp()
{
  vector<cv::Point3f> pts3d;
  vector<cv::Point2f> pts2d;
  for(cv::DMatch m:feature_matches_){
    pts3d.push_back(pts_3d_ref_[m.queryIdx]);
    pts2d.push_back(Keypoints_curr_[m.trainIdx].pt);
  }
  Mat K=(cv::Mat_<double>(3,3)<<
    ref_->camera_->fx_,0,ref_->camera_->cx_,
    0,ref_->camera_->fy_,ref_->camera_->cy_,
    0,0,1
  );
    
  Mat rvec,tvec,inliers;
  cv::solvePnPRansac(pts3d,pts2d,K,Mat(),rvec,tvec,false,100,4.0,0.99,inliers);
  num_inliners_=inliers.rows;
  cout<<"PnP inliners: "<<num_inliners_<<endl;
  T_c_r_estimated_=SE3(
      SO3(rvec.at<double>(0,0),rvec.at<double>(1,0),rvec.at<double>(2,0)),
      Vector3d(tvec.at<double>(0,0),tvec.at<double>(1,0),tvec.at<double>(2,0))
  );
}
bool VisualOdometry::checkEstimatedPose()
{
  if(num_inliners_<min_inliers_){
    cout<<"reject because num_inliners_ is too small" <<endl;
    return false;
  }
  Sophus::Vector6d d=T_c_r_estimated_.log();
  if(d.norm()>5.0){
    cout<< "reject because motion is too large"<<endl;
    return false;
  }
  return true;
}

bool VisualOdometry::checkKeyFrame()
{
  Sophus::Vector6d d=T_c_r_estimated_.log();
  Vector3d trans=d.head<3>();
  Vector3d rot=d.tail<3>();
  if(trans.norm()>key_frame_min_trans||rot.norm()>key_frame_min_rot){
    return true;
  }
  return false;
}
void VisualOdometry::addKeyFrame()
{
  cout<<"add key frame"<<endl;
  map_->insertKeyFrame(curr_);
}







}
