#include"lmars/visualodometry.h"
#include"lmars/config.h"
#include<opencv2/imgcodecs.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/viz.hpp>
#include<boost/timer.hpp>
#include<fstream>
int main(int argc,char ** argv)
{
    if(argc!=2){
      cout<<"usage:run_vo parameter file"<<endl;
      return 1;
    }
    lmars::Config::setParameterFile(argv[1]);
    lmars::VisualOdometry::Ptr vo(new lmars::VisualOdometry);
    string dataset_dir=lmars::Config::get<string>("dataset_dir");
    cout<<"dataset_dir:"<<dataset_dir<<endl;
    ifstream fin(dataset_dir+"/associate.txt");
    if(!fin){
      cout<<"please generate the associate file called associate.txt"<<endl;
      return 1;
    }
    vector<string> rgb_files,depth_files;
    vector<double> rgb_times,depth_times;
    while(!fin.eof()){
	string rgb_time,rgb_file,depth_time,depth_file;
	fin>>rgb_time>>rgb_file>>depth_time>>depth_file;
	rgb_times.push_back(atof(rgb_time.c_str()));
	rgb_files.push_back(dataset_dir+"/"+rgb_file);
	depth_times.push_back(atof(depth_time.c_str()));
	depth_files.push_back(dataset_dir+"/"+depth_file);
	if(fin.good()==false){
	  break;
	}
    }
    
    lmars::Camera::Ptr camera(new lmars::Camera);
    
    cv::viz::Viz3d vis("Visual Odometry");
    cv::viz::WCoordinateSystem world_coor(1.0),camera_coor(0.5);
    cv::Point3d cam_pos(0,-1.0,-1.0),cam_focal_point(0,0,0),cam_y_dir(0,1,0);
    cv::Affine3d cam_pose=cv::viz::makeCameraPose(cam_pos,cam_focal_point,cam_y_dir);
    vis.setViewerPose(cam_pose);
    world_coor.setRenderingProperty(cv::viz::LINE_WIDTH,2.0);
    camera_coor.setRenderingProperty(cv::viz::LINE_WIDTH,1.0);
    vis.showWidget("world",world_coor);
    vis.showWidget("camera",camera_coor);
    cout<<"read total "<<rgb_files.size()<<" entries"<<endl;
    for(int i=0;i<rgb_files.size();i++){
      Mat color=cv::imread(rgb_files[i]);
      Mat depth=cv::imread(depth_files[i],-1);
      if(color.data==nullptr||depth.data==nullptr){
	break;
      }
      lmars::Frame::Ptr pFrame=lmars::Frame::createFrame();
      pFrame->camera_=camera;
      pFrame->color_=color;
      pFrame->depth_=depth;
      pFrame->time_stamp_=rgb_times[i];
      
      boost::timer timer;
      vo->addFrame(pFrame);
      cout<<"VO costs time:"<<timer.elapsed()<<endl;
      if(vo->state_==lmars::VisualOdometry::LOST){
	break;
      }
      SE3 Tcw=pFrame->T_c_w_.inverse();
      cv::Affine3d M(
	cv::Affine3d::Mat3(
	  Tcw.rotation_matrix()(0,0), Tcw.rotation_matrix()(0,1), Tcw.rotation_matrix()(0,2),
	  Tcw.rotation_matrix()(1,0), Tcw.rotation_matrix()(1,1), Tcw.rotation_matrix()(1,2),
	  Tcw.rotation_matrix()(2,0), Tcw.rotation_matrix()(2,1), Tcw.rotation_matrix()(2,2)
	),
	cv::Affine3d::Vec3(
	  Tcw.rotation_matrix()(0,0),Tcw.rotation_matrix()(1,0),Tcw.rotation_matrix()(2,0)
	)
      );
      cv::imshow("image",color);
      cv::waitKey(3000);
      vis.setWidgetPose("camera",M);
      vis.spinOnce(1,false);
    }
    return 0;
    
}