#include"lmars/visualodometry.h"
#include"lmars/config.h"
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/viz.hpp>
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
	rgb_times.push_back(rgb_time);
	rgb_files.push_back(rgb_file);
	depth_times.push_back(depth_time);
	depth_files.push_back(depth_file);
	if(fin.good()==false){
	  break;
	}
    }
    
    lmars::Camera::Ptr camera(new lmars::Camera);
    
    cv::viz::Viz3d vis("Visual Odometry");
    cv::viz::WCoordinateSystem world_coor(1.0),camera_coor(0.5);
    cv::Point3d cam_pos(0,-1.0,-1.0),cam_focal_point(0,0,0),cam_y_dir(0,1,0);
    cv::Affine3d cam_pos=cv::viz::makeCameraPose(cam_pos,cam_focal_point,cam_y_dir);
    vis.setViewerPose(cam_pos);
    world_coor.setRenderingProperty(cv::viz::LINE_WIDTH,2.0);
    camera_coor.setRenderingProperty(cv::viz::LINE_WIDTH,1.0);
    vis.showWidget("world",world_coor);
    vis.showWidget("camera",camera_coor);
    
}