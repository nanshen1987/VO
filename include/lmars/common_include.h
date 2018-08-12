#ifndef COMMON_INCLUDE
#define COMMON_INCLUDE
#include<Eigen/Core>
#include<Eigen/Geometry>
using Eigen::Vector2d;
using Eigen::Vector3d;


#include<sophus/se3.h>
#include<sophus/so3.h>
using Sophus::SE3;
using Sophus::SO3;

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<opencv2/core/mat.hpp>
using cv::Mat;
using cv::ORB;


#include<iostream>
#include<memory>
#include<unordered_map>
#include<vector>
#include<string>
using namespace std;




#endif
