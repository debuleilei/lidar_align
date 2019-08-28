/************************************************************************************
eight calibrate
1)计算各自的ｒ，ｔ
２)


***************************************************************************************/
// #pragma once


#include<string>
#include<velodyne/parser/gps_common/conversions.h>
#include "pointmatcher/PointMatcher.h"

#include <pcl-1.7/pcl/ModelCoefficients.h>
#include <pcl-1.7/pcl/io/ply_io.h>
#include <pcl-1.7/pcl/point_types.h>
#include <pcl-1.7/pcl/features/normal_3d.h>
#include <pcl-1.7/pcl/filters/passthrough.h>
#include <pcl-1.7/pcl/filters/extract_indices.h>
#include <pcl-1.7/pcl/sample_consensus/method_types.h>
#include <pcl-1.7/pcl/sample_consensus/model_types.h>
#include <pcl-1.7/pcl/segmentation/sac_segmentation.h>
#include <pcl-1.7/pcl/segmentation/extract_clusters.h>
#include <pcl-1.7/pcl/registration/icp.h>
#include<pcl-1.7/pcl/common/transforms.h>
#include<pcl-1.7/pcl/visualization/cloud_viewer.h>
#include <pcl-1.7/pcl/visualization/pcl_visualizer.h>
#include <pcl-1.7/pcl/filters/voxel_grid.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>




using namespace std;

struct utm
{
    double x;
    double y;
};


class cal_eight
{
public:

    bool compute_utm (vector<double>& latilong,utm& utm);   
    Eigen::Matrix4d sent_x_y_angle(vector<Eigen::Matrix2d>& r,vector<Eigen::Matrix2d>& lidar,vector<utm>& tins,double li_height);    
   
	cal_eight();    
public:
  vector<double> angle;
  vector<Eigen::Isometry3d> T_list;
  vector<Eigen::Matrix2d>R_vec;
  vector<utm>tins_vec;
  vector<Eigen::Matrix2d>lidar_vec;

 
  bool init_calib;  
  bool init_add;

  Eigen::Matrix4d result_init;

  vector<Eigen::Isometry3d> imu_odo;
  vector<Eigen::Matrix4d> lidar_odo;
  
  int iter_z;
  int iter_f;
  int iter;//点云叠加保存
  int last;
  double lidar_height;

};


class pcTransEstimator
{
private:
	PointMatcher<float>::ICP icp1;
	void convert(pcl::PointCloud<pcl::PointXYZ>& in, PointMatcher<float>::DataPoints& out);

public:
	// pcTransEstimator(const std::string& config);
	pcTransEstimator();
	void estimate(pcl::PointCloud<pcl::PointXYZ>& ref,
				  pcl::PointCloud<pcl::PointXYZ>& data,
				  Eigen::Affine3d& t);

};