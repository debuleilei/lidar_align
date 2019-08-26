/************************************************************************************
eight calibrate
1)计算各自的ｒ，ｔ
２)


***************************************************************************************/
// #pragma once


#include<string>
#include<velodyne/parser/gps_common/conversions.h>

#include <pcl-1.7/pcl/ModelCoefficients.h>
#include <pcl-1.7/pcl/io/pcd_io.h>
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

    Eigen::Matrix2d ins_r; 
    Eigen::Vector2d ins_tr;
    Eigen::Vector2d lidar_tr;  
    
    vector<double> relitive_position;

    bool compute_utm (vector<double>& latilong,utm& utm);


    bool compute_ins_r(double z,Eigen::Matrix2d& r);
    // bool trans_car(vector<double>& utm,double angle,vector<double>& car);
    bool trans_car( utm& deta_utm,double angle,utm& car);
    bool compute_ins_tr(vector<double>& utm1,vector<double>& utm2,vector<double>& utm3);
    bool sent_x_y_angle(Eigen::Matrix2d& r,Eigen::Matrix2d& lidar,vector<double>& tins);
    Eigen::Matrix4d sent_x_y_angle(vector<Eigen::Matrix2d>& r,vector<Eigen::Matrix2d>& lidar,vector<utm>& tins);    
    bool sent_qt();

	cal_eight();
    ~cal_eight();
private:
    
    
    
    bool compute_lidar_t();
public:
    double height_back_imu;
};