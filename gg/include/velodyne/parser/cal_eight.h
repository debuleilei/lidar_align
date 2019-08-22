/************************************************************************************
eight calibrate
1)计算各自的ｒ，ｔ
２)


***************************************************************************************/
#pragma once


#include<string>
#include<velodyne/parser/gps_common/conversions.h>

#include <pcl-1.7/pcl/ModelCoefficients.h>
#include <pcl-1.7/pcl/io/pcd_io.h>
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
// #include<pcl-1.7/pcl/visualization/cloud_viewer.h>
#include <pcl-1.7/pcl/visualization/pcl_visualizer.h>
#include <pcl-1.7/pcl/filters/voxel_grid.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "pointmatcher/PointMatcher.h"


using namespace std;

struct utm
{
    double x;
    double y;
};


class cal_eight
{
public:

    bool init();

    bool compute_utm (vector<double>& latilong,utm& utm);

    bool compute_ins_r(double z,Eigen::Matrix2d& r);

    Eigen::Matrix4d  sent_x_y_angle(vector<Eigen::Matrix2d>& r,vector<Eigen::Matrix2d>& lidar,vector<utm>& tins,double height);   

	cal_eight();
   
public:
    vector<double> angle;
    vector<utm> utm_list;
    vector<Eigen::Isometry3d> T_list;//utm坐标系下的位置矩阵

    vector<Eigen::Matrix2d>R_vec;
    vector<utm>tins_vec;
    vector<Eigen::Matrix2d>tlidar_vec;

    
    bool zheng,fu;//顺时针，逆时针标志位
    bool zheng_,fu_;
    bool init_calib;//true-标定成功，false-标定失败
    bool ss;//true-开始计算外参，false，跳出
    bool init_add;//标定结束，开启点云叠加标志
    
    bool start_save;//保存数据标志位 


    double lidar_height;
    Eigen::Matrix4d result_init;
    Eigen::Matrix4d m_xy;
    int iter;
    int zheng_iter;
    int fu_iter;   

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

class
calib_imu_lidar
{
public:
    calib_imu_lidar();
    //输入一帧点云.提取地面法向量,
    //succes 1,failure 0.
    int filter_floor(pcl::PointCloud<pcl::PointXYZ>::Ptr& point,pcl::ModelCoefficients::Ptr& Normal_floor);

    //点云坐标系垂直矫正
    void turn_vetical (pcl::PointCloud<pcl::PointXYZ>::Ptr& point,pcl::ModelCoefficients::Ptr& Normal_floor);

    //绕任意轴旋转的矩阵 point（轴上的点）vector（单位化的轴向量）,t（旋转角度）
    Eigen::Matrix4d rot_mat(const Eigen::Vector3d& point, const Eigen::Vector3d& vector, const double t);
    //计算雷达的高度——点到平面距离
    double calpointtoplane(Eigen::Vector4d& centroid, Eigen::Vector4d& plane); 

private:
    //读取csv文件的子程序；
    string Trim(string& str);
    //线到面的交点 d_line线的法向量，Q线上一点，n_plane平面法向量，
    Eigen::Vector3d linetoplane(Eigen::Vector3d& d_line,Eigen::Vector3d& Q,Eigen::Vector3d& n_plane,double d);
    //求广义矩阵的逆
    Eigen::MatrixXd pinv(Eigen::MatrixXd  A);

public:
    double height;//激光雷达距离地面的检测高度
    double imu_z;//IMU距离地面的安装高度 274mm
    //double lslidar_z;//镭神单线距离地面高度182mm
    double head; //车的航向角
    Eigen::Matrix4d M_XY;
    int color=0; //点云颜色
 
    // Eigen::Vector3d car_position;//车上IMU的UTM坐标
    bool show ;

};