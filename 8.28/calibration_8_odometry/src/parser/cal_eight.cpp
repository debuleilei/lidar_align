#include "velodyne/parser/cal_eight.h"
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>




using namespace std;
using namespace Eigen;

cal_eight::cal_eight()
{
  iter_z=0;
  iter_f=0;
  iter=0;//点云叠加保存
  last=0;
  lidar_height=0;

   init_calib=false;  
   init_add=true;
}

bool  cal_eight::compute_utm (vector<double>& latilong,utm& utm)
{
    
    std::string zone;
     gps_common::LLtoUTM(latilong[0], latilong[1],utm.y,utm.x, zone);//左手系，y指北，x指东，顺时针＋//什么也不是
   
}
  
Eigen::Matrix4d cal_eight::sent_x_y_angle(vector<Eigen::Matrix2d>& r,vector<Eigen::Matrix2d>& lidar,vector<utm>& tins,double li_height)
{
   
   Eigen::Matrix2d I;
    I << 1, 0,
        0, 1;
    int rows=r.size();
    int tin=tins.size();
    cout<<"rows:"<<rows<<" "<<"tins:"<<tin<<endl;

    Eigen::MatrixXd A(2*rows,4);
    Eigen::MatrixXd b(2*rows,1);
    for(int i=0;i<r.size();++i)
    {
       
        A.block<2,2>(2*i,0)=r[i]-I;
        A.block<2,2>(2*i,2)=lidar[i];

        Eigen::MatrixXd aa(2,1);
        aa<<-tins[i].x,-tins[i].y;
        b.block<2,1>(2*i,0)=aa;
    }

   
    Eigen::MatrixXd X(4,1),Y(4,1);
    // X=A.fullPivHouseholderQr().solve(b);
    X=A.colPivHouseholderQr().solve(b);
 
    cout<<"外参X为："<<X <<endl;

    Eigen::Vector2d cos_sin(X(2,0),X(3,0));
    cos_sin.normalize();//归一化
    cout<<"归一化："<<cos_sin<<endl;

    Eigen::Matrix4d result_init;
    // 从新求解tx,ty
    // {
    //     Eigen::MatrixXd A_(2*rows,2);
    //     Eigen::MatrixXd b_(2*rows,1);

    //     Eigen::MatrixXd c_lidar(2*rows,1);

    //     for(int i=0;i<r.size();++i)
    //     {
        
    //         A_.block<2,2>(2*i,0)=r[i]-I;
    //         Eigen::Vector2d li=lidar[i]*cos_sin;

    //         Eigen::Vector2d aa_;
    //         aa_<<-tins[i].x,-tins[i].y;
    //         b_.block<2,1>(2*i,0)=aa_-li;
    //     }
    //     // cout<<"A:"<<A_<<endl;
    //     // cout<<"b="<<b_<<endl;
    //     Eigen::Vector2d txy;
    //     Eigen::MatrixXd aass(2,1);
    //     // cout<<"txy:"<<txy<<endl;
    //     //aass=A_.inverse()*b_;
    //     aass=A_.colPivHouseholderQr().solve(b_);
    //     // cout<<"ax:"<<A_*aass<<endl;

    //     // cout<<"txy:"<<aass<<endl;

    //     result_init<<cos_sin(0),-cos_sin(1),0,aass(0,0),
    //             cos_sin(1),cos_sin(0),0,aass(1,0),
    //             0,0,1,1.5,
    //             0,0,0,1;
    // }
        result_init<<cos_sin(0),-cos_sin(1),0,X(0,0),
                cos_sin(1),cos_sin(0),0,X(1,0),
                0,0,1,li_height,
                0,0,0,1; 
        //failure   result_init<<X(2,0),-X(3,0),0,X(0,0),
                // X(3,0),X(2,0),0,X(1,0),
                // 0,0,1,1.5,
                // 0,0,0,1;   

    return result_init;  
}



pcTransEstimator::pcTransEstimator()
{
	icp1.setDefault();
	// std::cout<<"icp use default config"<< std::endl;
}

void pcTransEstimator::convert(pcl::PointCloud<pcl::PointXYZ>& in, PointMatcher<float>::DataPoints& out)
{
	std::size_t n = in.size();
	out.features.resize(4,n);

	for (std::size_t i=0; i< n; i++)
	{
		out.features(0,i) = in.points[i].x;
		out.features(1,i) = in.points[i].y;
		out.features(2,i) = in.points[i].z;
		out.features(3,i) = 1.0;
	}
}

void pcTransEstimator::estimate(pcl::PointCloud<pcl::PointXYZ>& ref, pcl::PointCloud<pcl::PointXYZ>& data, Eigen::Affine3d& t)
{
	
	PointMatcher<float>::DataPoints p1;
	PointMatcher<float>::DataPoints p2;

	convert(ref, p1);
	convert(data, p2);

	PointMatcher<float>::TransformationParameters T = icp1(p2, p1);

	float matchRatio = icp1.errorMinimizer->getWeightedPointUsedRatio();

	Eigen::Matrix4f tmp = T;
	t = Eigen::Affine3d(tmp.cast<double>());
}