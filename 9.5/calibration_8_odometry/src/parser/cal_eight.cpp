#include "velodyne/parser/cal_eight.h"
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

#define PI (3.1415926535897932346f)


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

void cal_eight::compute_score(vector<Eigen::Affine3d>& trans,double& score)
{
    Eigen::Vector3d angle_min;
    Eigen::Vector3d t_min;
    for(auto t:trans)
    {        
        Eigen::Matrix3d rotation_matrix;
        rotation_matrix=t.matrix().block<3,3>(0,0);
        Eigen::Vector3d eulerAngle=rotation_matrix.eulerAngles(2,1,0);
        eulerAngle*=(180/PI);
        if(eulerAngle(0)>170)
        {
            eulerAngle(0)-=180;
            eulerAngle(1)+=180;
            eulerAngle(2)+=180;
        }

        // cout<<"angle:"<<eulerAngle<<endl;

        Eigen::Vector3d trans_vec(t(0,3),t(1,3),t(2,3));

        // cout<<"norm:"<<trans_vec.norm()<<endl;


        angle_min+=eulerAngle;
        t_min+=trans_vec;
       
    }
    cout<<"angle_error:"<<angle_min/trans.size()<<endl;
    cout<<"trans_error"<<t_min/trans.size()<<endl;

    double angle_mean=(angle_min/trans.size()).norm();
    double tran_mean=(t_min/trans.size()).norm();
    if (angle_mean>0.5 || tran_mean>0.05)
    {
        score=0;
    }
    else
    {
        score=100-100*angle_mean-1000*tran_mean;
    }
    cout<<"sroc:"<<score<<endl;

}

void cal_eight::filter_ground(pcl::PointCloud<pcl::PointXYZ>::Ptr& points,pcl::PointCloud<pcl::PointXYZ>::Ptr& points_filter)
{
    for(auto point:points->points)
    {
        if(point.z>-1.6)
        {
            points_filter->push_back(point);
        }
    }
    // cout<<"before filter:"<<points->points.size()<<" "<<"after filter:"<<points_filter->points.size()<<endl;
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