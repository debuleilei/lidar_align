#include "velodyne/parser/cal_eight.h"
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

cal_eight::cal_eight()
{
    init();
}
bool  cal_eight::init ()
{
    zheng=true;
    fu=true;
    zheng_=true;
    fu_=true;
    init_calib=false;
    ss=true;
    init_add=false;
    iter=0;
    zheng_iter=0;
    fu_iter=0;

    start_save=true;
    cout<<"happy init************************"<<endl;
}

bool  cal_eight::compute_utm (vector<double>& latilong,utm& utm)
{    
    std::string zone; 
    
    //  gps_common::LLtoUTM(latilong[0], latilong[1],utm.x,utm.y, zone); //右手系，x指北，ｙ指东,顺时针＋
     gps_common::LLtoUTM(latilong[0], latilong[1],utm.y,utm.x, zone);//左手系，y指北，x指东，顺时针＋//什么也不是
    //cout << setprecision(20);
    //   cout<<"utm:x="<<utm.x<<" y="<<utm.y<<endl;
}


bool cal_eight::compute_ins_r(double z,Eigen::Matrix2d& r)
{      
    r<<cos(z),-sin(z),sin(z),cos(z);//sin(弧度) 
    return true;
}
Eigen::Matrix4d cal_eight::sent_x_y_angle(vector<Eigen::Matrix2d>& r,vector<Eigen::Matrix2d>& lidar,vector<utm>& tins, double height)
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
    
    // cout<<"***********************************************"<<endl;
    Eigen::MatrixXd X(4,1),Y(4,1);
    X=A.fullPivHouseholderQr().solve(b);
    
    cout<<"外参X为："<<X <<endl;
    // cout<<"***********************************************"<<endl;

    Eigen::Vector2d cos_sin(X(2,0),X(3,0));
    cos_sin.normalize();//归一化
    // cout<<"归一化："<<cos_sin<<endl;

    Eigen::Matrix4d result_init;
    //从新求解tx,ty
    {
        Eigen::MatrixXd A_(2*rows,2);
        Eigen::MatrixXd b_(2*rows,1);

        Eigen::MatrixXd c_lidar(2*rows,1);

        for(int i=0;i<r.size();++i)
        {
        
            A_.block<2,2>(2*i,0)=r[i]-I;
            Eigen::Vector2d li=lidar[i]*cos_sin;

            Eigen::Vector2d aa_;
            aa_<<-tins[i].x,-tins[i].y;
            b_.block<2,1>(2*i,0)=aa_-li;
        }

        Eigen::Vector2d txy;
        Eigen::MatrixXd aass(2,1);
       
        aass=A_.colPivHouseholderQr().solve(b_);


        result_init<<cos_sin(0),-cos_sin(1),0,aass(0,0),
                cos_sin(1),cos_sin(0),0,aass(1,0),
                0,0,1,height,
                0,0,0,1;
    }
        
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

	// std::cout << "match ratio1: " << std::endl;
	// std::cout << p1.features(0,1) << std::endl;
	// std::cout << p2.features(3,1) << std::endl;

	PointMatcher<float>::TransformationParameters T = icp1(p2, p1);
	// std::cout << "match ratio2: " << std::endl;
	float matchRatio = icp1.errorMinimizer->getWeightedPointUsedRatio();

	// std::cout << "match ratio: " <<  matchRatio << std::endl;

	Eigen::Matrix4f tmp = T;
	t = Eigen::Affine3d(tmp.cast<double>());
}


Eigen::Vector3d calib_imu_lidar::linetoplane(Eigen::Vector3d& d_line,Eigen::Vector3d& Q,Eigen::Vector3d& n_plane,double d)
{
    Eigen::Vector3d p(0,0,0);
    double delta=d_line.dot(n_plane);
    double dist=n_plane.dot(Q)+d;
    if(delta !=0)
    {
        p=Q-(dist-delta)*d_line;
    }
    return p;
}


double calib_imu_lidar::calpointtoplane(Eigen::Vector4d& centroid, Eigen::Vector4d& plane)
{
	double sample, sa, sb;
	sa = centroid.dot(plane);
	Eigen::Vector3f plane1;
	plane1 << plane(0, 0), plane(1, 0), plane(2, 0);
	sb = plane1.norm();
	sample = std::abs(sa / sb);

	return sample;
}

string calib_imu_lidar::Trim(string& str)
{
    //str.find_first_not_of(" \t\r\n"),
    str.erase(0, str.find_first_not_of(" \t\r\n"));
    str.erase(str.find_last_not_of(" \t\r\n") + 1);
    return str;
}

int  calib_imu_lidar::filter_floor(pcl::PointCloud<pcl::PointXYZ>::Ptr& point,pcl::ModelCoefficients::Ptr& Normal_floor)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZ>);
   
    for(int i=0;i<point->points.size();++i)
    {
        float x=abs(point->points[i].y);
        float y=point->points[i].x;
        float z=point->points[i].z;
        if(x<8.0f && y>0 && y<5.0f && z<0.0f && z>-2.5f)
        {
            cloud_filter->push_back(point->points[i]);
        }
    }

    pcl::io::savePLYFile ("cloud_filter.ply", *cloud_filter);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg123;
    // Optional
    seg123.setOptimizeCoefficients (true);
    // Mandatory
    seg123.setModelType (pcl::SACMODEL_PLANE);
    seg123.setMethodType (pcl::SAC_RANSAC);
    seg123.setDistanceThreshold (0.05);

    seg123.setInputCloud (cloud_filter);
    seg123.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return (-1);
    }

    //＊＊＊＊＊＊＊＊＊＊＊＊＊＊引入会导致pointmatcher的库报错＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊
    // // //得到地面并显示
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_filter_floor(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    // extract.setInputCloud(cloud_filter);
    // extract.setIndices(inliers);
    // extract.setNegative(false);
    // extract.filter(*point_filter_floor);


    double error_z=0; //cos theta

    if(coefficients->values[2]<0)
    {
        Normal_floor->values[0]=-1*coefficients->values[0];
        Normal_floor->values[1]=-1*coefficients->values[1];
        Normal_floor->values[2]=-1*coefficients->values[2];
        Normal_floor->values[3]=-1*coefficients->values[3];
    }else
    {
        Normal_floor=coefficients;
    }
    Eigen::Vector3d normal_vector(Normal_floor->values[0],Normal_floor->values[1],Normal_floor->values[2]);
    error_z=Normal_floor->values[2]/normal_vector.norm();
    if(error_z>0.866 && error_z<1)//小于30度
    {

        Eigen::Vector4d zero(0.0,0.0,0.0,1.0);
        Eigen::Vector4d fplane(Normal_floor->values[0],Normal_floor->values[1],Normal_floor->values[2],Normal_floor->values[3]);
        height=calpointtoplane(zero,fplane);
        return 0;
    }else
    {
        cout<<"fail to get the floor"<<endl;
        return 1;
    }
}

void calib_imu_lidar::turn_vetical (pcl::PointCloud<pcl::PointXYZ>::Ptr& point,pcl::ModelCoefficients::Ptr& Normal_floor)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_tmp(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4d R_T;

    Eigen::Vector3d norm(Normal_floor->values[0],Normal_floor->values[1],Normal_floor->values[2]);
    Eigen::Vector4d Normal_floor_4(Normal_floor->values[0],Normal_floor->values[1],Normal_floor->values[2],Normal_floor->values[3]);
    Eigen::Matrix3d xy_mat;
    Eigen::Vector3d z_v(0.0,0.0,1.0);

    //********************************计算M_XY******************************************
    //1,计算法向量方向，并单位化
    Eigen::Vector3d xy_axis=norm.cross(z_v);

//    cout<<"地面法向量模长："<<norm.norm()<<endl;
//    cout<<"单位化前："<<xy_axis<<endl;
    xy_axis.normalize();
//    cout<<"单位化后："<<xy_axis<<endl;
    //2.计算旋转角度，从norm到z_v
    double t=acos(norm.dot(z_v));
    //控制旋转方向


    Eigen::Vector3d zero(0.0,0.0,0.0);
    M_XY=rot_mat(zero,xy_axis,t);

    pcl::transformPointCloud(*point,*point_tmp, M_XY);
    point->swap(*point_tmp);

}

Eigen::Matrix4d calib_imu_lidar::rot_mat(const Eigen::Vector3d& point, const Eigen::Vector3d& vector, const double t)
{
    double u = vector(0);
    double v = vector(1);
    double w = vector(2);
    double a = point(0);
    double b = point(1);
    double c = point(2);

    Eigen::Matrix4d matrix;
    matrix<<u*u + (v*v + w*w)*cos(t), u*v*(1 - cos(t)) - w*sin(t), u*w*(1 - cos(t)) + v*sin(t), (a*(v*v + w*w) - u*(b*v + c*w))*(1 - cos(t)) + (b*w - c*v)*sin(t),
            u*v*(1 - cos(t)) + w*sin(t), v*v + (u*u + w*w)*cos(t), v*w*(1 - cos(t)) - u*sin(t), (b*(u*u + w*w) - v*(a*u + c*w))*(1 - cos(t)) + (c*u - a*w)*sin(t),
            u*w*(1 - cos(t)) - v*sin(t), v*w*(1 - cos(t)) + u*sin(t), w*w + (u*u + v*v)*cos(t), (c*(u*u + v*v) - w*(a*u + b*v))*(1 - cos(t)) + (a*v - b*u)*sin(t),
            0, 0, 0, 1;
    return matrix;
}

Eigen::MatrixXd calib_imu_lidar::pinv(Eigen::MatrixXd  A)//求矩阵的广义逆
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);//M=USV*
    double  pinvtoler = 1.e-8; //tolerance
    int row = A.rows();
    int col = A.cols();
    int k = min(row,col);
    Eigen::MatrixXd X = Eigen::MatrixXd::Zero(col,row);
    Eigen::MatrixXd singularValues_inv = svd.singularValues();//奇异值
    Eigen::MatrixXd singularValues_inv_mat = Eigen::MatrixXd::Zero(col, row);
    for (long i = 0; i<k; ++i) {
        if (singularValues_inv(i) > pinvtoler)
            singularValues_inv(i) = 1.0 / singularValues_inv(i);
        else singularValues_inv(i) = 0;
    }
    for (long i = 0; i < k; ++i)
    {
        singularValues_inv_mat(i, i) = singularValues_inv(i);
    }
    X=(svd.matrixV())*(singularValues_inv_mat)*(svd.matrixU().transpose());//X=VS+U*

    return X;

}
calib_imu_lidar::calib_imu_lidar()
{
    imu_z=0.274;
}

