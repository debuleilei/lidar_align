#include "velodyne/parser/cal_eight.h"
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>




using namespace std;
using namespace Eigen;

bool  cal_eight::compute_utm (vector<double>& latilong,utm& utm)
{
    
    std::string zone;
 
    // gps_common::LLtoUTM(latilong[0], latilong[1],utm.x,utm.y, zone); //右手系，x指北，ｙ指东,顺时针＋
     gps_common::LLtoUTM(latilong[0], latilong[1],utm.y,utm.x, zone);//左手系，y指北，x指东，顺时针＋//什么也不是
    // cout << setprecision(20);
    // cout<<"utm:x="<<utm.x<<" y="<<utm.y<<endl;
}
bool cal_eight::trans_car( utm& deta_utm,double angle,utm& car)
{
    double length;
    length=sqrt(deta_utm.x*deta_utm.x+deta_utm.y*deta_utm.y);

     double deta_angle;//转角
     deta_angle=atan(deta_utm.y/deta_utm.x);



     car.x=-length*sin(deta_angle-angle);
     car.y=length*cos(deta_angle-angle);
}   

bool cal_eight::compute_ins_r(double z,Eigen::Matrix2d& r)
{   
   
    r<<cos(z),-sin(z),sin(z),cos(z);//sin(弧度)
    // cout<<"角度为："<<z<<" "<<sin(z)<<endl;

    // cout<<"r:"<<r<<endl;
    return true;
}

bool cal_eight::compute_ins_tr(vector<double>& utm1,vector<double>& utm2,vector<double>& utm3)
{
   
   // utm3=utm2-utm1;
   for(int i=0;i<2;++i)
   {
       utm3.push_back(utm2[i]-utm1[i]);
   }
//    cout<<"compute_ins_tr"<<endl;
   return true;
}
Eigen::Matrix4d cal_eight::sent_x_y_angle(vector<Eigen::Matrix2d>& r,vector<Eigen::Matrix2d>& lidar,vector<utm>& tins)
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

    //**********************检验＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊
        // cout<<"**********************检验＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊"<<endl;
        // Eigen::MatrixXd xx(4,1);
        // xx<<0.143,0,0,1;

        // cout<<"a:"<<A<<endl;
        // // cout<<"b:"<<b<<endl;

        // cout<<"result:"<<A*xx-b<<endl;
    //**********************************************************

            // Eigen::MatrixXd A(2*i,4);
            // A.block<2,2>(0,0)=r[0]-I;
            // A.block<2,2>(0,2)=lidar[0];

            // A.block<2,2>(2,0)=r[1]-I;
            // A.block<2,2>(2,2)=lidar[1];

            //  cout << "A:"<<A << endl << endl;

            // Eigen::MatrixXd X(4,1);
            // X<<-tins[0].x,-tins[0].y,-tins[1].x,-tins[1].y;
            //   cout <<"b:"<< b << endl << endl;
    
    // cout<<"***********************************************"<<endl;
    Eigen::MatrixXd X(4,1),Y(4,1);
    // X=A.fullPivHouseholderQr().solve(b);
    X=A.colPivHouseholderQr().solve(b);
    // cout<<"***********************************************"<<endl;
    
    // cout<<"***********************************************"<<endl;
    cout<<"外参X为："<<X <<endl;
    // cout<<"***********************************************"<<endl;

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
                0,0,1,0.4575830378778479,
                0,0,0,1; 
        //failure   result_init<<X(2,0),-X(3,0),0,X(0,0),
                // X(3,0),X(2,0),0,X(1,0),
                // 0,0,1,1.5,
                // 0,0,0,1;   

    return result_init;
    // Y=A.inverse()*b;
    // cout<<"外参Y为："<<Y<<endl;
    // cout<<"***********************************************"<<endl;
   
    // double as=X(0,0)*X(0,0)+X(1,0)*X(1,0);
    // cout<<"位移："<<sqrt(as)<<endl;
    // cout<<"角度为:"<<acos(X(2,0))<<endl;
}
bool cal_eight::sent_x_y_angle(Eigen::Matrix2d& r,Eigen::Matrix2d& lidar,vector<double>& tins)
{
   
   Eigen::Matrix2d I;
    I << 1, 0,
        0, 1;
   
    Eigen::MatrixXd A(2,4);
    A.block<2,2>(0,0)=r-I;
    A.block<2,2>(0,2)=lidar;
    cout << A << endl << endl;

    Eigen::MatrixXd X(2,1);
    X<<-tins[0],-tins[1];
    cout << X << endl << endl;

    cout<< A.fullPivHouseholderQr().solve(X)<<endl;

    // Eigen::MatrixXf m(4,4);
    // m <<  1, 2, 3, 4,
    //         5, 6, 7, 8,
    //          9,10,11,12,
    //           13,14,15,16;
    // cout << "Block " <<m<< endl;
    // cout << m.block<2,2>(1,1) << endl << endl;

}

bool cal_eight::sent_qt()
{
    /*
    res=(tx,ty,cos,sin)->transform
    */
    cout<<"hi"<<endl;    
    Eigen::MatrixXd res(4,1);

    res<<0.5,0.5,0.5,1;
    cout<<res<<endl;

    Eigen::Affine3d transform_2 = Eigen::Affine3d::Identity();
    float theta=acos(res(2,0));
     
    transform_2.rotate (Eigen::AngleAxisd (theta, Eigen::Vector3d::UnitZ()));
      
    Eigen::Matrix4d transform=transform_2.matrix();
    
    transform(0,3)=res(0,0);
    cout<<"hi"<<endl; 
    transform(1,3)=res(1,0);
    cout<<"hi"<<endl; 
    transform(2,3)=1.5;//height_back_imu
    

    // cout<<"矩阵为："<<endl;
    cout<<transform<<endl;

    Eigen::Quaterniond q;
    Eigen::Matrix3d R=transform.block(0,0,3,3);
    q=R;
    // cout<<"四元数："<<q.coeffs()<<endl;

    Eigen::Vector3d t;
    t<<res(0,0),res(1,0),1.5;
    // cout<<"平移矩阵为："<<t<<endl;
}
