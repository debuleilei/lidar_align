/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "velodyne/parser/calib_eight.h"
#include "pcl-1.7/pcl/io/pcd_io.h"
#include "pcl-1.7/pcl/io/ply_io.h"
#include "proto/novatel_ins.pb.h"
#include <iomanip>
// #pragma once

#include <iostream>
#include<vector>
#include<math.h>
#include<fstream>
#include "pointmatcher/PointMatcher.h"
#include <string>
#include <algorithm>
#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>


#include "velodyne/parser/write_yaml.h"


#include <fstream>
#include <iostream>

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


using namespace std;

typedef pcl::PointXYZRGB PointT;
#define PI (3.1415926535897932346f)

  vector<double> angle;
  vector<utm> utm_list;
  vector<utm> car_position;
  vector<Eigen::Isometry3d> T_list;

  // pcl::PointCloud<pcl::PointXYZ>::Ptr refer(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr data(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr data_befor(new pcl::PointCloud<pcl::PointXYZ>);

  vector<Eigen::Matrix2d>R_vec;
  vector<utm>tins_vec;
  vector<Eigen::Matrix2d>lidar_vec;

  bool zheng=true;
  bool fu=true;
  bool zheng_=true;
  bool fu_=true;
  bool init_calib=false;
  bool ss=true;
  bool init_add=true;
  int iter=0;
  // pcTransEstimator es;


  Eigen::Matrix4d result_init;
  Eigen::Matrix4d m_xy;

  vector<Eigen::Isometry3d> imu_odo;
  vector<Eigen::Matrix4d> lidar_odo;
   vector<double> an;
  //  vector<double> angle_fu;
  //  bool turn=true;
  //  bool jump=true;
  //  bool jump_=true;
  int iter_z=0;
  int iter_f=0;
  int last=0;



  // vector<double> insp_list;
  // pcl::PointCloud<pcl::PointXYZ>::Ptr left_before(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr back_before(new pcl::PointCloud<pcl::PointXYZ>);

  // vector<Eigen::Matrix2d>R_vec_lidar;
  // vector<utm>tins_vec_lidar;
  // vector<Eigen::Matrix2d>tlidar_vec_lidar;



  pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
  
 pcl::PLYWriter writer;
namespace cybertron {
namespace drivers {
namespace velodyne {

bool Calibrate_eight::calib_eight_lidar(
const std::shared_ptr < adu::common::sensor::PointCloud const>& point_cloud_first,
const std::shared_ptr < adu::common::sensor::PointCloud const>& point_cloud_second,
const std::shared_ptr < adu::common::sensor::INSPVA   const>& INSPVA,
std::shared_ptr< adu::common::sensor::PointCloud>& point_cloud_fusion)
{  
    cout<<"just test"<<endl;
}



bool Calibrate_eight::calib_eight_back(
const std::shared_ptr < adu::common::sensor::PointCloud const>& point_cloud_first,
const std::shared_ptr < adu::common::sensor::INSPVA   const>& INSPVA,
std::shared_ptr< adu::common::sensor::PointCloud>& point_cloud_fusion){ 

  
  // cout<<"INSPVA.latitude:"<<INSPVA->latitude()<<endl;
  // cout<<"INSPVA.longitude:"<<INSPVA->longitude()<<endl; 
  // cout<<"INSPVA.azimuth:"<<INSPVA->azimuth()<<endl; 
  pcl::PointCloud<pcl::PointXYZ>::Ptr lidar(new pcl::PointCloud<pcl::PointXYZ>);
    
    for (auto& point : point_cloud_first->point()) {
      if (!std::isnan(point.x()))
      {
        pcl::PointXYZ point1;    
        point1.x=point.x();
        point1.y=point.y();
        point1.z=point.z();       
        lidar->push_back(point1);
      }
    }


  vector<double> lonlat1(2);
    
  lonlat1[0]=INSPVA->latitude();
  lonlat1[1]=INSPVA->longitude();
 
  utm utm_ins;
  m.compute_utm(lonlat1,utm_ins);

  //计算odometry的矩阵
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  {  
      //  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
       Eigen::Matrix3d r;
       double angle_rad=INSPVA->azimuth()*(PI/180);
        
        r<<cos(angle_rad),sin(angle_rad),0,//***********注意角度信息***********
            -sin(angle_rad),cos(angle_rad),0,
            0,0,1;

        Eigen::AngleAxisd rotate_matrix(r);         
        T = rotate_matrix;
        T(0,3)=utm_ins.x;
        T(1,3)=utm_ins.y;
        T(2,3)=0;     
  } 
  if(T_list.size()==0){

      *data_befor=*lidar;
      cout<<"init*"<<endl;
      T_list.push_back(T);

      Eigen::Isometry3d imu_ini = Eigen::Isometry3d::Identity();
      imu_odo.push_back(imu_ini);

      Eigen::Matrix4d lidar_ini = Eigen::Matrix4d::Identity();
      lidar_odo.push_back(lidar_ini);

      *tmp=*lidar;

      angle.push_back(INSPVA->azimuth());
      // an.push_back(INSPVA->azimuth());
  }
  if(T_list.size()>0 && !init_calib)
  {
      // cout<<"hi!"<<endl;
      // if(INSPVA->azimuth()>an.back())
      // {

      //   // cout<<"247"<<endl;
      //   an.push_back(INSPVA->azimuth());
      //   turn=true;
      // }
      // else
      // {        
      //   an.push_back(INSPVA->azimuth());
        
      //   turn=false;
      // }
      //   // cout<<"hi!255"<<endl;
      // if(!turn && jump)
      // { 
      //   last=T_list.size();
      //   cout<<"last="<<last<<endl;
      //   angle_fu.push_back(INSPVA->azimuth());
      //   jump=false;
      // }

      // if(turn && jump_)
      // { 
      //   last=T_list.size();
      //   cout<<"last="<<last<<endl;
      //   angle.push_back(INSPVA->azimuth());
      //   jump_=false;
      // }
      
        //  cout<<"hi!263"<<endl;      

      T_list.push_back(T);
      Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
      int i=T_list.size();
      // cout<<"proc*"<<lidar_odo.size()<<endl;
      // cout<<lidar_odo.back().matrix()<<endl;
      t=T.inverse()*T_list[i-2];

      Eigen::Isometry3d imu_odometry;
      // imu_odometry=imu_odo.back()*t;
      imu_odometry=imu_odo.back()*t.inverse();
      imu_odo.push_back(imu_odometry);
      
      Eigen::Affine3d t_lidar;
      *data=*lidar;
      pcTransEstimator es;
      es.estimate(*data_befor,*data,t_lidar);
      *data_befor=*lidar;

      // cout<<"matrix:"<<t_lidar.matrix()<<endl;

      Eigen::Matrix4d lidar_odometry,t_lidar_matrix;
      t_lidar_matrix=t_lidar.matrix();
      // lidar_odometry=lidar_odo.back()*t_lidar_matrix.inverse();
      lidar_odometry=lidar_odo.back()*t_lidar_matrix;
      lidar_odo.push_back(lidar_odometry);
        // cout<<"hi!290"<<endl;
      //****************************save the lidar points***********************************
        // Eigen::Matrix4d inver;
        // inver=t_lidar_matrix.inverse();
        // pcl::PointCloud<pcl::PointXYZ>::Ptr amp(new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::transformPointCloud( *tmp, *amp, inver);

        // *amp+=*lidar;
        // tmp->clear();
        // *tmp=*amp;
        // if(i==50)
        // {
        //   cout<<"write******************************************"<<endl;
        //   pcl::io::savePLYFile ("final.ply", *tmp);
        // }
       

      //****************计算初值******************
        //筛选原则：旋转45度，加入8组数据，
        // Eigen::Matrix3d rotation_matrix;
        // rotation_matrix=imu_odo.back().matrix().block<3,3>(0,0);
        // Eigen::Vector3d eulerAngle=rotation_matrix.eulerAngles(2,1,0);
        // cout<<"imu 旋转角度："<<eulerAngle(0)*(180/PI)<<endl;        
           cout<<"hi!"<<endl;
        if(INSPVA->azimuth()-angle.back()>10 && iter_z<4  )
        {
            cout<<"hi!316"<<endl;
          angle.push_back(INSPVA->azimuth());
          Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
          
          t=imu_odo[last].inverse()*imu_odometry;
          if(abs(t(1,3))<1 )
          {
            cout<<"imu:"<<t.matrix()<<endl;
            Eigen::Matrix2d R;
            R<<t(0,0),t(0,1),t(1,0),t(1,1);
            // cout<<"r:"<<R<<endl;
              
            R_vec.push_back(R);

            utm ins_car;
            ins_car.x=t(0,3);
            ins_car.y=t(1,3);
            tins_vec.push_back(ins_car);
            
            Eigen::Matrix4d pc_transform;
            pc_transform=lidar_odo[last].inverse()*lidar_odometry;
            Eigen::Matrix2d tlidar;
            tlidar<<-pc_transform(0,3),pc_transform(1,3),
                    -pc_transform(1,3),-pc_transform(0,3);

            cout<<"lidar:"<<pc_transform<<endl;
            lidar_vec.push_back(tlidar);
            
            last=i;
            cout<<"zlast:"<<last<<endl;
            ++iter_z;
          }
          // else{
          //   last=i;
          // }          
        }
          // cout<<"hi!347"<<endl;  
         if(INSPVA->azimuth()-angle.back()<-10 && iter_f<4  )
        {
           cout<<"hi!350"<<endl; 
          angle.push_back(INSPVA->azimuth());          
          Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
          
          t=imu_odo[last].inverse()*imu_odometry;
          cout<<"qq:"<<abs(t(1,3))<<endl;
          if(abs(t(1,3))<1 )
          { 
             cout<<"imu:"<<t.matrix()<<endl;
          Eigen::Matrix2d R;
          R<<t(0,0),t(0,1),t(1,0),t(1,1);
          // cout<<"r:"<<R<<endl;
            
          R_vec.push_back(R);

          utm ins_car;
          ins_car.x=t(0,3);
          ins_car.y=t(1,3);
          tins_vec.push_back(ins_car);
          
          Eigen::Matrix4d pc_transform;
          pc_transform=lidar_odo[last].inverse()*lidar_odometry;
          Eigen::Matrix2d tlidar;
          tlidar<<-pc_transform(0,3),pc_transform(1,3),
                  -pc_transform(1,3),-pc_transform(0,3);
          cout<<"lidar:"<<pc_transform<<endl;
          lidar_vec.push_back(tlidar);
          
          last=i;
          cout<<"flast:"<<last<<endl;
          ++iter_f;          
          } 
          else
          {
            last=i;
          }         
        }
          // cout<<"hi!378"<<endl;
        if(iter_z==4 && iter_f==4)
        {
          
          double lidar_height=1.5;
          result_init=m.sent_x_y_angle(R_vec,lidar_vec,tins_vec);
          cout<<"result_init="<<result_init<<endl;
          init_calib=true;
        }


      //保存数据**************************************
     
      // if(lidar_odo.size()==300)
      // {
      //   ofstream rt_file_lidar,rt_file_imu;
        
      //   rt_file_lidar.open("lidar.txt");
      //   rt_file_imu.open("imu.txt");
      //   cout<<"2"<<endl;
      //   for (int i=0;i<300;i++)
      //   { 
      //     rt_file_lidar<<lidar_odo[i](0,3)<<" "<<lidar_odo[i](1,3)<<"\n";

      //     rt_file_imu<<imu_odo[i](0,3)<<" "<<imu_odo[i](1,3)<<"\n";
      //     //rt_file<<utm_list[i].x-utm_list[0].x<<" "<<utm_list[i].y-utm_list[0].y<<"\n";
      //   }

      // rt_file_lidar.close();
      // rt_file_imu.close();
      // cout<<"finish write!!!!!!!!!!!!"<<endl;
     
      // }  
       //****************************************
  }
     if(init_calib)
    {
        if(init_add)
        {
          angle.push_back(INSPVA->azimuth());
          T_list.push_back(T);
          *data_befor=*lidar;
          *tmp=*lidar;
          init_add=false;
          cout<<"m_back计算真值为："<<result_init.matrix() << endl;

          iter=0;
        }
        else{
          angle.push_back(INSPVA->azimuth());
          T_list.push_back(T);
          *data=*lidar;
          int i=T_list.size();                
        
        Eigen::Isometry3d T_ure(result_init);       
        Eigen::Isometry3d T_ure_inv=T_ure.inverse();
        Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
   
        t=T_list[i-1].inverse()*T_list[i-2];
        Eigen::Isometry3d t_inv=t.inverse();        

        Eigen::Isometry3d mult_t=T_ure_inv*t_inv*T_ure;

        Eigen::Isometry3d mult_t_inv=mult_t.inverse();       
        pcl::PointCloud<pcl::PointXYZ>::Ptr amp(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud( *tmp, *amp, mult_t_inv.matrix());

         tmp->clear();
        *tmp=*amp+*data;

        
        cout<<iter<<"帧点云叠加"<<endl;
        if(iter==20)
        {
          pcl::io::savePLYFile ("final_back.ply", *tmp);
          cout<<"save the final.ply***"<<endl<<endl;
          init_calib=false;          
        }
        iter++;
        }
    }

  // if(angle.size()==0)
  // {
  //   angle.push_back(INSPVA->azimuth());
  //   utm_list.push_back(utm_ins);
  //   T_list.push_back(T);

    

  //   pcl::PointCloud<pcl::PointXYZRGB>::Ptr lidar_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  //   for (auto& point : lidar->points) {
  //     if (!std::isnan(point.x))
  //     {
  //       pcl::PointXYZRGB point1;    
  //       point1.x=point.x;
  //       point1.y=point.y;
  //       point1.z=point.z;
  //       point1.g=255;        
  //       lidar_rgb->push_back(point1);
  //     }
  //   }

    
  //   *data_befor=*lidar; 
  // }



  // if(abs(INSPVA->azimuth()-angle.back())>10 && !init_calib )
  // {   
  //   angle.push_back(INSPVA->azimuth());
  //   utm_list.push_back(utm_ins); 
  //   T_list.push_back(T);
    
  //   *data=*lidar;    
  //   Eigen::Affine3d tt;
 
  //   pcTransEstimator es;
  //   es.estimate(*data_befor,*data,tt);

  //   *data_befor=*lidar;
   
    
  //   // Eigen::Matrix3d rotation_matrix;
  //   // rotation_matrix=tt.matrix().block<3,3>(0,0);
  //   // Eigen::Vector3d eulerAngle=rotation_matrix.eulerAngles(2,1,0);

     
  //   // if(eulerAngle(0)>PI/2)
  //   // {
  //   //     eulerAngle(0)-=PI;
  //   // }
  
  //   //   int i=angle.size();
  //   //   double deta_angle;
  //   //   deta_angle=angle.back()-angle[i-2];
     
  //   //   if(abs(-deta_angle-eulerAngle(0)*(180/PI))<1)
  //   // {       

  //   //     if(INSPVA->azimuth()-angle[i-2]<0 && zheng)
  //   //     {
        
  //   //     int i=T_list.size();
  //   //     Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
      
  //   //     t=T_list[i-1].inverse()*T_list[i-2];
  //   //     Eigen::Isometry3d t_inv=t.inverse();        
        
  //   //     Eigen::Matrix2d R;
  //   //     R<<t_inv(0,0),t_inv(0,1),t_inv(1,0),t_inv(1,1);
  //   //     cout<<"r:"<<R<<endl;
  //   //     // m.compute_ins_r(ins_theta,R);
  //   //     R_vec.push_back(R);

  //   //     utm ins_car;
  //   //     ins_car.x=t_inv(0,3);
  //   //     ins_car.y=t_inv(1,3);
  //   //     tins_vec.push_back(ins_car);
      
  //   //     Eigen::Matrix2d tlidar;
  //   //     tlidar<<-tt(0,3),tt(1,3),
  //   //             -tt(1,3),-tt(0,3);
          
  //   //     tlidar_vec.push_back(tlidar);
  //   //     zheng=false;
  //   //     }

  //   //     if(INSPVA->azimuth()-angle[i-2]>0 && fu)
  //   //     {
        
  //   //     int i=T_list.size();
  //   //     Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
      
  //   //     t=T_list[i-1].inverse()*T_list[i-2];
  //   //     Eigen::Isometry3d t_inv=t.inverse();
  //   //     // cout<<"ins旋转矩阵："<<t_inv.matrix()<<endl;       
        
  //   //     Eigen::Matrix2d R;
  //   //     R<<t_inv(0,0),t_inv(0,1),t_inv(1,0),t_inv(1,1);
  //   //     cout<<"r:"<<R<<endl;
  //   //     // m.compute_ins_r(ins_theta,R);
  //   //     R_vec.push_back(R);

  //   //     utm ins_car;
  //   //     ins_car.x=t_inv(0,3);
  //   //     ins_car.y=t_inv(1,3);
  //   //     tins_vec.push_back(ins_car);
      
  //   //     Eigen::Matrix2d tlidar;
  //   //     tlidar<<-tt(0,3),tt(1,3),
  //   //             -tt(1,3),-tt(0,3);
          
  //   //     tlidar_vec.push_back(tlidar);
  //   //     fu=false;
  //   //     }
       
  //   //    if(!zheng && !fu && ss)
  //   //   {
  //   //     zheng=true;
  //   //     fu=true;
  //   //     result_init=m.sent_x_y_angle(R_vec,tlidar_vec,tins_vec);
  //   //     init_calib=true;
        
  //   //     {
  //   //         //save the result
  //   //         result_init*=m_xy;

  //   //         write_result write;
  //   //         double time=floor(point_cloud_first->measurement_time());
  //   //         string as="velodyne16_back";
  //   //         write.save_extrinsics(as,time,result_init);
  //   //     }

  //   //     R_vec.clear();      
  //   //     tlidar_vec.clear();
  //   //     tins_vec.clear(); 
  //   //     init_add=true;   

  //   //     cout<<" init result finished*******!!!!!!"<<endl;
  //   //     //return true;
  //   //     ss=false;
  //   //   } 
  //   // }
    
  //   // *data_befor=*lidar;
  //   //最优化求解

  // }
  //  if(init_calib)
  //   {
  //       if(init_add)
  //       {
  //         angle.push_back(INSPVA->azimuth());
  //         T_list.push_back(T);
  //         *data_befor=*lidar;
  //         *tmp=*lidar;
  //         init_add=false;
  //         cout<<"计算真值为："<<result_init.matrix() << endl;

         
  //       }
  //       else{
  //         angle.push_back(INSPVA->azimuth());
  //         T_list.push_back(T);
  //         *data=*lidar;
  //         int i=T_list.size(); 

  //          //真值校验
  //       Eigen::Isometry3d T_ure_r = Eigen::Isometry3d::Identity();
  //       Eigen::Quaterniond q(0.706763,0.008225,-0.013502,0.707273);
  //       // cout<<"四元数："<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<endl;
  //       Eigen::AngleAxisd angle1(q);         
  //       T_ure_r = angle1;

  //       T_ure_r(0,3)=0.03;
  //       T_ure_r(1,3)=0.1029;
  //       T_ure_r(2,3)=1.5; 

  //       // cout<<"真值为："<<T_ure_r.matrix() << endl;               
        
  //       Eigen::Isometry3d T_ure(T_ure_r);
  //       // Eigen::Isometry3d T_ure(result_init);       
  //       Eigen::Isometry3d T_ure_inv=T_ure.inverse();
  //       Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
   
  //       t=T_list[i-1].inverse()*T_list[i-2];
  //       Eigen::Isometry3d t_inv=t.inverse();        

  //       Eigen::Isometry3d mult_t=T_ure_inv*t_inv*T_ure;

  //       Eigen::Isometry3d mult_t_inv=mult_t.inverse();       
  //       pcl::PointCloud<pcl::PointXYZ>::Ptr amp(new pcl::PointCloud<pcl::PointXYZ>);
  //       pcl::transformPointCloud( *tmp, *amp, mult_t_inv.matrix());

  //        tmp->clear();
  //       *tmp=*amp+*data;
        
  //       cout<<iter<<"帧点云叠加"<<endl;
  //       if(iter==100)
  //       {
  //         pcl::io::savePLYFile ("final.ply", *tmp);
  //         cout<<"save the final.ply***"<<endl<<endl;
  //         init_calib=false;          
  //       }
  //       iter++;
        
       
  //      }
  //   }
}
bool Calibrate_eight::velodyne_add(const std::shared_ptr<adu::common::sensor::PointCloud const>& point_cloud_left,
                  const std::shared_ptr<adu::common::sensor::PointCloud const>& point_cloud_right,
                  const std::shared_ptr<adu::common::sensor::PointCloud const>& point_cloud_back,
                  std::shared_ptr<adu::common::sensor::PointCloud>& point_cloud_fusion)
{
    
    if(point_cloud_left->point_size()==0 || point_cloud_right->point_size()==0 || point_cloud_back->point_size()==0)
    {
        cout<<"velodyne_three_add sth is empty!"<<endl;
        return 0;
    }
    // cout<<"left*******************"<<point_cloud_left->measurement_time()<<endl;
    // cout<<"right*******************"<<point_cloud_right->measurement_time()<<endl;
    // cout<<"back*******************"<<point_cloud_back->measurement_time()<<endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr left(new pcl::PointCloud<pcl::PointXYZRGB>);    
    for (auto& point : point_cloud_left->point()) {
    
        pcl::PointXYZRGB point1;    
        point1.x=point.x();
        point1.y=point.y();
        point1.z=point.z();
        point1.r=255;
        left->push_back(point1);            
    }
    

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr right(new pcl::PointCloud<pcl::PointXYZRGB>);    
    for (auto& point : point_cloud_right->point()) {
    
        pcl::PointXYZRGB point1;    
        point1.x=point.x();
        point1.y=point.y();
        point1.z=point.z();
        point1.b=255;
        right->push_back(point1);            
    }
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr back(new pcl::PointCloud<pcl::PointXYZRGB>);    
    for (auto& point : point_cloud_back->point()) {
    
        pcl::PointXYZRGB point1;    
        point1.x=point.x();
        point1.y=point.y();
        point1.z=point.z();
        point1.g=255;
        back->push_back(point1);            
    }
    
    pcl::PLYWriter writer;
    writer.write<pcl::PointXYZRGB>("left.ply", *left, false);
    writer.write<pcl::PointXYZRGB>("right.ply", *right, false);
    writer.write<pcl::PointXYZRGB>("back.ply", *back, false);
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr add(new pcl::PointCloud<pcl::PointXYZRGB>); 
    *add=*left;
    *add+=*right;
    *add+=*back;
    writer.write<pcl::PointXYZRGB>("cal_data/add.ply", *add, false);

    if(add->points.size()>100)
    {
        cout<<"velodyne 3 lidars have successfully calibrate******"<<endl;

        for(auto& point : add->points){
            if(!std::isnan(point.x) )
            {                                
                adu::common::sensor::PointXYZIT* point_new = point_cloud_fusion->add_point();
                // point_new->set_intensity(point.intensity());
                // point_new->set_stamp(point.stamp());
                point_new->set_x(point.x);
                point_new->set_y(point.y);
                point_new->set_z(point.z);
        
            }
        }
        point_cloud_fusion->set_measurement_time(point_cloud_left->measurement_time());


    }
    else
    {
        cout<<"failure calibrate********************"<<endl;
    }
    
    return true;
}

}  // namespace velodyne
}
}