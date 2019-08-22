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
#include "velodyne/parser/write_yaml.h"
#include "pcl-1.7/pcl/io/pcd_io.h"
#include "pcl-1.7/pcl/io/ply_io.h"
#include "proto/novatel_ins.pb.h"
#include <iomanip>
// #pragma once

#include <iostream>
#include<vector>
#include<math.h>
#include<fstream>


#include <string>
#include <algorithm>

#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <fstream>
#include <iostream>
#include "velodyne/parser/cal_eight.h"


using namespace std;

cal_eight m_left;
cal_eight m_back;
cal_eight m_right;
pcTransEstimator es_left;
pcTransEstimator es_right;
pcTransEstimator es_back;
calib_imu_lidar a_left;
calib_imu_lidar a_right;
calib_imu_lidar a_back;

typedef pcl::PointXYZRGB PointT;
#define PI (3.1415926535897932346f)


pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_b(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr data_b(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr data_befor_b(new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_l(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr data_l(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr data_befor_l(new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_r(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr data_r(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr data_befor_r(new pcl::PointCloud<pcl::PointXYZ>);

// vector<double> angle;
// vector<utm> utm_list;
  
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
  cout<<"just a test"<<endl;

}



bool Calibrate_eight::calib_eight_back(
const std::shared_ptr < adu::common::sensor::PointCloud const>& point_cloud_first,
const std::shared_ptr < adu::common::sensor::INSPVA   const>& INSPVA,
std::shared_ptr< adu::common::sensor::PointCloud>& point_cloud_fusion){ 

   cout<<"hello back"<<endl; 
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

  m_back.compute_utm(lonlat1,utm_ins);

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
   
    

  if(m_back.angle.size()==0)
  {
   
    m_back.angle.push_back(INSPVA->azimuth());  
    m_back.utm_list.push_back(utm_ins);
    m_back.T_list.push_back(T);     

    *data_befor_b=*lidar; 

     pcl::ModelCoefficients::Ptr Normal_floor(new pcl::ModelCoefficients);
    int flag=1;
    if(flag==a_back.filter_floor(lidar,Normal_floor))
    {        
        cout<<"can't get a good floor normal"<<endl;
        return 0;
    }   
    a_back.turn_vetical(lidar,Normal_floor);
    m_back.m_xy=a_back.M_XY;   
    m_back.lidar_height=a_back.height-a_back.imu_z;

     cout<<"M_XY"<<a_back.M_XY<<endl;
     cout<<"height="<<a_back.height<<endl;
  }
   
  if(abs(INSPVA->azimuth()-m_back.angle.back())>10&& !m_back.init_calib )
  {
    //angle数组会一直增加，记得后面释放
    m_back.angle.push_back(INSPVA->azimuth());
    m_back.utm_list.push_back(utm_ins);
    m_back.T_list.push_back(T); 
    
    *data_b=*lidar;    
    Eigen::Affine3d tt;   
    es_back.estimate(*data_befor_b,*data_b,tt);

    *data_befor_b=*lidar;

    Eigen::Matrix3d rotation_matrix;
    rotation_matrix=tt.matrix().block<3,3>(0,0);
    Eigen::Vector3d eulerAngle=rotation_matrix.eulerAngles(2,1,0);

      if(eulerAngle(0)>PI/2)
      {
          eulerAngle(0)-=PI;
      }
     cout<<"点云turn:"<<eulerAngle<<endl;
      int i=m_back.angle.size();
      double deta_angle;
      deta_angle=m_back.angle.back()-m_back.angle[i-2];
     
      if(abs(-deta_angle-eulerAngle(0)*(180/PI))<1)
    {       


        if(INSPVA->azimuth()-m_back.angle[i-2]<0 && m_back.zheng)
        {
        
        int i=m_back.T_list.size();
        Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
      
        t=m_back.T_list[i-1].inverse()*m_back.T_list[i-2];
        Eigen::Isometry3d t_inv=t.inverse();        
        
        Eigen::Matrix2d R;
        R<<t_inv(0,0),t_inv(0,1),t_inv(1,0),t_inv(1,1);
         cout<<"back r:"<<R<<endl;
        
        m_back.R_vec.push_back(R);

        utm ins_car;
        ins_car.x=t_inv(0,3);
        ins_car.y=t_inv(1,3);
        m_back.tins_vec.push_back(ins_car);
      
        Eigen::Matrix2d tlidar;
        tlidar<<-tt(0,3),tt(1,3),
                -tt(1,3),-tt(0,3);
          
        m_back.tlidar_vec.push_back(tlidar);
        m_back.zheng_iter++;
          if(m_back.zheng_iter==5)
          {
            m_back.zheng=false;
          }
        
        }

        if(INSPVA->azimuth()-m_back.angle[i-2]>0 && m_back.fu)
        {
        
        int i=m_back.T_list.size();
        Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
      
        t=m_back.T_list[i-1].inverse()*m_back.T_list[i-2];
        Eigen::Isometry3d t_inv=t.inverse();
         cout<<"ins旋转矩阵："<<t_inv.matrix()<<endl;       
        
        Eigen::Matrix2d R;
        R<<t_inv(0,0),t_inv(0,1),t_inv(1,0),t_inv(1,1);
          cout<<"back r:"<<R<<endl;
       
        m_back.R_vec.push_back(R);

        utm ins_car;
        ins_car.x=t_inv(0,3);
        ins_car.y=t_inv(1,3);
        m_back.tins_vec.push_back(ins_car);
      
        Eigen::Matrix2d tlidar;
        tlidar<<-tt(0,3),tt(1,3),
                -tt(1,3),-tt(0,3);
          
        m_back.tlidar_vec.push_back(tlidar);
        m_back.fu_iter++;
          if(m_back.fu_iter==5)
          {
            m_back.fu=false;
          }
        }
       
       if(!m_back.zheng && !m_back.fu && m_back.ss)
      {
        m_back.zheng=true;
        m_back.fu=true;
        m_back.result_init=m_back.sent_x_y_angle(m_back.R_vec,m_back.tlidar_vec,m_back.tins_vec,m_back.lidar_height);
        m_back.init_calib=true;
        
        {
            //save the result
            m_back.result_init*=m_back.m_xy;

            write_result write;
            double time=floor(point_cloud_first->measurement_time());
            string as="velodyne16_back";
            write.save_extrinsics(as,time,m_back.result_init);
        }

        m_back.R_vec.clear();      
        m_back.tlidar_vec.clear();
        m_back.tins_vec.clear(); 

        m_back.angle.clear();
        m_back.T_list.clear();
        m_back.init_add=true;   

        cout<<" init_back result finished*******!!!!!!"<<endl;
        //return true;
        m_back.ss=false;
      } 
    }
    *data_befor_b=*lidar;
    
  }

  if(m_back.init_calib && m_back.start_save)
  {
        if(m_back.init_add)
        {
          m_back.angle.push_back(INSPVA->azimuth());
          m_back.T_list.push_back(T);
          *data_befor_b=*lidar;
          *tmp_b=*lidar;
          m_back.init_add=false;
          cout<<"m_back计算真值为："<<m_back.result_init.matrix() << endl;

          m_back.iter=0;
        }
        else{
          m_back.angle.push_back(INSPVA->azimuth());
          m_back.T_list.push_back(T);
          *data_b=*lidar;
          int i=m_back.T_list.size();

        // Eigen::Isometry3d T_ure_r = Eigen::Isometry3d::Identity();
        // Eigen::Quaterniond q(0.706763,0.008225,-0.013502,0.707273);
        // // cout<<"四元数："<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<endl;
        // Eigen::AngleAxisd angle1(q);         
        // T_ure_r = angle1;

        // T_ure_r(0,3)=0.03;
        // T_ure_r(1,3)=0.1029;
        // T_ure_r(2,3)=1.5; 

        // cout<<"真值为："<<T_ure_r.matrix() << endl;                
        // Eigen::Isometry3d T_ure(T_ure_r);
        Eigen::Isometry3d T_ure(m_back.result_init);       
        Eigen::Isometry3d T_ure_inv=T_ure.inverse();
        Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
   
        t=m_back.T_list[i-1].inverse()*m_back.T_list[i-2];
        Eigen::Isometry3d t_inv=t.inverse();        

        Eigen::Isometry3d mult_t=T_ure_inv*t_inv*T_ure;

        Eigen::Isometry3d mult_t_inv=mult_t.inverse();

        pcl::PointCloud<pcl::PointXYZ>::Ptr amp(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud( *tmp_b, *amp, mult_t_inv.matrix());

         tmp_b->clear();
        *tmp_b=*amp+*data_b;

        if(m_back.iter==10)
        {
          pcl::io::savePLYFile ("final_back.ply", *tmp_b);
          cout<<"save the final back.ply***"<<endl<<endl;
          m_back.start_save=false;          
        }
        m_back.iter++;          
       
       }
     }
    //发布topic
    if(m_back.init_calib)
    {

       pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_result(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud( *lidar, *lidar_result, m_back.result_init);

          for(auto point : lidar_result->points){       
  
              adu::common::sensor::PointXYZIT* point_new = point_cloud_fusion->add_point();
              // point_new->set_intensity(point.intensity);
              // point_new->set_stamp(point.stamp);
              point_new->set_x(point.x);
              point_new->set_y(point.y);
              point_new->set_z(point.z);       
          }
          point_cloud_fusion->set_measurement_time(point_cloud_first->measurement_time()); 
         return 1;
    }        
 
}



bool Calibrate_eight::calib_eight_left(
const std::shared_ptr < adu::common::sensor::PointCloud const>& point_cloud_first,
const std::shared_ptr < adu::common::sensor::INSPVA   const>& INSPVA,
std::shared_ptr< adu::common::sensor::PointCloud>& point_cloud_fusion){ 

   cout<<"hello left"<<endl; 
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

  m_left.compute_utm(lonlat1,utm_ins);

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
   
    

  if(m_left.angle.size()==0)
  {
   
    m_left.angle.push_back(INSPVA->azimuth());  
    m_left.utm_list.push_back(utm_ins);
    m_left.T_list.push_back(T);     

    *data_befor_l=*lidar; 

     pcl::ModelCoefficients::Ptr Normal_floor(new pcl::ModelCoefficients);
    int flag=1;
    if(flag==a_left.filter_floor(lidar,Normal_floor))
    {        
        cout<<"can't get a good floor normal"<<endl;
        return 0;
    }   
    a_left.turn_vetical(lidar,Normal_floor);
    m_left.m_xy=a_left.M_XY;   
    m_left.lidar_height=a_left.height-a_left.imu_z;

     cout<<"M_XY"<<a_left.M_XY<<endl;
     cout<<"height="<<a_left.height<<endl;
  }
   
  if(abs(INSPVA->azimuth()-m_left.angle.back())>10&& !m_left.init_calib )
  {
    //angle数组会一直增加，记得后面释放
    m_left.angle.push_back(INSPVA->azimuth());
    m_left.utm_list.push_back(utm_ins);
    m_left.T_list.push_back(T); 
    
    *data_l=*lidar;    
    Eigen::Affine3d tt;   
    es_left.estimate(*data_befor_l,*data_l,tt);

    *data_befor_l=*lidar;

    Eigen::Matrix3d rotation_matrix;
    rotation_matrix=tt.matrix().block<3,3>(0,0);
    Eigen::Vector3d eulerAngle=rotation_matrix.eulerAngles(2,1,0);

    if(eulerAngle(0)>PI/2)
    {
        eulerAngle(0)-=PI;
    }
    // cout<<"点云turn:"<<eulerAngle<<endl;
    int i=m_left.angle.size();
      double deta_angle;
      deta_angle=m_left.angle.back()-m_left.angle[i-2];
     
      if(abs(-deta_angle-eulerAngle(0)*(180/PI))<1)
    {       


        if(INSPVA->azimuth()-m_left.angle[i-2]<0 && m_left.zheng)
        {
        
        int i=m_left.T_list.size();
        Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
      
        t=m_left.T_list[i-1].inverse()*m_left.T_list[i-2];
        Eigen::Isometry3d t_inv=t.inverse();        
        
        Eigen::Matrix2d R;
        R<<t_inv(0,0),t_inv(0,1),t_inv(1,0),t_inv(1,1);
        //  cout<<"left r:"<<R<<endl;
        
        m_left.R_vec.push_back(R);

        utm ins_car;
        ins_car.x=t_inv(0,3);
        ins_car.y=t_inv(1,3);
        m_left.tins_vec.push_back(ins_car);
      
        Eigen::Matrix2d tlidar;
        tlidar<<-tt(0,3),tt(1,3),
                -tt(1,3),-tt(0,3);
          
        m_left.tlidar_vec.push_back(tlidar);
        m_left.zheng_iter++;
          if(m_left.zheng_iter==5)
          {
            m_left.zheng=false;
          }
        
        }

        if(INSPVA->azimuth()-m_left.angle[i-2]>0 && m_left.fu)
        {
        
        int i=m_left.T_list.size();
        Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
      
        t=m_left.T_list[i-1].inverse()*m_left.T_list[i-2];
        Eigen::Isometry3d t_inv=t.inverse();
        // cout<<"ins旋转矩阵："<<t_inv.matrix()<<endl;       
        
        Eigen::Matrix2d R;
        R<<t_inv(0,0),t_inv(0,1),t_inv(1,0),t_inv(1,1);
        //  cout<<"left r:"<<R<<endl;
       
        m_left.R_vec.push_back(R);

        utm ins_car;
        ins_car.x=t_inv(0,3);
        ins_car.y=t_inv(1,3);
        m_left.tins_vec.push_back(ins_car);
      
        Eigen::Matrix2d tlidar;
        tlidar<<-tt(0,3),tt(1,3),
                -tt(1,3),-tt(0,3);
          
        m_left.tlidar_vec.push_back(tlidar);
        m_left.fu_iter++;
          if(m_left.fu_iter==5)
          {
            m_left.fu=false;
          }
        }
       
       if(!m_left.zheng && !m_left.fu && m_left.ss)
      {
        m_left.zheng=true;
        m_left.fu=true;
        m_left.result_init=m_left.sent_x_y_angle(m_left.R_vec,m_left.tlidar_vec,m_left.tins_vec,m_left.lidar_height);
        m_left.init_calib=true;
        
        {
            //save the result
            m_left.result_init*=m_left.m_xy;

            write_result write;
            double time=floor(point_cloud_first->measurement_time());
            string as="velodyne16_left";
            write.save_extrinsics(as,time,m_left.result_init);
        }

        m_left.R_vec.clear();      
        m_left.tlidar_vec.clear();
        m_left.tins_vec.clear(); 

        m_left.angle.clear();
        m_left.T_list.clear();
        m_left.init_add=true;   

        cout<<" init_left result finished*******!!!!!!"<<endl;
        //return true;
        m_left.ss=false;
      } 
    }
    
    *data_befor_l=*lidar;
  }

  if(m_left.init_calib && m_left.start_save)
  {
        if(m_left.init_add)
        {
          m_left.angle.push_back(INSPVA->azimuth());
          m_left.T_list.push_back(T);
          *data_befor_l=*lidar;
          *tmp_l=*lidar;
          m_left.init_add=false;
          cout<<"m_left计算真值为："<<m_left.result_init.matrix() << endl;

          m_left.iter=0;
        }
        else{
          m_left.angle.push_back(INSPVA->azimuth());
          m_left.T_list.push_back(T);
          *data_l=*lidar;
          int i=m_left.T_list.size();

        // Eigen::Isometry3d T_ure_r = Eigen::Isometry3d::Identity();
        // Eigen::Quaterniond q(0.706763,0.008225,-0.013502,0.707273);
        // // cout<<"四元数："<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<endl;
        // Eigen::AngleAxisd angle1(q);         
        // T_ure_r = angle1;

        // T_ure_r(0,3)=0.03;
        // T_ure_r(1,3)=0.1029;
        // T_ure_r(2,3)=1.5; 

        // cout<<"真值为："<<T_ure_r.matrix() << endl;                
        // Eigen::Isometry3d T_ure(T_ure_r);
        Eigen::Isometry3d T_ure(m_left.result_init);       
        Eigen::Isometry3d T_ure_inv=T_ure.inverse();
        Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
   
        t=m_left.T_list[i-1].inverse()*m_left.T_list[i-2];
        Eigen::Isometry3d t_inv=t.inverse();        

        Eigen::Isometry3d mult_t=T_ure_inv*t_inv*T_ure;

        Eigen::Isometry3d mult_t_inv=mult_t.inverse();

        pcl::PointCloud<pcl::PointXYZ>::Ptr amp(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud( *tmp_l, *amp, mult_t_inv.matrix());

         tmp_l->clear();
        *tmp_l=*amp+*data_l;

        if(m_left.iter==10)
        {
          pcl::io::savePLYFile ("final_left.ply", *tmp_l);
          cout<<"save the final left.ply***"<<endl<<endl;
          m_left.start_save=false;          
        }
        m_left.iter++;          
       
       }
     }
    //发布topic
    if(m_left.init_calib)
    {

       pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_result(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud( *lidar, *lidar_result, m_left.result_init);

          for(auto point : lidar_result->points){       
  
              adu::common::sensor::PointXYZIT* point_new = point_cloud_fusion->add_point();
              // point_new->set_intensity(point.intensity);
              // point_new->set_stamp(point.stamp);
              point_new->set_x(point.x);
              point_new->set_y(point.y);
              point_new->set_z(point.z);       
          }
          point_cloud_fusion->set_measurement_time(point_cloud_first->measurement_time()); 
         return 1;
    }        
 
}

bool Calibrate_eight::calib_eight_right(
const std::shared_ptr < adu::common::sensor::PointCloud const>& point_cloud_first,
const std::shared_ptr < adu::common::sensor::INSPVA   const>& INSPVA,
std::shared_ptr< adu::common::sensor::PointCloud>& point_cloud_fusion){ 

   cout<<"hello right"<<endl; 
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

  m_right.compute_utm(lonlat1,utm_ins);

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
   
    pcl::ModelCoefficients::Ptr Normal_floor(new pcl::ModelCoefficients);
    int flag=1;
    if(flag==a_right.filter_floor(lidar,Normal_floor))
    {        
        cout<<"can't get a good floor normal"<<endl;
        return 0;
    }   
    a_right.turn_vetical(lidar,Normal_floor);

  if(m_right.angle.size()==0)
  {
   
    m_right.angle.push_back(INSPVA->azimuth());  
    m_right.utm_list.push_back(utm_ins);
    m_right.T_list.push_back(T);   
    m_right.m_xy=a_right.M_XY;   
    m_right.lidar_height=a_right.height-a_right.imu_z;

     cout<<"M_XY"<<a_right.M_XY<<endl;
     cout<<"height="<<a_right.height<<endl;

    *data_befor_r=*lidar; 
  }
   
  if(abs(INSPVA->azimuth()-m_right.angle.back())>10&& !m_right.init_calib )
  {
    //angle数组会一直增加，记得后面释放
    m_right.angle.push_back(INSPVA->azimuth());
    m_right.utm_list.push_back(utm_ins);
    m_right.T_list.push_back(T); 
    
    *data_r=*lidar;    
    Eigen::Affine3d tt;   
    es_right.estimate(*data_befor_r,*data_r,tt);

    *data_befor_r=*lidar;

    Eigen::Matrix3d rotation_matrix;
    rotation_matrix=tt.matrix().block<3,3>(0,0);
    Eigen::Vector3d eulerAngle=rotation_matrix.eulerAngles(2,1,0);

    if(eulerAngle(0)>PI/2)
    {
        eulerAngle(0)-=PI;
    }
    // cout<<"点云turn:"<<eulerAngle<<endl;
    int i=m_right.angle.size();
      double deta_angle;
      deta_angle=m_right.angle.back()-m_right.angle[i-2];
     
      if(abs(-deta_angle-eulerAngle(0)*(180/PI))<1)
    {       


        if(INSPVA->azimuth()-m_right.angle[i-2]<0 && m_right.zheng)
        {
        
        int i=m_right.T_list.size();
        Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
      
        t=m_right.T_list[i-1].inverse()*m_right.T_list[i-2];
        Eigen::Isometry3d t_inv=t.inverse();        
        
        Eigen::Matrix2d R;
        R<<t_inv(0,0),t_inv(0,1),t_inv(1,0),t_inv(1,1);
        // cout<<"r:"<<R<<endl;
        
        m_right.R_vec.push_back(R);

        utm ins_car;
        ins_car.x=t_inv(0,3);
        ins_car.y=t_inv(1,3);
        m_right.tins_vec.push_back(ins_car);
      
        Eigen::Matrix2d tlidar;
        tlidar<<-tt(0,3),tt(1,3),
                -tt(1,3),-tt(0,3);
          
        m_right.tlidar_vec.push_back(tlidar);
        m_right.zheng_iter++;
          if(m_right.zheng_iter==5)
          {
            m_right.zheng=false;
          }
        
        }

        if(INSPVA->azimuth()-m_right.angle[i-2]>0 && m_right.fu)
        {
        
        int i=m_right.T_list.size();
        Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
      
        t=m_right.T_list[i-1].inverse()*m_right.T_list[i-2];
        Eigen::Isometry3d t_inv=t.inverse();
        // cout<<"ins旋转矩阵："<<t_inv.matrix()<<endl;       
        
        Eigen::Matrix2d R;
        R<<t_inv(0,0),t_inv(0,1),t_inv(1,0),t_inv(1,1);
        // cout<<"r:"<<R<<endl;
       
        m_right.R_vec.push_back(R);

        utm ins_car;
        ins_car.x=t_inv(0,3);
        ins_car.y=t_inv(1,3);
        m_right.tins_vec.push_back(ins_car);
      
        Eigen::Matrix2d tlidar;
        tlidar<<-tt(0,3),tt(1,3),
                -tt(1,3),-tt(0,3);
          
        m_right.tlidar_vec.push_back(tlidar);
        m_right.fu_iter++;
          if(m_right.fu_iter==5)
          {
            m_right.fu=false;
          }
        }
       
       if(!m_right.zheng && !m_right.fu && m_right.ss)
      {
        m_right.zheng=true;
        m_right.fu=true;
        m_right.result_init=m_right.sent_x_y_angle(m_right.R_vec,m_right.tlidar_vec,m_right.tins_vec,m_right.lidar_height);
        m_right.init_calib=true;
        
        {
            //save the result
            m_right.result_init*=m_right.m_xy;

            write_result write;
            double time=floor(point_cloud_first->measurement_time());
            string as="velodyne16_right";
            write.save_extrinsics(as,time,m_right.result_init);
        }

        m_right.R_vec.clear();      
        m_right.tlidar_vec.clear();
        m_right.tins_vec.clear(); 

        m_right.angle.clear();
        m_right.T_list.clear();
        m_right.init_add=true;   

        cout<<" init right result finished*******!!!!!!"<<endl;
        //return true;
        m_right.ss=false;
      } 
    }
    
    *data_befor_r=*lidar;
  }

  if(m_right.init_calib && m_right.start_save)
  {
        if(m_right.init_add)
        {
          m_right.angle.push_back(INSPVA->azimuth());
          m_right.T_list.push_back(T);
          *data_befor_r=*lidar;
          *tmp_r=*lidar;
          m_right.init_add=false;
          cout<<"m_right计算真值为："<<m_right.result_init.matrix() << endl;

          m_right.iter=0;
        }
        else{
          m_right.angle.push_back(INSPVA->azimuth());
          m_right.T_list.push_back(T);
          *data_r=*lidar;
          int i=m_right.T_list.size();

        // Eigen::Isometry3d T_ure_r = Eigen::Isometry3d::Identity();
        // Eigen::Quaterniond q(0.706763,0.008225,-0.013502,0.707273);
        // // cout<<"四元数："<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<endl;
        // Eigen::AngleAxisd angle1(q);         
        // T_ure_r = angle1;

        // T_ure_r(0,3)=0.03;
        // T_ure_r(1,3)=0.1029;
        // T_ure_r(2,3)=1.5; 

        // cout<<"真值为："<<T_ure_r.matrix() << endl;                
        // Eigen::Isometry3d T_ure(T_ure_r);
        Eigen::Isometry3d T_ure(m_right.result_init);       
        Eigen::Isometry3d T_ure_inv=T_ure.inverse();
        Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
   
        t=m_right.T_list[i-1].inverse()*m_right.T_list[i-2];
        Eigen::Isometry3d t_inv=t.inverse();        

        Eigen::Isometry3d mult_t=T_ure_inv*t_inv*T_ure;

        Eigen::Isometry3d mult_t_inv=mult_t.inverse();

        pcl::PointCloud<pcl::PointXYZ>::Ptr amp(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud( *tmp_r, *amp, mult_t_inv.matrix());

         tmp_r->clear();
        *tmp_r=*amp+*data_r;

        if(m_right.iter==10)
        {
          pcl::io::savePLYFile ("final_right.ply", *tmp_r);
          cout<<"save the final.ply***"<<endl<<endl;
          m_right.start_save=false;          
        }
        m_right.iter++;          
       
       }
     }
    //发布topic
    if(m_right.init_calib)
    {
       pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_result(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud( *lidar, *lidar_result, m_right.result_init);

          for(auto point : lidar_result->points){       
  
              adu::common::sensor::PointXYZIT* point_new = point_cloud_fusion->add_point();
              // point_new->set_intensity(point.intensity);
              // point_new->set_stamp(point.stamp);
              point_new->set_x(point.x);
              point_new->set_y(point.y);
              point_new->set_z(point.z);       
          }
          point_cloud_fusion->set_measurement_time(point_cloud_first->measurement_time()); 
       return 1;
    }        
 
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
