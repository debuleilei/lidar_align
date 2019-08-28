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
// #include "pointmatcher/PointMatcher.h"
#include <string>
#include <algorithm>
#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>


#include "velodyne/parser/write_yaml.h"
// #include "velodyne/parser/filter_floor.h"

#include <fstream>
#include <iostream>


using namespace std;

typedef pcl::PointXYZRGB PointT;
#define PI (3.1415926535897932346f)

cal_eight m,m_l,m_r;

  // pcl::PointCloud<pcl::PointXYZ>::Ptr refer(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr data_back(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr data_befor_back(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_back(new pcl::PointCloud<pcl::PointXYZ>);

   pcl::PointCloud<pcl::PointXYZ>::Ptr data_left(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr data_befor_left(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_left(new pcl::PointCloud<pcl::PointXYZ>);

   pcl::PointCloud<pcl::PointXYZ>::Ptr data_right(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr data_befor_right(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_right(new pcl::PointCloud<pcl::PointXYZ>);
  
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
  if(m.T_list.size()==0){

      *data_befor_back=*lidar;
      cout<<"init*"<<endl;
      m.T_list.push_back(T);

      Eigen::Isometry3d imu_ini = Eigen::Isometry3d::Identity();
      m.imu_odo.push_back(imu_ini);

      Eigen::Matrix4d lidar_ini = Eigen::Matrix4d::Identity();
      m.lidar_odo.push_back(lidar_ini);

      // *tmp=*lidar;

      m.angle.push_back(INSPVA->azimuth());

      // pcl::ModelCoefficients::Ptr Normal_floor(new pcl::ModelCoefficients);
      // int flag=1;
      // if(flag==a_back.filter_floor(lidar,Normal_floor))
      // {        
      //     cout<<"can't get a good floor normal"<<endl;
      //     return 0;
      // }   
      // a_back.turn_vetical(lidar,Normal_floor);
     
  }
  if(m.T_list.size()>0 && !m.init_calib)
  {

      m.T_list.push_back(T);
      Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
      int i=m.T_list.size();
      // cout<<"proc*"<<lidar_odo.size()<<endl;
      // cout<<lidar_odo.back().matrix()<<endl;
      t=T.inverse()*m.T_list[i-2];

      Eigen::Isometry3d imu_odometry;
      // imu_odometry=imu_odo.back()*t;
      imu_odometry=m.imu_odo.back()*t.inverse();
      m.imu_odo.push_back(imu_odometry);
      
      Eigen::Affine3d t_lidar;
      *data_back=*lidar;
      pcTransEstimator es;
      es.estimate(*data_befor_back,*data_back,t_lidar);
      *data_befor_back=*lidar;

      // cout<<"matrix:"<<t_lidar.matrix()<<endl;

      Eigen::Matrix4d lidar_odometry,t_lidar_matrix;
      t_lidar_matrix=t_lidar.matrix();
      // lidar_odometry=lidar_odo.back()*t_lidar_matrix.inverse();
      lidar_odometry=m.lidar_odo.back()*t_lidar_matrix;
      m.lidar_odo.push_back(lidar_odometry);
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
      
           cout<<"hi!"<<endl;
        if(INSPVA->azimuth()-m.angle.back()>10 && m.iter_z<4  )
        {
          
          m.angle.push_back(INSPVA->azimuth());
          Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
          
          t=m.imu_odo[m.last].inverse()*imu_odometry;
          if(abs(t(1,3))<1 )
          {
            //cout<<"imu:"<<t.matrix()<<endl;
            Eigen::Matrix2d R;
            R<<t(0,0),t(0,1),t(1,0),t(1,1);
            
            m.R_vec.push_back(R);

            utm ins_car;
            ins_car.x=t(0,3);
            ins_car.y=t(1,3);
            m.tins_vec.push_back(ins_car);
            
            Eigen::Matrix4d pc_transform;
            pc_transform=m.lidar_odo[m.last].inverse()*lidar_odometry;
            Eigen::Matrix2d tlidar;
            tlidar<<-pc_transform(0,3),pc_transform(1,3),
                    -pc_transform(1,3),-pc_transform(0,3);

            cout<<"lidar:"<<pc_transform<<endl;
            m.lidar_vec.push_back(tlidar);
            
            m.last=i;
            cout<<"zlast:"<<m.last<<endl;
            ++m.iter_z;
          }
          else{
            m.last=i;
          }          
        }
         
         if(INSPVA->azimuth()-m.angle.back()<-10 && m.iter_f<4  )
        {          
          m.angle.push_back(INSPVA->azimuth());          
          Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
          
          t=m.imu_odo[m.last].inverse()*imu_odometry;
          cout<<"qq:"<<abs(t(1,3))<<endl;
          double len=abs(t(1,3));
          if(len<1 && len>0 )
          { 
            cout<<"qqqqq:"<<len<<endl;
            //  cout<<"imu:"<<t.matrix()<<endl;
          Eigen::Matrix2d R;
          R<<t(0,0),t(0,1),t(1,0),t(1,1);
          // cout<<"r:"<<R<<endl;
            
          m.R_vec.push_back(R);

          utm ins_car;
          ins_car.x=t(0,3);
          ins_car.y=t(1,3);
          m.tins_vec.push_back(ins_car);
          
          Eigen::Matrix4d pc_transform;
          pc_transform=m.lidar_odo[m.last].inverse()*lidar_odometry;
          Eigen::Matrix2d tlidar;
          tlidar<<-pc_transform(0,3),pc_transform(1,3),
                  -pc_transform(1,3),-pc_transform(0,3);
          cout<<"lidar:"<<pc_transform<<endl;
          m.lidar_vec.push_back(tlidar);
          
          m.last=i;
          cout<<"flast:"<<m.last<<endl;
          ++m.iter_f;          
          } 
          else
          {
            m.last=i;
          }         
        }
          // cout<<"hi!378"<<endl;
        if(m.iter_z==4 && m.iter_f==4)
        {
          
          m.lidar_height=1.5;
          m.result_init=m.sent_x_y_angle(m.R_vec,m.lidar_vec,m.tins_vec,m.lidar_height);
          cout<<"result_init="<<m.result_init<<endl;
          m.init_calib=true;

          { 
            write_result write;
            double time=floor(point_cloud_first->measurement_time());
            string as="velodyne16_back";
            write.save_extrinsics(as,time,m.result_init);
           }

        }


      //保存数据**************************************
     
      // if(m.lidar_odo.size()==100)
      // {
      //   ofstream rt_file_lidar,rt_file_imu;
        
      //   rt_file_lidar.open("lidar.txt");
      //   rt_file_imu.open("imu.txt");
      //   cout<<"2"<<endl;
      //   for (int i=0;i<100;i++)
      //   { 
      //     rt_file_lidar<<m.lidar_odo[i](0,3)<<" "<<m.lidar_odo[i](1,3)<<"\n";

      //     rt_file_imu<<m.imu_odo[i](0,3)<<" "<<m.imu_odo[i](1,3)<<"\n";
      //     //rt_file<<utm_list[i].x-utm_list[0].x<<" "<<utm_list[i].y-utm_list[0].y<<"\n";
      //   }

      // rt_file_lidar.close();
      // rt_file_imu.close();
      // cout<<"finish write!!!!!!!!!!!!"<<endl;
     
      // }  
       //****************************************
  }
     if(m.init_calib)
    {
        if(m.init_add)
        {
          m.angle.push_back(INSPVA->azimuth());
          m.T_list.push_back(T);
          *data_befor_back=*lidar;
          *tmp_back=*lidar;
          m.init_add=false;
          cout<<"m_back计算真值为："<<m.result_init.matrix() << endl;

          m.iter=0;
        }
        else{
          m.angle.push_back(INSPVA->azimuth());
          m.T_list.push_back(T);
          *data_back=*lidar;
          int i=m.T_list.size();                
        
        Eigen::Isometry3d T_ure(m.result_init);       
        Eigen::Isometry3d T_ure_inv=T_ure.inverse();
        Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
   
        t=m.T_list[i-1].inverse()*m.T_list[i-2];
        Eigen::Isometry3d t_inv=t.inverse();        

        Eigen::Isometry3d mult_t=T_ure_inv*t_inv*T_ure;

        Eigen::Isometry3d mult_t_inv=mult_t.inverse();       
        pcl::PointCloud<pcl::PointXYZ>::Ptr amp(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud( *tmp_back, *amp, mult_t_inv.matrix());

         tmp_back->clear();
        *tmp_back=*amp+*data_back;

        
        cout<<m.iter<<"帧点云叠加"<<endl;
        if(m.iter==20)
        {
          pcl::io::savePLYFile ("final_back.ply", *tmp_back);
          cout<<"save the final back.ply***"<<endl<<endl;
          m.init_calib=false;          
        }
        m.iter++;
        }
    }

     if(m.init_calib)
    {

       pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_result(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud( *lidar, *lidar_result, m.result_init);

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
  m_l.compute_utm(lonlat1,utm_ins);

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
  if(m_l.T_list.size()==0){

      *data_befor_left=*lidar;
      cout<<"init*"<<endl;
      m_l.T_list.push_back(T);

      Eigen::Isometry3d imu_ini = Eigen::Isometry3d::Identity();
      m_l.imu_odo.push_back(imu_ini);

      Eigen::Matrix4d lidar_ini = Eigen::Matrix4d::Identity();
      m_l.lidar_odo.push_back(lidar_ini);

      // *tmp=*lidar;

      m_l.angle.push_back(INSPVA->azimuth());
          
  }
  if(m_l.T_list.size()>0 && !m_l.init_calib)
  {

      m_l.T_list.push_back(T);
      Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
      int i=m_l.T_list.size();
      t=T.inverse()*m_l.T_list[i-2];

      Eigen::Isometry3d imu_odometry;
      // imu_odometry=imu_odo.back()*t;
      imu_odometry=m_l.imu_odo.back()*t.inverse();
      m_l.imu_odo.push_back(imu_odometry);
      
      Eigen::Affine3d t_lidar;
      *data_left=*lidar;
      pcTransEstimator es;
      es.estimate(*data_befor_left,*data_left,t_lidar);
      *data_befor_left=*lidar;


      Eigen::Matrix4d lidar_odometry,t_lidar_matrix;
      t_lidar_matrix=t_lidar.matrix();     
      lidar_odometry=m_l.lidar_odo.back()*t_lidar_matrix;
      m_l.lidar_odo.push_back(lidar_odometry);
      
      
           cout<<"left hi!"<<endl;
        if(INSPVA->azimuth()-m_l.angle.back()>10 && m_l.iter_z<4  )
        {
          
          m_l.angle.push_back(INSPVA->azimuth());
          Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
          
          t=m_l.imu_odo[m_l.last].inverse()*imu_odometry;
          if(abs(t(1,3))<1 )
          {
            //cout<<"imu:"<<t.matrix()<<endl;
            Eigen::Matrix2d R;
            R<<t(0,0),t(0,1),t(1,0),t(1,1);
            
            m_l.R_vec.push_back(R);

            utm ins_car;
            ins_car.x=t(0,3);
            ins_car.y=t(1,3);
            m_l.tins_vec.push_back(ins_car);
            
            Eigen::Matrix4d pc_transform;
            pc_transform=m_l.lidar_odo[m_l.last].inverse()*lidar_odometry;
            Eigen::Matrix2d tlidar;
            tlidar<<-pc_transform(0,3),pc_transform(1,3),
                    -pc_transform(1,3),-pc_transform(0,3);

            cout<<"lidar:"<<pc_transform<<endl;
            m_l.lidar_vec.push_back(tlidar);
            
            m_l.last=i;
            cout<<"zlast:"<<m_l.last<<endl;
            ++m_l.iter_z;
          }
          else{
            m_l.last=i;
          }          
        }
         
         if(INSPVA->azimuth()-m_l.angle.back()<-10 && m_l.iter_f<4  )
        {          
          m_l.angle.push_back(INSPVA->azimuth());          
          Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
          
          t=m_l.imu_odo[m_l.last].inverse()*imu_odometry;
          cout<<"qq:"<<abs(t(1,3))<<endl;
          double len=abs(t(1,3));
          if(len<1 && len>0 )
          { 
            cout<<"qqqqq:"<<len<<endl;
            //  cout<<"imu:"<<t.matrix()<<endl;
          Eigen::Matrix2d R;
          R<<t(0,0),t(0,1),t(1,0),t(1,1);
          // cout<<"r:"<<R<<endl;
            
          m_l.R_vec.push_back(R);

          utm ins_car;
          ins_car.x=t(0,3);
          ins_car.y=t(1,3);
          m_l.tins_vec.push_back(ins_car);
          
          Eigen::Matrix4d pc_transform;
          pc_transform=m_l.lidar_odo[m_l.last].inverse()*lidar_odometry;
          Eigen::Matrix2d tlidar;
          tlidar<<-pc_transform(0,3),pc_transform(1,3),
                  -pc_transform(1,3),-pc_transform(0,3);
          cout<<"lidar:"<<pc_transform<<endl;
          m_l.lidar_vec.push_back(tlidar);
          
          m_l.last=i;
          cout<<"flast:"<<m_l.last<<endl;
          ++m_l.iter_f;          
          } 
          else
          {
            m_l.last=i;
          }         
        }
   
        if(m_l.iter_z==4 && m_l.iter_f==4)
        {
          
          m_l.lidar_height=0.48;
          m_l.result_init=m_l.sent_x_y_angle(m_l.R_vec,m_l.lidar_vec,m_l.tins_vec,m_l.lidar_height);
          cout<<"left result_init="<<m_l.result_init<<endl;
          m_l.init_calib=true;

          {
            write_result write;
            double time=floor(point_cloud_first->measurement_time());
            string as="velodyne16_left";
            write.save_extrinsics(as,time,m_l.result_init);
           }
        }
 
  }
     if(m_l.init_calib)
    {
        if(m_l.init_add)
        {
          m_l.angle.push_back(INSPVA->azimuth());
          m_l.T_list.push_back(T);
          *data_befor_left=*lidar;
          *tmp_left=*lidar;
          m_l.init_add=false;
          cout<<"m_left计算真值为："<<m_l.result_init.matrix() << endl;

          m_l.iter=0;
        }
        else{
          m_l.angle.push_back(INSPVA->azimuth());
          m_l.T_list.push_back(T);
          *data_left=*lidar;
          int i=m_l.T_list.size();                
        
        Eigen::Isometry3d T_ure(m_l.result_init);       
        Eigen::Isometry3d T_ure_inv=T_ure.inverse();
        Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
   
        t=m_l.T_list[i-1].inverse()*m_l.T_list[i-2];
        Eigen::Isometry3d t_inv=t.inverse();        

        Eigen::Isometry3d mult_t=T_ure_inv*t_inv*T_ure;

        Eigen::Isometry3d mult_t_inv=mult_t.inverse();       
        pcl::PointCloud<pcl::PointXYZ>::Ptr amp(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud( *tmp_left, *amp, mult_t_inv.matrix());

         tmp_left->clear();
        *tmp_left=*amp+*data_left;

        
        cout<<m_l.iter<<"帧点云叠加"<<endl;
        if(m_l.iter==20)
        {
          pcl::io::savePLYFile ("final_left.ply", *tmp_left);
          cout<<"save the left.ply***"<<endl<<endl;
          m_l.init_calib=false;          
        }
        m_l.iter++;
        }
    }

     if(m_l.init_calib)
    {

       pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_result(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud( *lidar, *lidar_result, m_l.result_init);

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
  m_r.compute_utm(lonlat1,utm_ins);

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
  if(m_r.T_list.size()==0){

      *data_befor_right=*lidar;
      cout<<" right init*"<<endl;
      m_r.T_list.push_back(T);

      Eigen::Isometry3d imu_ini = Eigen::Isometry3d::Identity();
      m_r.imu_odo.push_back(imu_ini);

      Eigen::Matrix4d lidar_ini = Eigen::Matrix4d::Identity();
      m_r.lidar_odo.push_back(lidar_ini);

      m_r.angle.push_back(INSPVA->azimuth());
          
  }
  if(m_r.T_list.size()>0 && !m_r.init_calib)
  {

      m_r.T_list.push_back(T);
      Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
      int i=m_r.T_list.size();
      t=T.inverse()*m_r.T_list[i-2];

      Eigen::Isometry3d imu_odometry;
      // imu_odometry=imu_odo.back()*t;
      imu_odometry=m_r.imu_odo.back()*t.inverse();
      m_r.imu_odo.push_back(imu_odometry);
      
      Eigen::Affine3d t_lidar;
      *data_right=*lidar;
      pcTransEstimator es;
      es.estimate(*data_befor_right,*data_right,t_lidar);
      *data_befor_right=*lidar;


      Eigen::Matrix4d lidar_odometry,t_lidar_matrix;
      t_lidar_matrix=t_lidar.matrix();     
      lidar_odometry=m_r.lidar_odo.back()*t_lidar_matrix;
      m_r.lidar_odo.push_back(lidar_odometry);
      
      
           cout<<"right hi!"<<endl;
        if(INSPVA->azimuth()-m_r.angle.back()>10 && m_r.iter_z<4  )
        {
          
          m_r.angle.push_back(INSPVA->azimuth());
          Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
          
          t=m_r.imu_odo[m_r.last].inverse()*imu_odometry;
          if(abs(t(1,3))<1 )
          {
            cout<<"imu:"<<t.matrix()<<endl;
            Eigen::Matrix2d R;
            R<<t(0,0),t(0,1),t(1,0),t(1,1);
            
            m_r.R_vec.push_back(R);

            utm ins_car;
            ins_car.x=t(0,3);
            ins_car.y=t(1,3);
            m_r.tins_vec.push_back(ins_car);
            
            Eigen::Matrix4d pc_transform;
            pc_transform=m_r.lidar_odo[m_r.last].inverse()*lidar_odometry;
            Eigen::Matrix2d tlidar;
            tlidar<<-pc_transform(0,3),pc_transform(1,3),
                    -pc_transform(1,3),-pc_transform(0,3);

            cout<<"lidar:"<<pc_transform<<endl;
            m_r.lidar_vec.push_back(tlidar);
            
            m_r.last=i;
            cout<<"zlast:"<<m_r.last<<endl;
            ++m_r.iter_z;
          }
          // else{
          //   last=i;
          // }          
        }
         
         if(INSPVA->azimuth()-m_r.angle.back()<-10 && m_r.iter_f<4  )
        {          
          m_r.angle.push_back(INSPVA->azimuth());          
          Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
          
          t=m_r.imu_odo[m_r.last].inverse()*imu_odometry;
          cout<<"qq:"<<abs(t(1,3))<<endl;
          double len=abs(t(1,3));
          if(len<1 && len>0 )
          { 
            cout<<"qqqqq:"<<len<<endl;
            cout<<"imu:"<<t.matrix()<<endl;
          Eigen::Matrix2d R;
          R<<t(0,0),t(0,1),t(1,0),t(1,1);
          // cout<<"r:"<<R<<endl;
            
          m_r.R_vec.push_back(R);

          utm ins_car;
          ins_car.x=t(0,3);
          ins_car.y=t(1,3);
          m_r.tins_vec.push_back(ins_car);
          
          Eigen::Matrix4d pc_transform;
          pc_transform=m_r.lidar_odo[m_r.last].inverse()*lidar_odometry;
          Eigen::Matrix2d tlidar;
          tlidar<<-pc_transform(0,3),pc_transform(1,3),
                  -pc_transform(1,3),-pc_transform(0,3);
          cout<<"lidar:"<<pc_transform<<endl;
          m_r.lidar_vec.push_back(tlidar);
          
          m_r.last=i;
          cout<<"flast:"<<m_r.last<<endl;
          ++m_r.iter_f;          
          } 
          else
          {
            m_r.last=i;
          }         
        }
   
        if(m_r.iter_z==4 && m_r.iter_f==4)
        {
          
          m_r.lidar_height=0.48;
          m_r.result_init=m_r.sent_x_y_angle(m_r.R_vec,m_r.lidar_vec,m_r.tins_vec,m_r.lidar_height);
          cout<<"right result_init="<<m_r.result_init<<endl;
          m_r.init_calib=true;
        }
 
  }
     if(m_r.init_calib)
    {
        if(m_r.init_add)
        {
          m_r.angle.push_back(INSPVA->azimuth());
          m_r.T_list.push_back(T);
          *data_befor_right=*lidar;
          *tmp_right=*lidar;
          m_r.init_add=false;
          cout<<"m_right 计算真值为："<<m_r.result_init.matrix() << endl;

          m_r.iter=0;
        }
        else{
          m_r.angle.push_back(INSPVA->azimuth());
          m_r.T_list.push_back(T);
          *data_right=*lidar;
          int i=m_r.T_list.size();                
        
        Eigen::Isometry3d T_ure(m_r.result_init);       
        Eigen::Isometry3d T_ure_inv=T_ure.inverse();
        Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
   
        t=m_r.T_list[i-1].inverse()*m_r.T_list[i-2];
        Eigen::Isometry3d t_inv=t.inverse();        

        Eigen::Isometry3d mult_t=T_ure_inv*t_inv*T_ure;

        Eigen::Isometry3d mult_t_inv=mult_t.inverse();       
        pcl::PointCloud<pcl::PointXYZ>::Ptr amp(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud( *tmp_right, *amp, mult_t_inv.matrix());

         tmp_right->clear();
        *tmp_right=*amp+*data_right;

        
        cout<<m_r.iter<<"帧点云叠加"<<endl;
        if(m_r.iter==20)
        {
          pcl::io::savePLYFile ("final_right.ply", *tmp_right);
          cout<<"save the final right.ply***"<<endl<<endl;
          m_r.init_calib=false;          
        }
        m_r.iter++;
        }
    }

     if(m_r.init_calib)
    {

       pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_result(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud( *lidar, *lidar_result, m_r.result_init);

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
bool Calibrate_eight::velodyne_fusion(const std::shared_ptr<adu::common::sensor::PointCloud const>& point_cloud_left,
                  const std::shared_ptr<adu::common::sensor::PointCloud const>& point_cloud_right,                  
                  std::shared_ptr<adu::common::sensor::PointCloud>& point_cloud_fusion)
{
    
    if(point_cloud_left->point_size()==0 || point_cloud_right->point_size()==0)
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
    
   
    
    pcl::PLYWriter writer;
    writer.write<pcl::PointXYZRGB>("left.ply", *left, false);
    writer.write<pcl::PointXYZRGB>("right.ply", *right, false);
   
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr add(new pcl::PointCloud<pcl::PointXYZRGB>); 
    *add=*left;
    *add+=*right;
    
    writer.write<pcl::PointXYZRGB>("cal_data/fusion.ply", *add, false);

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