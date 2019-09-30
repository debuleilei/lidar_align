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
#include "velodyne/parser/aligner.h"
#include "velodyne/parser/sensors.h"


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
cal_eight m_lidar;
bool use_baidu_result=false;

pcl::PointCloud<pcl::PointXYZ>::Ptr data_1(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr data_befor_1(new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr data_2(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr data_befor_2(new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr data_lidar(new pcl::PointCloud<pcl::PointXYZ>);

vector<Eigen::Matrix4d> lidar1_odo;
vector<Eigen::Matrix4d> lidar2_odo;
vector<double> angle_lidar;
bool init_calib=false;
int last;
int iter_z=0;
int iter_f=0;
int i=0;
vector<Eigen::Matrix2d>R_vec;
vector<utm>tins_vec;
vector<Eigen::Matrix2d>lidar_vec;
Eigen::Matrix4d result_init;

string save_path_lidar="data/lidar/";
string save_path_odom="data/odom/";
vector<double>time_stamp_vec;
// vector<pcl::PointCloud<pcl::PointXYZ>>lidar_vec;
// vector<Eigen::Isometry3d> imu_odo_vec;

bool save_odom(string str,vector<Eigen::Isometry3d>&T,vector<double> time_stamp)
{
  ofstream csv_file;
  csv_file.open(str, ios::out); 
  
  const char kDelimiter = ',';

  csv_file << std::string("# timestamp [ns]") << kDelimiter << " vertex-id"
           << kDelimiter << " p_G_Ix [m]" << kDelimiter << " p_G_Iy [m]"
           << kDelimiter << " p_G_Iz [m]" << kDelimiter << " q_G_Iw"
           << kDelimiter << " q_G_Ix" << kDelimiter << " q_G_Iy" << kDelimiter
           << " q_G_Iz\n";
  
  for (int i=0;i<T.size()-1;++i) {
    
    Eigen::Matrix4d odom=T[i+1].matrix();
    const double timestamp_nanoseconds = time_stamp[i];    
    const Eigen::Vector3d p_G_I =odom.topRightCorner<3, 1>();

    Eigen::Matrix3d transform_matrix=odom.block(0,0,3,3);
    Eigen::Quaterniond q_G_I=Eigen::Quaterniond(transform_matrix);
    //  q_G_I = odom.topLeftCorner<3, 3>();
   

    csv_file <<setiosflags(ios::fixed)<<setprecision(6)<< timestamp_nanoseconds << kDelimiter << i
             << kDelimiter << p_G_I[0] << kDelimiter << p_G_I[1] << kDelimiter
             << p_G_I[2] << kDelimiter << q_G_I.w() << kDelimiter << q_G_I.x()
             << kDelimiter << q_G_I.y() << kDelimiter << q_G_I.z() << std::endl;

  }
  csv_file.close();
  return true;
}
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


bool Calibrate_eight::transform_lidar_regist(const std::shared_ptr<adu::common::sensor::PointCloud const>& point_cloud_first,
                  const std::shared_ptr<adu::common::sensor::PointCloud const>& point_cloud_second,                                   
                  std::shared_ptr<adu::common::sensor::PointCloud>& point_cloud_fusion,
                  const Eigen::Affine3d& lidar_extrinsic_,string path)
{
  cout<<"start transform regist"<<endl;
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

  // pcl::PointCloud<pcl::PointXYZ>::Ptr amp(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::transformPointCloud( *lidar, *amp, lidar_extrinsic_.matrix());

  pcl::PointCloud<pcl::PointXYZ>::Ptr back(new pcl::PointCloud<pcl::PointXYZ>);
    
    for (auto& point : point_cloud_second->point()) {
      if (!std::isnan(point.x()))
      {
        pcl::PointXYZ point1;    
        point1.x=point.x();
        point1.y=point.y();
        point1.z=point.z();       
        back->push_back(point1);
      }
    }

    Eigen::Affine3d l_b;    
    pcTransEstimator es;
    es.estimate(*lidar,*back,l_b);
    cout<<"l_b:"<<l_b.matrix()<<endl;
  //save the result
  {
    Eigen::Matrix4d result, ttmp,ttmp_inv;
    ttmp=l_b.matrix();
    ttmp_inv=ttmp.inverse();
    result=ttmp_inv*lidar_extrinsic_.matrix();
    write_result write;
    double time=floor(point_cloud_first->measurement_time());
    // string as="velodyne16_back";
    write.save_extrinsics(path,time,result);

    pcl::io::savePLYFile ("left.ply", *lidar);
    pcl::io::savePLYFile ("back.ply", *back);

    pcl::PointCloud<pcl::PointXYZ>::Ptr left_t(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud( *lidar, *left_t, ttmp_inv);
    pcl::io::savePLYFile ("left_trans.ply", *left_t);

     pcl::PointCloud<pcl::PointXYZ>::Ptr fusion_back_left(new pcl::PointCloud<pcl::PointXYZ>);
     *fusion_back_left+=*left_t;
     *fusion_back_left+=*back;
     pcl::io::savePLYFile ("fusion_back_left.ply", *fusion_back_left);



     if(left_t->points.size()>100)
    {
      for(auto point : left_t->points){       

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
  

  
}

bool Calibrate_eight::transform_lidar(const std::shared_ptr<adu::common::sensor::PointCloud const>& point_cloud_first,                                   
                  std::shared_ptr<adu::common::sensor::PointCloud>& point_cloud_fusion,
                  const Eigen::Affine3d& lidar_extrinsic_)
{
  cout<<"start transform"<<endl;
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

  pcl::PointCloud<pcl::PointXYZ>::Ptr amp(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud( *lidar, *amp, lidar_extrinsic_.matrix());
  if(amp->points.size()>100)
  {
    for(auto point : amp->points){       

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


bool Calibrate_eight::calib_eight_lidar(
const std::shared_ptr < adu::common::sensor::PointCloud const>& point_cloud_first,
const std::shared_ptr < adu::common::sensor::PointCloud const>& point_cloud_second,
const std::shared_ptr < adu::common::sensor::INSPVA   const>& INSPVA,
std::shared_ptr< adu::common::sensor::PointCloud>& point_cloud_fusion)
{  
    cout<<"just test"<<endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_1(new pcl::PointCloud<pcl::PointXYZ>);
    
    for (auto& point : point_cloud_first->point()) {
      if (!std::isnan(point.x()))
      {
        pcl::PointXYZ point1;    
        point1.x=point.x();
        point1.y=point.y();
        point1.z=point.z();       
        lidar_1->push_back(point1);
      }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_2(new pcl::PointCloud<pcl::PointXYZ>);
    
    for (auto& point : point_cloud_second->point()) {
      if (!std::isnan(point.x()))
      {
        pcl::PointXYZ point1;    
        point1.x=point.x();
        point1.y=point.y();
        point1.z=point.z();       
        lidar_2->push_back(point1);
      }
    }

    if(i==0){

      *data_befor_1=*lidar_1;
      *data_befor_2=*lidar_2;
      cout<<"lidar init*"<<endl;     

      Eigen::Matrix4d lidar_ini = Eigen::Matrix4d::Identity();
      lidar1_odo.push_back(lidar_ini);
      lidar2_odo.push_back(lidar_ini);
      angle_lidar.push_back(INSPVA->azimuth());
      last=0;
      // *tmp=*lidar;     
    }
    i++;

    if(angle_lidar.size()>0 && !init_calib)
    {
      
      
      {
        pcTransEstimator es;
        Eigen::Affine3d t_lidar;
        *data_1=*lidar_1;        
        es.estimate(*data_befor_1,*data_1,t_lidar);
        *data_befor_1=*lidar_1;

        Eigen::Matrix4d lidar_odometry,t_lidar_matrix;
        t_lidar_matrix=t_lidar.matrix();
        // lidar_odometry=lidar_odo.back()*t_lidar_matrix.inverse();
        lidar_odometry=lidar1_odo.back()*t_lidar_matrix;
        lidar1_odo.push_back(lidar_odometry);
      }

      {
        pcTransEstimator es1;
        Eigen::Affine3d t_lidar2;
        *data_2=*lidar_2;        
        es1.estimate(*data_befor_2,*data_2,t_lidar2);
        *data_befor_2=*lidar_2;

        Eigen::Matrix4d lidar_odometry2,t_lidar_matrix2;
        t_lidar_matrix2=t_lidar2.matrix();
        // lidar_odometry=lidar_odo.back()*t_lidar_matrix.inverse();
        lidar_odometry2=lidar2_odo.back()*t_lidar_matrix2;
        lidar2_odo.push_back(lidar_odometry2);
      }
      if(lidar2_odo.size()==300)
      {
        ofstream rt_file_lidar,rt_file_lidar2;
        
        rt_file_lidar.open("lidar1.txt");
        rt_file_lidar2.open("lidar2.txt");
        
        for (int i=0;i<300;i++)
        { 
          rt_file_lidar<<lidar1_odo[i](0,3)<<" "<<lidar1_odo[i](1,3)<<"\n";
          rt_file_lidar2<<lidar2_odo[i](0,3)<<" "<<lidar2_odo[i](1,3)<<"\n";

          // rt_file_imu<<m.imu_odo[i](0,3)<<" "<<m.imu_odo[i](1,3)<<"\n";
          //rt_file<<utm_list[i].x-utm_list[0].x<<" "<<utm_list[i].y-utm_list[0].y<<"\n";
        }

      rt_file_lidar.close();
      rt_file_lidar2.close();
      cout<<"finish write!!!!!!!!!!!!"<<endl;
     
      }  

      // if(INSPVA->azimuth()-angle_lidar.back()>10 && iter_z<4  )
      //   {
          
      //     angle_lidar.push_back(INSPVA->azimuth());
      //     Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
          
      //     t=lidar1_odo[last].inverse()*lidar1_odo.back();
      //     // cout<<"顺："<<abs(t(1,3))<<endl;
      //     double len=abs(t(1,3));
      //     if(len<1 && len>0 )
      //     {
      //       cout<<"lidar_1:"<<t.matrix()<<endl;
      //       Eigen::Matrix2d R;
      //       R<<t(0,0),t(0,1),t(1,0),t(1,1); 
            
      //       R_vec.push_back(R);

      //       utm ins_car;
      //       ins_car.x=t(0,3);
      //       ins_car.y=t(1,3);
      //       tins_vec.push_back(ins_car);
            
      //       Eigen::Matrix4d pc_transform;
      //       pc_transform=lidar2_odo[last].inverse()*lidar2_odo.back();
      //       Eigen::Matrix2d tlidar;
      //       tlidar<<-pc_transform(0,3),pc_transform(1,3),
      //               -pc_transform(1,3),-pc_transform(0,3);

      //       cout<<"lidar z:"<<pc_transform<<endl;
      //       lidar_vec.push_back(tlidar);
            
      //       last=i;
      //       cout<<"zlast:"<<last<<endl;
      //       iter_z++;
      //     }           
                 
      //   }

      //   if(INSPVA->azimuth()-angle_lidar.back()<-10 && iter_f<4  )
      //   {
          
      //     angle_lidar.push_back(INSPVA->azimuth());
      //     Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
          
      //     t=lidar1_odo[last].inverse()*lidar1_odo.back();
      //     // cout<<"顺："<<abs(t(1,3))<<endl;
      //     double len=abs(t(1,3));
      //     if(len<1 && len>0 )
      //     {
      //       cout<<"lidar_1:"<<t.matrix()<<endl;
      //       Eigen::Matrix2d R;
      //       R<<t(0,0),t(0,1),t(1,0),t(1,1); 
            
      //       R_vec.push_back(R);

      //       utm ins_car;
      //       ins_car.x=t(0,3);
      //       ins_car.y=t(1,3);
      //       tins_vec.push_back(ins_car);
            
      //       Eigen::Matrix4d pc_transform;
      //       pc_transform=lidar2_odo[last].inverse()*lidar2_odo.back();
      //       Eigen::Matrix2d tlidar;
      //       tlidar<<-pc_transform(0,3),pc_transform(1,3),
      //               -pc_transform(1,3),-pc_transform(0,3);

      //       cout<<"lidar f:"<<pc_transform<<endl;
      //       lidar_vec.push_back(tlidar);
            
      //       last=i;
      //       cout<<"flast:"<<last<<endl;
      //       iter_f++;
      //     }
      //     else
      //     {
      //       last=i;
      //     }                     
                 
      //   }
      //   if(iter_z==4 && iter_f==4)
      //   {
          
      //     double lidar_height=1.045;
      //     result_init=m_lidar.sent_x_y_angle(R_vec,lidar_vec,tins_vec,lidar_height);
      //     cout<<"result_init="<<result_init<<endl;
      //     init_calib=true;

      //     { 
      //       write_result write;
      //       double time=floor(point_cloud_first->measurement_time());
      //       string as="velodyne16_back";
      //       write.save_extrinsics(as,time,result_init);
      //      }

      //   }
    }
    // if(init_calib)
    // {
    //   cout<<"start transform*"<<endl;
    //   pcl::PointCloud<pcl::PointXYZ>::Ptr amp(new pcl::PointCloud<pcl::PointXYZ>);
      
    //   Eigen::Matrix4d a;
    //   a=result_init.inverse();
    //   pcl::transformPointCloud( *lidar_1,*amp, a);

    //     data_lidar->clear();
    //     *data_lidar=*amp+*lidar_2;

    //     pcl::io::savePLYFile ("fusion_lidar.ply", *data_lidar);

    // }

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
      // *combined_pointcloud_ptr=*lidar;
      m.angle.push_back(INSPVA->azimuth());

      // pcl::ModelCoefficients::Ptr Normal_floor(new pcl::ModelCoefficients);
      // int flag=1;
      // if(flag==a_back.filter_floor(lidar,Normal_floor))
      // {        
      //     cout<<"can't get a good floor normal"<<endl;
      //     return 0;
      // }   
      // a_back.turn_vetical(lidar,Normal_floor);
      // cout<<"平面方程："<<a_back.
     
  }
  // cout<<"angle.back="<<m.angle.back()<<endl;
  // if(m.T_list.size()>0 )
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


      // lidar.setOdomOdomTransforms(odom);
      // Aligner aligner(Aligner::getConfig(&nh_private));
      // aligner.lidarOdomTransform(&lidar, &odom);

      cout<<"start align"<<endl;
      using namespace lidar_align;

      
      LoaderPointcloud* pointcloud;
       cout<<"pointcloud.size():"<<pointcloud->size()<<endl;
      for(int i=0;i<lidar->points.size();++i) {
        PointAllFields point;
        point.x = lidar->points[i].x;
        point.y = lidar->points[i].y;
        point.z = lidar->points[i].z;
        
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z)) {
          continue;
        }

        pointcloud->push_back(point);
      }
      
       cout<<"pointcloud.size():"<<pointcloud->size()<<endl;
      // pointcloud->header = raw_pointcloud.header;
      Lidar lidar_;

      // lidar->addPointcloud(pointcloud, scan_config);
      Odom odom;
      cout<<"start align 1"<<endl;













      
      // Eigen::Affine3d t_lidar;
      // *data_back=*lidar;
      // pcTransEstimator es;
      // es.estimate(*data_befor_back,*data_back,t_lidar);
      // *data_befor_back=*lidar;

      // // cout<<"matrix:"<<t_lidar.matrix()<<endl;

      // Eigen::Matrix4d lidar_odometry,t_lidar_matrix;
      // t_lidar_matrix=t_lidar.matrix();
      // // lidar_odometry=lidar_odo.back()*t_lidar_matrix.inverse();
      // lidar_odometry=m.lidar_odo.back()*t_lidar_matrix;
      // m.lidar_odo.push_back(lidar_odometry);

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

      //*********************************pcd csv*******************************************
        // string cc_ply=".ply";
        // string lidar_str=save_path_lidar+to_string(i-1)+cc_ply;
        // cout<<lidar_str<<endl;
        // pcl::io::savePLYFile (lidar_str, *lidar);

        
        // // lidar_vec.emplace_back(*lidar);
        // //保存odmo


        // time_stamp_vec.emplace_back(point_cloud_first->measurement_time()*1000000ll);
             
        // if(time_stamp_vec.size()==100)
        // {
        //   string cc_csv="odom.csv";
        //   string odom_str=save_path_odom+cc_csv;
        //   cout<<odom_str<<endl;
          
        //   save_odom(odom_str,m.imu_odo,time_stamp_vec);

        //   string cc_timestamp=".csv";
        //   for(int i=0;i<100;++i)
        //   {
        //     string lidar_stamp_str=save_path_lidar+to_string(i+1)+cc_timestamp;
        //     ofstream csv_file;
        //     csv_file.open(lidar_stamp_str, ios::out);
        //     csv_file <<setiosflags(ios::fixed)<<setprecision(0)<<time_stamp_vec[i]<<endl;
        //     csv_file.close();
        //   }
        //   m.init_calib=true;

         
          // for(int i=0;i<100;++i)
          // {
          //     string cc_ply=".ply";
          //     string lidar_str=save_path_lidar+to_string(i)+cc_ply;
          //     cout<<lidar_str<<endl;
          //     pcl::io::savePLYFile (lidar_str, lidar_vec[i]);
          // }
          
       // }       

      //***********************************************************************************
      
        // cout<<"hi!"<<endl;


        // if(i-m.last>20)//防止点云的序列之后
        //   {
        //     cout<<"更新"<<endl;
        //     m.last=i;
        //     m.angle.push_back(INSPVA->azimuth());
        //   } 
        
        // if(INSPVA->azimuth()-m.angle.back()>10 && m.iter_z<4  )
        // {
          
        //   m.angle.push_back(INSPVA->azimuth());
        //   Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
          
        //   t=m.imu_odo[m.last].inverse()*imu_odometry;
        //   // cout<<"顺："<<abs(t(1,3))<<endl;
        //   double len=abs(t(1,3));
        //   if(len<1 && len>0 )
        //   {
        //     cout<<"imu:"<<t.matrix()<<endl;
        //     Eigen::Matrix2d R;
        //     R<<t(0,0),t(0,1),t(1,0),t(1,1); 
            
        //     m.R_vec.push_back(R);

        //     utm ins_car;
        //     ins_car.x=t(0,3);
        //     ins_car.y=t(1,3);
        //     m.tins_vec.push_back(ins_car);
            
        //     Eigen::Matrix4d pc_transform;
        //     pc_transform=m.lidar_odo[m.last].inverse()*lidar_odometry;
        //     Eigen::Matrix2d tlidar;
        //     tlidar<<-pc_transform(0,3),pc_transform(1,3),
        //             -pc_transform(1,3),-pc_transform(0,3);

        //     cout<<"lidar:"<<pc_transform<<endl;
        //     m.lidar_vec.push_back(tlidar);
            
        //     m.last=i;
        //     cout<<"zlast:"<<m.last<<endl;
        //     ++m.iter_z;
        //   }
        //   else{
        //     m.last=i;
        //   }  
                 
        // }
         
        //  if(INSPVA->azimuth()-m.angle.back()<-10 && m.iter_f<4  )
        // { 
           
        //   m.angle.push_back(INSPVA->azimuth());          
        //   Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
          
        //   t=m.imu_odo[m.last].inverse()*imu_odometry;
        //   //cout<<"qq:"<<abs(t(1,3))<<endl;
        //   double len=abs(t(1,3));
        //   cout<<"逆："<<abs(t(1,3))<<endl;
        //   if(len<1.5 && len>0 )
        //   { 
        //     // cout<<"qqqqq:"<<len<<endl;
        //   cout<<"imu:"<<t.matrix()<<endl;
        //   Eigen::Matrix2d R;
        //   R<<t(0,0),t(0,1),t(1,0),t(1,1);
        //   // cout<<"r:"<<R<<endl;
            
        //   m.R_vec.push_back(R);

        //   utm ins_car;
        //   ins_car.x=t(0,3);
        //   ins_car.y=t(1,3);
        //   m.tins_vec.push_back(ins_car);
          
        //   Eigen::Matrix4d pc_transform;
        //   pc_transform=m.lidar_odo[m.last].inverse()*lidar_odometry;
        //   Eigen::Matrix2d tlidar;
        //   tlidar<<-pc_transform(0,3),pc_transform(1,3),
        //           -pc_transform(1,3),-pc_transform(0,3);
        //   cout<<"lidar:"<<pc_transform<<endl;
        //   m.lidar_vec.push_back(tlidar);
          
        //   m.last=i;
        //   cout<<"flast:"<<m.last<<endl;
        //   ++m.iter_f;          
        //   } 
        //   else
        //   {
        //     m.last=i;
        //   }         
        // }
        //   // cout<<"hi!378"<<endl;
        // if(m.iter_z==4 && m.iter_f==4)
        // {
          
        //   m.lidar_height=1.5;
        //   m.result_init=m.sent_x_y_angle(m.R_vec,m.lidar_vec,m.tins_vec,m.lidar_height);
        //   cout<<"result_init="<<m.result_init<<endl;
        //   m.init_calib=true;

        //   { 
        //     write_result write;
        //     double time=floor(point_cloud_first->measurement_time());
        //     string as="velodyne16_back";
        //     write.save_extrinsics(as,time,m.result_init);

        //     // pcl::PointCloud<pcl::PointXYZ>::Ptr back(new pcl::PointCloud<pcl::PointXYZ>);
        //     // Eigen::Matrix4d inv_result;
        //     // // inv_result= m.result_init.inverse();
        //     // inv_result= m.result_init.matrix();
        //     // pcl::transformPointCloud( *lidar, *back, inv_result);        
        //     // pcl::io::savePLYFile ("back.ply", *back);
        //    }

        // }


      //保存数据**************************************
     
      // if(m.lidar_odo.size()==300)
      // {
      //   ofstream rt_file_lidar,rt_file_imu;
        
      //   rt_file_lidar.open("lidar.txt");
      //   rt_file_imu.open("imu.txt");
      //   cout<<"2"<<endl;
      //   for (int i=0;i<300;i++)
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

           //真值校验
           if(use_baidu_result)
           {
             Eigen::Isometry3d T_ure_r = Eigen::Isometry3d::Identity();       
             Eigen::Quaterniond q(0.6966765149185651,0.001694144340871962,-0.003914666312324782,0.7173727335378295);
             
             cout<<"四元数："<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<endl;
            Eigen::AngleAxisd angle1(q);         
            T_ure_r = angle1;

            T_ure_r(0,3)=0.05217693096888097;
            T_ure_r(1,3)=0.1037075200687824;
            T_ure_r(2,3)=1.5; 
            
            m.result_init=T_ure_r.matrix();
           }
               

          cout<<"m_back计算真值为："<<m.result_init.matrix() << endl;

          m.iter=0;
        }
        else{
          m.angle.push_back(INSPVA->azimuth());
          m.T_list.push_back(T);
          *data_back=*lidar;
          int i=m.T_list.size(); 

          m.iter++;               
        
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

        

        if(m.iter<=20)
        {
          pcl::PointCloud<pcl::PointXYZ>::Ptr amp_(new pcl::PointCloud<pcl::PointXYZ>),
           data_back_(new pcl::PointCloud<pcl::PointXYZ>) ;
          m.filter_ground(amp,amp_);
          m.filter_ground(data_back,data_back_);
          // pcl::io::savePLYFile ("turn_lidar.ply", *amp_);
          // pcl::io::savePLYFile ("back_lidar.ply", *data_back_);
          
          
          Eigen::Affine3d deta_trans;         
          pcTransEstimator es;
          es.estimate(*amp_,*data_back_,deta_trans);

          m.deta_trans.push_back(deta_trans);
          if(m.iter==20)
          {
            cout<<"start compute the score*******"<<endl;
            double score=10;
            m.compute_score(m.deta_trans,score);
            cout<<"score:"<<score<<endl;
          }         

        }

        cout<<m.iter<<"帧点云叠加"<<endl;
        if(m.iter==20)
        {
          pcl::io::savePLYFile ("final_back.ply", *tmp_back);
          cout<<"save the final back.ply***"<<endl<<endl;
          m.init_calib=false;          
        }
        
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
      //＊＊＊＊不合理＊＊＊＊
      // pcl::io::savePLYFile ("left_before.ply", *data_befor_left);
      // pcl::io::savePLYFile ("left_now.ply", *data_left);
      // //验证配准结果
      // pcl::PointCloud<pcl::PointXYZ>::Ptr left_re(new pcl::PointCloud<pcl::PointXYZ>);
      // Eigen::Matrix4d inv_result;      
      // inv_result= t_lidar.matrix();
      // pcl::transformPointCloud( *data_left, *left_re, inv_result);
      // *left_re+=*data_befor_left;
      // pcl::io::savePLYFile ("left_re.ply", *left_re);     
      

      *data_befor_left=*lidar;


      Eigen::Matrix4d lidar_odometry,t_lidar_matrix;
      t_lidar_matrix=t_lidar.matrix();     
      lidar_odometry=m_l.lidar_odo.back()*t_lidar_matrix;
      m_l.lidar_odo.push_back(lidar_odometry);
      
      
           cout<<"left hi!"<<endl;

           if(i-m_l.last>20)//防止点云的序列之后
          {
            cout<<"更新"<<endl;
            m_l.last=i;
            m_l.angle.push_back(INSPVA->azimuth());
          } 

        if(INSPVA->azimuth()-m_l.angle.back()>10 && m_l.iter_z<4  )
        {
          
          m_l.angle.push_back(INSPVA->azimuth());
          Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
          
          t=m_l.imu_odo[m_l.last].inverse()*imu_odometry;
          if(abs(t(1,3))<1.5 && abs(t(1,3))>0 )
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
            cout<<"imu:"<<t.matrix()<<endl;
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
            string as="velodyne16_left_init";
            write.save_extrinsics(as,time,m_l.result_init);

            // pcl::PointCloud<pcl::PointXYZ>::Ptr left(new pcl::PointCloud<pcl::PointXYZ>);
            // Eigen::Matrix4d inv_result;
            // // inv_result= m_l.result_init.inverse();
            // inv_result= m_l.result_init.matrix();
            // pcl::transformPointCloud( *lidar, *left, inv_result);        
            // pcl::io::savePLYFile ("left.ply", *left);
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

          if(use_baidu_result)
           {
             Eigen::Isometry3d T_ure_r = Eigen::Isometry3d::Identity();       
             Eigen::Quaterniond q(0.3542123419205854,-0.002056798061661806,-0.0112400366003774,0.935095207981567);
    
            cout<<"四元数："<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<endl;
            Eigen::AngleAxisd angle1(q);         
            T_ure_r = angle1;

            T_ure_r(0,3)=-0.4635003708038772;
            T_ure_r(1,3)=1.370689531427366;
            T_ure_r(2,3)=0.4604696255806184; 
            
            m_l.result_init=T_ure_r.matrix();
           }

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

        if(m_l.iter<=20)
        {
          pcl::PointCloud<pcl::PointXYZ>::Ptr amp_(new pcl::PointCloud<pcl::PointXYZ>),
           data_left_(new pcl::PointCloud<pcl::PointXYZ>) ;
          m_l.filter_ground(amp,amp_);
          m_l.filter_ground(data_left,data_left_);
          // pcl::io::savePLYFile ("turn_lidar.ply", *amp_);
          // pcl::io::savePLYFile ("back_lidar.ply", *data_back_);
          
          
          Eigen::Affine3d deta_trans;         
          pcTransEstimator es;
          es.estimate(*amp_,*data_left_,deta_trans);

          cout<<"1105"<<endl;
          m_l.deta_trans.push_back(deta_trans);
          if(m_l.iter==20)
          {
            cout<<"start compute the score*******"<<endl;
            double score=10;
            m_l.compute_score(m_l.deta_trans,score);
            cout<<"score:"<<score<<endl;
          }         

        }
        
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


           if(i-m_r.last>20)//防止点云的序列之后
          {
            cout<<"更新"<<endl;
            m_r.last=i;
            m_r.angle.push_back(INSPVA->azimuth());
          } 

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

           {
            write_result write;
            double time=floor(point_cloud_first->measurement_time());
            string as="velodyne16_right_init";
            write.save_extrinsics(as,time,m_r.result_init);

            // pcl::PointCloud<pcl::PointXYZ>::Ptr right(new pcl::PointCloud<pcl::PointXYZ>);
            // Eigen::Matrix4d inv_result;
            // // inv_result= m_r.result_init.inverse();
            // inv_result= m_r.result_init.matrix();
            // pcl::transformPointCloud( *lidar, *right, inv_result);        
            // pcl::io::savePLYFile ("right.ply", *right);
            
           }
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

           if(use_baidu_result)
           {
             Eigen::Isometry3d T_ure_r = Eigen::Isometry3d::Identity();       
             Eigen::Quaterniond q(0.9294284578596994,0.00977916061250282,-0.01375931530627733,0.3686160481863997);
    
            cout<<"四元数："<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<endl;
            Eigen::AngleAxisd angle1(q);         
            T_ure_r = angle1;

            T_ure_r(0,3)=0.550951383006692;
            T_ure_r(1,3)=1.396304056215646;
            T_ure_r(2,3)=0.465143174296948; 
            
            m_r.result_init=T_ure_r.matrix();
           }

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

        if(m_r.iter<=20)
        {
          pcl::PointCloud<pcl::PointXYZ>::Ptr amp_(new pcl::PointCloud<pcl::PointXYZ>),
           data_right_(new pcl::PointCloud<pcl::PointXYZ>) ;
          m_r.filter_ground(amp,amp_);
          m_r.filter_ground(data_right,data_right_);
          // pcl::io::savePLYFile ("turn_lidar.ply", *amp_);
          // pcl::io::savePLYFile ("back_lidar.ply", *data_back_);
          
          
          Eigen::Affine3d deta_trans;         
          pcTransEstimator es;
          es.estimate(*amp_,*data_right_,deta_trans);

          m_r.deta_trans.push_back(deta_trans);
          if(m_r.iter==20)
          {
            cout<<"start compute the score*******"<<endl;
            double score=10;
            m_r.compute_score(m_r.deta_trans,score);
            cout<<"score:"<<score<<endl;
          }         

        }

        
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

    typedef PointMatcher<float> PM;
	  typedef PM::DataPoints DP;

    const DP ref(DP::load("back.ply"));
	  const DP data(DP::load("left.ply"));

    PM::ICP icp;
    icp.setDefault();
    PM::TransformationParameters T = icp(data, ref);
  

    DP data_out(data);
	  icp.transformations.apply(data_out, T);

    ref.save("test_ref.vtk");
    data.save("test_data_in.vtk");
    data_out.save("test_data_out.vtk");
    cout << "Final transformation:" << endl << T << endl;
    

    Eigen::Affine3d l_b,r_b;    
    pcTransEstimator es;
    es.estimate(*back,*left,l_b);
    // es.estimate(*back,*right,r_b);

    cout<<"l_b:"<<l_b.matrix()<<endl;
    // cout<<"r_b:"<<r_b.matrix()<<endl;

     pcl::PointCloud<pcl::PointXYZRGB>::Ptr left_t(new pcl::PointCloud<pcl::PointXYZRGB>);
     pcl::transformPointCloud( *left, *left_t, l_b.matrix());

    //  pcl::PointCloud<pcl::PointXYZRGB>::Ptr right_t(new pcl::PointCloud<pcl::PointXYZRGB>);
    //  pcl::transformPointCloud( *right, *right_t, r_b.matrix()); 
    
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr add(new pcl::PointCloud<pcl::PointXYZRGB>); 
    // *add+=*left_t;
    // *add+=*right_t;
    *add=*left;
    *add+=*right;
    *add+=*back;
    writer.write<pcl::PointXYZRGB>("cal_data/add_final.ply", *add, false);
    writer.write<pcl::PointXYZRGB>("pc_left.ply", *left_t, false);

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