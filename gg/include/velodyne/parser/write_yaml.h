#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <limits>
#include <yaml-cpp/yaml.h>
#include <memory>
#include <map>
#include "pcl-1.7/pcl/io/pcd_io.h"
#include "pcl-1.7/pcl/io/ply_io.h"

using namespace std;

namespace YAML {
template <typename T>
void operator>>(const YAML::Node& node, T& i) {
  i = node.as<T>();
}
}

class write_result
{
    
 public:   
    void save_extrinsics(std::string path,double cal_time,Eigen::Matrix4d& T);
    void write_yaml(std::string ply_name ,double cal_time,std::vector <double>& rt,std::map <int,std::string>& txt);
};

// class write_result
// {
    
//  public:   
//     void save_extrinsics(std::string path,Eigen::Matrix4d& T){

//         string ply_name=path+"_novatel_extrinsics.yaml";
//         Eigen::Vector3d t;
//         Eigen::Quaterniond q;
        
//         t={T(0,3),T(1,3),T(2,3)};
//         Eigen::Matrix3d transform_matrix=T.block(0,0,3,3);

//         q=transform_matrix;
//         std::vector <double> rt_para;
//         rt_para.push_back(q.x());
//         rt_para.push_back(q.y());
//         rt_para.push_back(q.z());
//         rt_para.push_back(q.w());

//         rt_para.push_back(t(0));
//         rt_para.push_back(t(1));
//         rt_para.push_back(t(2));

//         std::map <int,std::string> txt;
//         txt[0] = "novatel";
//         txt[1] = path;
//         // cout<<"t:"<<t<<endl;
//         // cout<<"四元数："<<q.coeffs()<<endl;
//         write_yaml(ply_name,rt_para,txt);

//     }
   
//     void write_yaml(std::string ply_name ,std::vector <double>& rt,std::map <int,std::string>& txt)
//     {
//         std::ofstream fout(ply_name);          
//         YAML::Emitter out;
//         // out << "Hello, World! velodyne16_back_novatel_extrinsics";   
//         // std::cout << "Here's the output YAML:\n" << out.c_str(); // prints "Hello, World!"

//         out << YAML::BeginMap;

//         out << YAML::Key << "header";
//         out << YAML::Value <<YAML::BeginMap << "seq"<< YAML::Value <<0;

//         out << YAML::Key << "stamp" << YAML::Value <<YAML::BeginMap;
//         out<< YAML::Key << "secs" << YAML::Value << 1556434664;
//         out<< YAML::Key << "nsecs" << YAML::Value << 0<<YAML::EndMap;
//         out<< YAML::Key << "frame_id" << YAML::Value <<txt[0]<<YAML::EndMap;
     
//         out << YAML::Key << "transform";
//         out << YAML::Value <<YAML::BeginMap << "rotation"<< YAML::Value <<YAML::BeginMap;

//         out<< YAML::Key << "x" << YAML::Value <<rt[0];
//         out<< YAML::Key << "y" << YAML::Value <<rt[1];
//         out<< YAML::Key << "z" << YAML::Value <<rt[2];
//         out<< YAML::Key << "w" << YAML::Value <<rt[3]<<YAML::EndMap;

//         out<< "translation"<< YAML::Value <<YAML::BeginMap;
//         out<< YAML::Key << "z" << YAML::Value <<rt[6];
//         out<< YAML::Key << "x" << YAML::Value <<rt[4];
//         out<< YAML::Key << "y" << YAML::Value <<rt[5]<<YAML::EndMap<<YAML::EndMap;
        

//         // out << YAML::Key << "header";
//         // out << YAML::Value <<YAML::BeginSeq << "rotation"<<YAML::BeginSeq <<"x";


//         out << YAML::Key << "child_frame_id";
//         out << YAML::Value <<txt[1];
        
//         out << YAML::EndMap;
//         fout << out.c_str();
//         fout.close();
//     }

// };