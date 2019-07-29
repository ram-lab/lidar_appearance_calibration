#include <stdint.h>
#include <iostream>
#include <fstream>
#include <string>

#include <yaml-cpp/yaml.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include "pointmatcher/PointMatcher.h"

#include "calib_pointcloud.h"
#include "utils.h"
#include "mutil_cal_eval.h"
#include "cal_icp.h"
#include "Kabsch.hpp"

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

void ReadArg(CalibPointCloud &cpc, char *argv, std::string &str_cfg_fold, std::string &str_cloud)
{
	std::string fn(argv);
	YAML::Node config = YAML::LoadFile(fn);
	std::cout << "Begin reading args" << std::endl;
	
	for (size_t i = 0; i < 3; i++)
	{
		Eigen::Vector4f coeff;
		for (size_t j = 0; j < 4; j++)
		{
			float x = config["coef_plane"][i][j].as<float>();
			coeff(j) = x;
		}
		cpc.v_coeff_.push_back(coeff);
	}
	
	for (size_t i = 0; i < 3; i++)
	{
		Eigen::Vector3f normal;
		for (size_t j = 0; j < 3; j++)
		{
			float x = config["normal_plane"][i][j].as<float>();
			normal(j) = x;
		}
		cpc.v_normal_.push_back(normal);
	}
	
	if(!config["cloud"] || !config["cloud"].IsSequence())
	{
		std::cerr << "E: read yaml but no node(pointclouds)" << std::endl;
		return;
	} else
	{
		auto pcs = config["cloud"];
		for(size_t i = 0; i < pcs.size(); i++)
		{
			pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>);
			if (pcl::io::loadPCDFile(pcs[i].as<std::string>(), *pc) == -1)
			{
				std::cout << "E: fail to read pcd file" << std::endl
						<< pcs[i].as<std::string>() << std::endl;
			} else
			{
				cpc.cloud_ = *pc;
			}
		}
	}
	
	if(!config["planes"] || !config["planes"].IsSequence())
	{
		std::cerr << "E: read yaml but no node(pointclouds)" << std::endl;
		return;
	} else
	{
		auto pcs = config["planes"];
		for(size_t i = 0; i < pcs.size(); i++)
		{
			pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>);
			if (pcl::io::loadPCDFile(pcs[i].as<std::string>(), *pc) == -1)
			{
				std::cout << "E: fail to read pcd file" << std::endl
						<< pcs[i].as<std::string>() << std::endl;
			} else
			{
				cpc.v_plane_.push_back(*pc);
			}
		}
	}
	
// 	Eigen::Matrix4f tf;
// 	tf.setIdentity();
// 	for(uint32_t i=0; i<4; i++ )
// 	{
// 		for(uint32_t j=0; j<4; j++)
// 		{
// 			tf(i,j) = config["tf"][i][j].as<float>();
// 		}
// 	}
// 	cpc.tf_ini_ = tf;	
	
	Eigen::Vector3f o;
	for(uint32_t i=0; i<3; i++ )
	{
		o(i) = config["origin"][0][i].as<float>();
	}
	cpc.origin_ = o;
	
	if(!config["cfg_fold"] || !config["cfg_fold"].IsSequence())
	{
		std::cerr << "E: read yaml but no cfg_fold" << std::endl;
		return;
	} else
	{
		str_cfg_fold = config["cfg_fold"][0].as<std::string>();
	}	

	if(!config["cloud"] || !config["cloud"].IsSequence())
	{
		std::cerr << "E: read yaml but no cloud" << std::endl;
		return;
	} else
	{
		str_cloud = config["cloud"][0].as<std::string>();
	}	
	std::cout << "Finish reading args" << std::endl;	
}

double PointPlaneError(const Eigen::Vector4f &coeff, const pcl::PointCloud<pcl::PointXYZI> &pc, const Eigen::Matrix4f &tf)
{
	pcl::PointCloud<pcl::PointXYZI> pc_transformed;
	pcl::transformPointCloud(pc, pc_transformed, tf);
	double point_plane_error = 0;
	for (size_t i = 0; i < pc_transformed.points.size(); i++)
	{
// 		double error = fabs(coeff[0]*pc_transformed.points[i].x + coeff[1]*pc_transformed.points[i].y + coeff[2]*pc_transformed.points[i].z + coeff[3]);
		double error = pow(coeff[0]*pc_transformed.points[i].x + coeff[1]*pc_transformed.points[i].y + coeff[2]*pc_transformed.points[i].z + coeff[3], 
						   2.0);	
	
		point_plane_error += error;
	}
	return point_plane_error;
}

int main(int argc, char *argv[])
{
	if(argc < 4)
	{
		std::cerr << "E: no enough argument(s)" << std::endl
				  << "Usage: " << argv[0] << " ref_cfg.yaml, data_cfg.yaml icp_cfg.yaml" << std::endl;
		return 1;
	}

	CalibPointCloud calib_cloud_ref, calib_cloud_data;	
	
	std::string str_cfg_fold, str_ref_cloud, str_data_cloud;	
	ReadArg(calib_cloud_ref, argv[1], str_cfg_fold, str_ref_cloud);
	ReadArg(calib_cloud_data, argv[2], str_cfg_fold, str_data_cloud);
	
// 	std::cout << calib_cloud_ref;
// 	std::cout << calib_cloud_data;
	
    MutilCalEval mce;
	mce.add_pointcloud(calib_cloud_ref);
	mce.add_pointcloud(calib_cloud_data);
	
	std::cout << "Base error: " << mce.get_base_error() << std::endl;
	
	Eigen::Matrix4f tf;
	tf.setIdentity();
	
	double sum_error;
	
	std::vector<float> w_ref, w_data;
	int num_point_ref, num_point_data, num_point;
	num_point_ref = calib_cloud_ref.v_plane_[0].size() + calib_cloud_ref.v_plane_[1].size() + calib_cloud_ref.v_plane_[2].size();
	num_point_data = calib_cloud_data.v_plane_[0].size() + calib_cloud_data.v_plane_[1].size() + calib_cloud_data.v_plane_[2].size();
	for (size_t i = 0; i < calib_cloud_ref.v_plane_.size(); i++)
	{
// 		w_ref.push_back(1 - (1.0*calib_cloud_ref.v_plane_[i].size()/num_point_ref) );
// 		w_data.push_back(1 - (1.0*calib_cloud_data.v_plane_[i].size()/num_point_data) );
		w_ref.push_back(1.0);
		w_data.push_back(1.0);
	}
	num_point = num_point_ref + num_point_data;
	
// TODO: W/O refinement
	tf << -0.999736, -0.00994325, 0.0207151, -1.92115
		,0.00883716, -0.998565, -0.052819, 0.0968707
		,0.0212106, -0.052622, 0.998389, -0.80199
		,0 ,0 ,0 ,1;

	sum_error = 0;
	for (size_t i = 0; i < calib_cloud_ref.v_coeff_.size(); i++)
	{
		Eigen::Vector4f coeff; 
		pcl::PointCloud<pcl::PointXYZI> plane;
		
		coeff = calib_cloud_ref.v_coeff_[i];
		plane = calib_cloud_data.v_plane_[i];
		sum_error += PointPlaneError(coeff, plane, tf) * w_data[i];
		
		coeff = calib_cloud_data.v_coeff_[i];
		plane = calib_cloud_ref.v_plane_[i];
		sum_error += PointPlaneError(coeff, plane, tf.inverse()) * w_ref[i];
	}
	std::cout << "W/O refinement: [NUM, ERROR, ERROR/NUM] " << num_point << " " << sum_error << " " << sum_error / num_point << std::endl;			
	
// TODO: proposed
	tf << -0.999991 ,-0.0037521 ,0.00200477 ,-1.91 
		,0.00375282 ,-0.999993 ,0.000355548 ,0.05299
		,0.00200343 ,0.000363068 ,0.999998 ,-1.0744
		,0 ,0 ,0 ,1;
		
	sum_error = 0;
	for (size_t i = 0; i < calib_cloud_ref.v_coeff_.size(); i++)
	{
		Eigen::Vector4f coeff; 
		pcl::PointCloud<pcl::PointXYZI> plane;
		
		coeff = calib_cloud_ref.v_coeff_[i];
		plane = calib_cloud_data.v_plane_[i];
		sum_error += PointPlaneError(coeff, plane, tf) * w_data[i];
		
		coeff = calib_cloud_data.v_coeff_[i];
		plane = calib_cloud_ref.v_plane_[i];
		sum_error += PointPlaneError(coeff, plane, tf.inverse()) * w_ref[i];
	}
	std::cout << "Proposed: [NUM, ERROR, ERROR/NUM] " << num_point << " " << sum_error << " " << sum_error / num_point << std::endl;	
		
// TODO: handlabeling
	tf << -0.9998, -0.0027, 0.0192, -1.96443 
		,0.0024, -0.9999, -0.0161, -0.0192
		,0.0192, -0.0161, 0.9997, -1.13756
		,0 ,0 ,0 ,1;		
		
	sum_error = 0;
	for (size_t i = 0; i < calib_cloud_ref.v_coeff_.size(); i++)
	{
		Eigen::Vector4f coeff; 
		pcl::PointCloud<pcl::PointXYZI> plane;
		
		coeff = calib_cloud_ref.v_coeff_[i];
		plane = calib_cloud_data.v_plane_[i];
		sum_error += PointPlaneError(coeff, plane, tf) * w_data[i];
		
		coeff = calib_cloud_data.v_coeff_[i];
		plane = calib_cloud_ref.v_plane_[i];
		sum_error += PointPlaneError(coeff, plane, tf.inverse()) * w_ref[i];
	}
	std::cout << "HandLabelling: [NUM, ERROR, ERROR/NUM] " << num_point << " " << sum_error << " " << sum_error / num_point << std::endl;			
	
// TODO: icp refinement
	tf << -0.99108, 0.00944037,  -0.132932, -1.95929
		,-0.013735,  -0.999412,  0.0314269, 0.0473068
		,-0.132557,  0.0329725,   0.990626, -0.383619
		,0 ,0 ,0 ,1;

	sum_error = 0;
	for (size_t i = 0; i < calib_cloud_ref.v_coeff_.size(); i++)
	{
		Eigen::Vector4f coeff; 
		pcl::PointCloud<pcl::PointXYZI> plane;
		
		coeff = calib_cloud_ref.v_coeff_[i];
		plane = calib_cloud_data.v_plane_[i];
		sum_error += PointPlaneError(coeff, plane, tf) * w_data[i];
		
		coeff = calib_cloud_data.v_coeff_[i];
		plane = calib_cloud_ref.v_plane_[i];
		sum_error += PointPlaneError(coeff, plane, tf.inverse()) * w_ref[i];
	}
	std::cout << "ICP refinement: [NUM, ERROR, ERROR/NUM] " << num_point << " " << sum_error << " " << sum_error / num_point << std::endl;			
		
	return 0;
}






