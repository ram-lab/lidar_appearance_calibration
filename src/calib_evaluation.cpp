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
	
	std::cout << calib_cloud_ref;
	std::cout << calib_cloud_data;
	
	Eigen::Matrix3d P;
	P.col(0) = calib_cloud_ref.v_normal_[0].cast<double>() + calib_cloud_ref.origin_.cast<double>();
	P.col(1) = calib_cloud_ref.v_normal_[1].cast<double>() + calib_cloud_ref.origin_.cast<double>();
	P.col(2) = calib_cloud_ref.v_normal_[2].cast<double>() + calib_cloud_ref.origin_.cast<double>();
	
	Eigen::Matrix3d Q;
	Q.col(0) = calib_cloud_data.v_normal_[0].cast<double>() + calib_cloud_data.origin_.cast<double>();
	Q.col(1) = calib_cloud_data.v_normal_[1].cast<double>() + calib_cloud_data.origin_.cast<double>();
	Q.col(2) = calib_cloud_data.v_normal_[2].cast<double>() + calib_cloud_data.origin_.cast<double>();
	
	std::cout << "P: " << std::endl << P << std::endl;
	std::cout << "Q: " << std::endl << Q << std::endl;
	
	Eigen::Affine3d A = Kabsch::Find3DAffineTransform(Q, P, false);
	Eigen::Matrix3d R = A.linear();
	Eigen::Vector3d t = A.translation();
	
	std::cout << "Rotation: " << std::endl << R << std::endl 
		<< "Translation: " << t.transpose() << std::endl;

// 	TODO: minimize point-to-plane error
    MutilCalEval mce;
	mce.add_pointcloud(calib_cloud_ref);
	mce.add_pointcloud(calib_cloud_data);
	std::cout << "Plane base average error: " << mce.get_base_error() << std::endl;
	
	std::vector<Eigen::Affine3d> eps, eps_op;
	eps.resize(2);
	eps[0].setIdentity();
	eps[1].setIdentity();
	eps[1].linear() = R;
	eps[1].translation() = t;
	std::cout << "Initial tf: " << std::endl 
		<< eps[1].linear().transpose() << std::endl 
		<< eps[1].translation().transpose() << std::endl;
    mce.set_parameters(eps);
	std::string str_merge_ini = str_cfg_fold + "merged_ini.pcd";
	mce.save_merged_pointcloud(str_merge_ini, eps);

	Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
	Eigen::IOFormat CsvFmt(12,0,",");
	auto save_eps = [&OctaveFmt, &CsvFmt](const std::string& fn,
										  const std::vector<Eigen::Affine3d>& eps)
	{
		std::ofstream f(fn);
		if(!f.good())
		{
			std::cerr << "E: Fail to open file: " << fn << std::endl;
			return;
		}

		for(uint32_t i=0; i<eps.size(); i++)
		{
			std::cout << "id: " << i << std::endl
					  << eps[i].matrix().format(OctaveFmt)
					  << std::endl;
			f << eps[i].matrix().format(CsvFmt) << std::endl;
			Eigen::Quaterniond q(eps[i].linear());
			std::cout << "Quaternion: x y z w" << std::endl << q.coeffs().transpose() << std::endl;
		}
	};

	std::cout << str_ref_cloud << std::endl << str_data_cloud << std::endl;
	DP pc0 = DP::load(str_ref_cloud);
	DP pc1 = DP::load(str_data_cloud);
	
	pc0.save("/home/jjiao/catkin_ws/src/lidar_appearance_calibration/data/yellow/top_tail/1.pcd");
	
// 	PM::ICP icp;
// 	std::string fn_config(argv[3]);
// 	std::ifstream ifs(fn_config);
// 	if( ifs.good() )
// 	{
// 		icp.loadFromYaml(ifs);
// 		ifs.close();
// 	}
// 	else
// 	{
// 		icp.setDefault();
// 	}
// 	PM::TransformationParameters T = icp(pc0, pc1, eps[1].matrix().cast<float>() );
// 	std::cout << T << std::endl;
// 	Eigen::Matrix4f tmp = T;
// 	std::vector<Eigen::Affine3d> eps_icp;
// 	eps_icp.resize(2);
// 	eps_icp[0].setIdentity();
// 	eps_icp[1] = Eigen::Affine3d( tmp.cast<double>() );
// 	covar_[id] = (icp.errorMinimizer->getCovariance()).cast<double>();

// 	// save eps
// 	std::string str_eps_icp = str_cfg_fold + "eps_icp.csv";
// 	save_eps(str_eps_icp, eps_icp);
	
	/**
	 *	ICP ERROR
	 */
// 	std::string fn(argv[3]);
// 	CalICP icp(fn);
// 	icp.add_pointcloud(calib_cloud_top.cloud_);
// 	icp.add_pointcloud(calib_cloud_tail.cloud_);
// 	icp.calibrate(eps, 0);
// 	// save pcd
// 	std::string str_merge_icp = str_cfg_fold + "merged_icp.pcd";
// 	mce.save_merged_pointcloud(str_merge_icp, eps);


	return 0;
}






