#include <stdint.h>
#include <iostream>
#include <fstream>
#include <string>

#include <yaml-cpp/yaml.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include "pointmatcher/PointMatcher.h"

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "calib_pointcloud.h"
#include "utils.h"
#include "mutil_cal_eval.h"
#include "cal_icp.h"
#include "Kabsch.hpp"

#define N_REPEAT 10

bool b_save_plane = false;

void ReadArg(CalibPointCloud &cpc, char *argv, std::string &str_cfg_fold)
{
	std::string fn(argv);
	YAML::Node config = YAML::LoadFile(fn);
// 	std::cout << "Begin reading args" << std::endl;
	
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
	// std::cout << "coef" << std::endl;
	
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
	// std::cout << "normal_plane" << std::endl;
	
	// TODO: please comment it 
	if(!config["cloud"] || !config["cloud"].IsSequence())
	{
		std::cerr << "E: read yaml but no node(pointclouds)" << std::endl;
		return;
	} else
	{
		auto pcs = config["cloud"];
		pcl::PointCloud<pcl::PointXYZI> pc;
		if (pcl::io::loadPCDFile(pcs[0].as<std::string>(), pc) == -1)
		{
			std::cout << "E: fail to read pcd file" << std::endl
					<< pcs[0].as<std::string>() << std::endl;
		} else
		{
			std::cout << pcs[0].as<std::string>() << std::endl;
			cpc.cloud_ = pc;
		}
	}
	// std::cout << "cloud" << std::endl;
	
	if (!config["planes"] || !config["planes"].IsSequence())
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
	// std::cout << "planes" << std::endl;
	
	Eigen::Vector3f o;
	for(uint32_t i=0; i<3; i++ )
	{
		o(i) = config["origin"][0][i].as<float>();
	}
	cpc.origin_ = o;
	std::cout << "origin" << std::endl;
	
	if(!config["cfg_fold"] || !config["cfg_fold"].IsSequence())
	{
		std::cerr << "E: read yaml but no cfg_fold" << std::endl;
		return;
	} else
	{
		str_cfg_fold = config["cfg_fold"][0].as<std::string>();
	}
	// std::cout << "cfg_fold" << std::endl;
	
	if (config["tf_gt"])
	{
		Eigen::Matrix4f tf;
		tf.setIdentity();
		for(uint32_t i=0; i<4; i++ )
		{
			for(uint32_t j=0; j<4; j++)
			{
				tf(i,j) = config["tf_gt"][i][j].as<float>();
			}
		}
		cpc.tf_gt_ = tf;	
	} else
	{
		cpc.tf_gt_.setIdentity();
	}	

	if (config["tf_ini"])
	{
		Eigen::Matrix4f tf;
		tf.setIdentity();
		for(uint32_t i=0; i<4; i++ )
		{
			for(uint32_t j=0; j<4; j++)
			{
				tf(i,j) = config["tf_ini"][i][j].as<float>();
			}
		}
		cpc.tf_ini_ = tf;	
	} else
	{
		cpc.tf_ini_.setIdentity();
	}	
}

void EvaluExtrinsic(const Eigen::Matrix4d &tf_base, const Eigen::Matrix4d &tf_measured, double &r_error, double &t_error)
{
	Eigen::Matrix3d R_delta = tf_base.topLeftCorner(3, 3) * tf_measured.topLeftCorner(3, 3).inverse();
	Eigen::AngleAxisd r_delta(R_delta);
	Eigen::Vector3d pi_delta = r_delta.angle() * r_delta.axis();
	r_error = pi_delta.lpNorm<2>();
	
	Eigen::Vector3d t_delta = tf_base.topRightCorner(3, 1) - tf_measured.topRightCorner(3, 1); 
	t_error = t_delta.lpNorm<2>();
}

void SavePlaneCallBack(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("%s", msg->data.c_str());
	b_save_plane = true;
}


int main(int argc, char *argv[])
{
	if(argc < 4)
	{
		std::cerr << "E: no enough argument(s)" << std::endl
				  << "Usage: " << argv[0] << " initialize ref_cfg.yaml, data_cfg.yaml i/m" << std::endl;
		return 1;
	}
	
	ros::init(argc, argv, "calib_icp");
	ros::NodeHandle nh("calib_icp");
	ros::Rate loop_rate(1);
	
	ros::Subscriber sub_plane = nh.subscribe("/contact/save_plane", 1000, SavePlaneCallBack);
	ros::Publisher pub_icp = nh.advertise<std_msgs::String>("/contact/icp", 1000);
	
	std::vector<double> v_r_error, v_t_error;
	std::string str_cfg_fold;
	int n_repeat = 0;
	while ((n_repeat < N_REPEAT) && (ros::ok()))
	{
		ros::spinOnce();
		loop_rate.sleep();
		if (!b_save_plane) continue;
		std::cout << "[TESTING ........] " << n_repeat << std::endl;
		CalibPointCloud calib_cloud_ref, calib_cloud_data;	
		
		ReadArg(calib_cloud_ref, argv[1], str_cfg_fold);
		ReadArg(calib_cloud_data, argv[2], str_cfg_fold);

		Eigen::Matrix3d R;
		Eigen::Vector3d t;
		if (std::string(argv[3]) == "a")		 
		{
			std::cout << "Auto initialization ****************" << std::endl;
			Eigen::Matrix3d P;
			P.col(0) = calib_cloud_ref.v_normal_[0].cast<double>() + calib_cloud_ref.origin_.cast<double>();
			P.col(1) = calib_cloud_ref.v_normal_[1].cast<double>() + calib_cloud_ref.origin_.cast<double>();
			P.col(2) = calib_cloud_ref.v_normal_[2].cast<double>() + calib_cloud_ref.origin_.cast<double>();
			
			Eigen::Matrix3d Q;
			Q.col(0) = calib_cloud_data.v_normal_[0].cast<double>() + calib_cloud_data.origin_.cast<double>();
			Q.col(1) = calib_cloud_data.v_normal_[1].cast<double>() + calib_cloud_data.origin_.cast<double>();
			Q.col(2) = calib_cloud_data.v_normal_[2].cast<double>() + calib_cloud_data.origin_.cast<double>();
			
			Eigen::Affine3d A = Kabsch::Find3DAffineTransform(Q, P, false);
			R = A.linear();
			t = A.translation();
		}
		else if (std::string(argv[3]) == "m")		 
		{
			std::cout << "Manual initialization ****************" << std::endl;
			Eigen::Matrix4d T = calib_cloud_data.tf_ini_.cast<double>();
			R = T.topLeftCorner(3, 3);
			t = T.topRightCorner(3, 1);		
		}
		std::cout << "Initial rotation: " << std::endl << R << std::endl 
			<< "Initial translation: " << t.transpose() << std::endl;

		// std::cout << calib_cloud_ref.v_plane_[0].size() << std::endl;

		// 	TODO: minimize point-to-plane error
		std::cout << std::endl << "Refinement ****************" << std::endl;
		MutilCalEval mce;
		mce.add_pointcloud(calib_cloud_ref);
		mce.add_pointcloud(calib_cloud_data);
		// std::cout << "Plane base average error: " << mce.get_base_error() << std::endl;

		std::vector<Eigen::Affine3d> eps, eps_icp, eps_op;
		eps.resize(2);
		eps[0].setIdentity();
		eps[1].setIdentity();

		mce.set_parameters(eps);
		std::string str_merge_before = str_cfg_fold + "merged_before.pcd";
		mce.save_merged_pointcloud(str_merge_before, eps);

		eps[1].linear() = R;
		eps[1].translation() = t;
		
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

		/**
		*	Optimization ERROR
		*  Output format: iter| cost| cost_change| gradient| |step |tr_ratio |tr_radius |ls_iter |iter_time |total_time
		*/
		std::cout << std::endl << "Nonlinear refinement ****************" << std::endl;
		std::cout << "ID Base: " << 0 << std::endl << std::endl;

		ros::Time t_begin_optimization = ros::Time::now();
		double mse = mce.calibrate(0, true);
		ros::Time t_end_optimization = ros::Time::now();
		ros::Duration t_optimization = t_end_optimization - t_begin_optimization;
		std::cout << "Nonlinear refinement time: " << t_optimization.toSec() << "s" << std::endl;

		std::cout << "Plane-to-Plane MSE: " << mse << std::endl;
		// save pcd
		std::string str_merge_opt = str_cfg_fold + "merged_opt.pcd";
		mce.save_merged_pointcloud(str_merge_opt);	
		// save eps
		std::string str_eps_opt = str_cfg_fold + "eps_opt.csv";
		mce.get_parameters(eps_op);
		save_eps(str_eps_opt, eps_op);
		std::cout << "OPT Error: " << mce.evaluate() << std::endl;	
		// save optimization report
		mce.save_opt_report(str_cfg_fold + "opt_report.yaml");
		std::cout << std::endl << "END----------" << std::endl;
		
		std::cout << "[GT] :" << std::endl;
		std::cout << calib_cloud_ref.tf_gt_ << std::endl;
		std::cout << calib_cloud_data.tf_gt_ << std::endl;
		
		// TODO: iv2019_evaluation
		double r_error, t_error;
		EvaluExtrinsic(calib_cloud_data.tf_gt_.cast<double>(), eps_op[1].matrix(), r_error, t_error);
		v_r_error.push_back(r_error);
		v_t_error.push_back(t_error);
		
		std::cout << "r_error: " << r_error << " t_error: " << t_error << std::endl;
		
		n_repeat ++;
		b_save_plane = false;
		std_msgs::String msg;
		std::stringstream ss;
		ss << "icp ...";
		msg.data = ss.str();
		pub_icp.publish(msg);
		
	}
	
	// TODO: save evaluation result
	std::string str_file = str_cfg_fold + "simulation_result.yaml";
	std::ofstream fn_evaluation_report(str_file);
	double mean, sq_sum, sigma;
	
	mean = std::accumulate(v_r_error.begin(), v_r_error.end(), 0.0) / v_r_error.size();
	fn_evaluation_report << "rotation mean:" << std::endl;
	fn_evaluation_report << mean << std::endl;
	sq_sum = std::inner_product(v_r_error.begin(), v_r_error.end(), v_r_error.begin(), 0.0);
	fn_evaluation_report << "rotation std:" << std::endl;
	sigma = sq_sum / v_t_error.size() - mean * mean;
	if (sigma < 0.000001) sigma = 0;
	fn_evaluation_report << sqrt(sigma) << std::endl;
// 	std::cout << sq_sum << std::endl;
	std::cout << mean << " " << sqrt(sigma) << std::endl;
	
	mean = std::accumulate(v_t_error.begin(), v_t_error.end(), 0.0) / v_t_error.size();
	fn_evaluation_report << "translation mean:" << std::endl;
	fn_evaluation_report << mean << std::endl;;
	sq_sum = std::inner_product(v_t_error.begin(), v_t_error.end(), v_t_error.begin(), 0.0);
	fn_evaluation_report << "translation std:" << std::endl;
	sigma = sq_sum / v_t_error.size() - mean * mean;
	if (sigma < 0.000001) sigma = 0;
	fn_evaluation_report << sqrt(sigma) << std::endl;;
// 	std::cout << sq_sum << std::endl;
	std::cout << mean << " " << sqrt(sigma) << std::endl;
	
	fn_evaluation_report << "rotation error:" << std::endl;
	for (size_t i = 0; i < v_r_error.size(); i++)
	{
		fn_evaluation_report << v_r_error[i] << std::endl;
	}
	fn_evaluation_report << "translation error:" << std::endl;
	for (size_t i = 0; i < v_t_error.size(); i++)
	{
		fn_evaluation_report << v_t_error[i] << std::endl;
	}	
	fn_evaluation_report.close();
	
	
// 	for (size_t i = 0; i < v_r_error.size(); i++)
// 	{
// 		fn_evaluation_report << v_r_error[i] << std::endl;
// 	}
// 	for (size_t i = 0; i < v_t_error.size(); i++)
// 	{
// 		fn_evaluation_report << v_t_error[i] << std::endl;
// 	}	
	
	return 0;
}






