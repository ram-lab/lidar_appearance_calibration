/*
 * calibration.cpp
 * Copyright (C) 2016 Nick.Liao <simplelife_nick@hotmail.com>
 *
 * Distributed under terms of the MIT license.
 */

/**
 * @file calibration.cpp
 * @brief calibration for 3d lidar and camera
 * @author Nick.Liao <simplelife_nick@hotmail.com>
 * @version 1.0.0
 * @date 2016-08-22
 */

#include "calibration.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <iterator>

#include <pcl/io/vtk_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/common/distances.h>

#include <unsupported/Eigen/MatrixFunctions>

#include <boost/make_shared.hpp>

#include "json.h"


#include "img_trans_estimate.h"
#include "pc_trans_estimate.h"
#include "utils.h"


#define FILE_TF		"/tf.csv"
#define	FILE_PC		"/pc.csv"
#define	FILE_IMG	"/img.csv"
#define	FILE_PAIRS	"/pairs.csv"
#define FOLDER_IMG	"/image/"
#define FOLDER_PC	"/pointcloud/"

const double PI = 3.1415926535897932384626433;



calibration::calibration(const std::string& fp)
{
	std::ifstream f (fp);

	if (!f)
	{
		std::cerr << "[ERROR]\tconfig file doesn't exist" << std::endl;
		pose_num = 0;
		return;
	}

	std::string content{ std::istreambuf_iterator<char>(f),
						 std::istreambuf_iterator<char>()  };
	f.close();

	auto j = nlohmann::json::parse(content);

	//implicit conversions
	//c.is_simulate = j["is_simulate"].get<bool>();
	pose_num 		= j["record_num"];
	pair_factor		= j["pair_factor"];
	path 			= j["path"]["data"];

	is_load_pairs   = j["is_load_pairs"];
	pair_angle_err_threshold_1 = j["pair_angle_err_threshold_1"];
	pair_angle_err_threshold_2 = j["pair_angle_err_threshold_2"];

	img_trans_iterations	 	= j["img_trans_iterations"];
	img_trans_threshold 		= j["img_trans_threshold"];
	img_trans_ba_iterations 	= j["img_trans_ba_iterations"];
	img_trans_ba_verbose 		= j["img_trans_ba_verbose"];

	auto cam = j["intrinsic_fx_fy_s_u_v"];
	K << cam[0], cam[2], cam[3],
	0, cam[1], cam[4],
	0, 0, 1;

	// check config
	if(pose_num < 3)
	{
		pose_num = 3;
	}
	//pose.clear();
	pose.reserve(pose_num);

	if(pair_factor > pose_num)
	{
		pair_factor = pose_num;
	}

	if(pair_factor < 2)
	{
		pair_factor = 2;
	}

	is_loaded_pairs = false;


	EDLineParam para;



	//conf_img.line_dir_err_max = img_j["line_dir_err_max"];

	auto img_j = j["img"];
	auto ed = img_j["edlines"];
	para.minLineLen 		 = ed["minLineLen"];
	para.lineFitErrThreshold = ed["lineFitErrThreshold"];
	para.ksize 				 = ed["ksize"];
	para.sigma 				 = ed["sigma"];
	para.gradientThreshold 	 = ed["gradientThreshold"];
	para.scanIntervals 		 = ed["scanIntervals"];
	para.anchorThreshold 	 = ed["anchorThreshold"];

	edl_detector_img = EDLineDetector (para);

	auto pc_j = j["pc"];
	conf_pc.pcm_config_path  = pc_j["pcm_config_path"];
	conf_pc.size 			 = pc_j["size"];
	conf_pc.exp 			 = pc_j["exp"];
	conf_pc.num_min 		 = pc_j["num_min"];
	conf_pc.dist_min 		 = pc_j["dist_min"];
	conf_pc.square_min 		 = pc_j["square_min"];

	ed = pc_j["edlines"];
	para.minLineLen 		 = ed["minLineLen"];
	para.lineFitErrThreshold = ed["lineFitErrThreshold"];
	para.ksize 				 = ed["ksize"];
	para.sigma 				 = ed["sigma"];
	para.gradientThreshold 	 = ed["gradientThreshold"];
	para.scanIntervals 		 = ed["scanIntervals"];
	para.anchorThreshold 	 = ed["anchorThreshold"];

	edl_detector_pc = EDLineDetector (para);

	auto line_j = j["lines"];
	conf_line.dir_err_max 	= line_j["dir_err_max"];
	conf_line.dist_err_max 	= line_j["dist_err_max"];

}

calibration::~calibration()
{

}

bool calibration::add(const pose_struct& arg)
{
	pose.push_back(arg);
	return true;
}


uint16_t calibration::save( )
{
	pcl::PCLPointCloud2 pc2;

	std::ofstream tf;
	tf.open(path + FILE_TF );

	tf.precision(15);
	tf.setf(std::ios::scientific);

	std::cout << "[INFO]\t---save #" << pose.size() << " record(s) to " << path  << std::endl;

	for (uint64_t i = 0; i < pose.size(); i++ )
	{
		//std::string pc_fn = path + FOLDER_PC + std::to_string(i) + ".vtk";
		std::string pc_fn_pcd =path + FOLDER_PC + std::to_string(i) + ".pcd";
		std::string img_fn = path + FOLDER_IMG + std::to_string(i) + ".png";

		std::cout << "\tsaving #" << i << "th record" << std::endl;



		cv::imwrite(img_fn, *(pose[i].img) );
		pcl::io::savePCDFileBinary(pc_fn_pcd, *pose[i].pc);

		//pcl::toPCLPointCloud2(*pose[i].pc, pc2);
		//pcl::io::saveVTKFile(pc_fn, pc2, 8);


		Eigen::Quaterniond* qp = &pose[i].c2w_q;
		Eigen::Translation3d* tp = &pose[i].c2w_t;
		tf<<std::scientific;
		tf << qp->w() << ", " << qp->x() << ", " << qp->y() << ", " << qp->z() << ",";
		tf << tp->x() << ", " << tp->y() << ", " << tp->z() << ",";;

		qp = &pose[i].c2l_q;
		tp = &pose[i].c2l_t;
		tf << qp->w() << ", " << qp->x() << ", " << qp->y() << ", " << qp->z() << ",";
		tf << tp->x() << ", " << tp->y() << ", " << tp->z() << ",";

		qp = &pose[i].l2w_q;
		tp = &pose[i].l2w_t;
		tf << qp->w() << ", " << qp->x() << ", " << qp->y() << ", " << qp->z() << ",";;
		tf << tp->x() << ", " << tp->y() << ", " << tp->z() << std::endl;

	}

	tf.close();

	return pose.size();
}

int32_t calibration::load()
{
	std::ifstream tfs(path + FILE_TF);

	if (!tfs)
	{
		std::cerr << "ERROR: The path does not include the record file" << std::endl;
		return -1;
	}

	pose_struct ele;
	std::string line;
	uint64_t cnt = 0;
	while(std::getline(tfs, line))
	{

		std::istringstream lines(line);
		std::string w, x, y, z;

		std::string fn_pc = path + FOLDER_PC + std::to_string(cnt) + ".pcd";
		std::string fn_img = path + FOLDER_IMG + std::to_string(cnt) + ".png";

		std::getline(lines, w, ',');
		std::getline(lines, x, ',');
		std::getline(lines, y, ',');
		std::getline(lines, z, ',');
		ele.c2w_q = Eigen::Quaterniond( std::stod(w), std::stod(x), std::stod(y), std::stod(z) );
		ele.c2w_q.normalize();
		std::getline(lines, x, ',');
		std::getline(lines, y, ',');
		std::getline(lines, z, ',');
		ele.c2w_t = Eigen::Translation3d( std::stod(x), std::stod(y), std::stod(z) );

		std::getline(lines, w, ',');
		std::getline(lines, x, ',');
		std::getline(lines, y, ',');
		std::getline(lines, z, ',');
		ele.c2l_q = Eigen::Quaterniond( std::stod(w), std::stod(x), std::stod(y), std::stod(z) );
		ele.c2l_q.normalize();
		std::getline(lines, x, ',');
		std::getline(lines, y, ',');
		std::getline(lines, z, ',');
		ele.c2l_t = Eigen::Translation3d( std::stod(x), std::stod(y), std::stod(z) );

		std::getline(lines, w, ',');
		std::getline(lines, x, ',');
		std::getline(lines, y, ',');
		std::getline(lines, z, ',');
		ele.l2w_q = Eigen::Quaterniond( std::stod(w), std::stod(x), std::stod(y), std::stod(z) );
		ele.l2w_q.normalize();
		std::getline(lines, x, ',');
		std::getline(lines, y, ',');
		std::getline(lines, z, ',');
		ele.l2w_t = Eigen::Translation3d( std::stod(x), std::stod(y), std::stod(z) );

		ele.pc = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
		pcl::io::loadPCDFile(fn_pc, *ele.pc);

		ele.img = std::make_shared<cv::Mat>();
		*ele.img = cv::imread(fn_img, CV_LOAD_IMAGE_COLOR);

		pose.push_back(ele);
		cnt++;
	}

	tfs.close();

	if(!is_load_pairs)
	{
		return pose.size();
	}

	// load un-filtered data
	std::ifstream fp(path + FILE_PAIRS);
	if(fp)
	{
		std::cout<< "[INFO] " <<FILE_PAIRS << " exist, try to prase"<<std::endl;
		fp.close();

		std::vector<std::vector<int>> v_pair;
		std::vector<std::vector<double>> v_pc;
		std::vector<std::vector<double>> v_img;

		bool res_parse = utils::csv2vector<int>(path+FILE_PAIRS, v_pair,2) &&
						 utils::csv2vector<double>(path+FILE_IMG, v_img)&&
						 utils::csv2vector<double>(path+FILE_PC, v_pc);

		if(res_parse)
		{
			std::cout<< "parse ok "<< v_pair.size()<<std::endl;

			pairs.clear();
			for (std::size_t i=0; i<v_pair.size(); i++ )
			{
				pairpose_struct sp;
				sp.pid1 = v_pair[i][0];
				sp.pid2 = v_pair[i][1];

				Eigen::Matrix4d trans_img, trans_pc;
				for(uint8_t m=0; m<4; m++)
				{
					for(uint8_t n=0; n<4; n++)
					{
						trans_img(m,n) = v_img[4*i+m][n];
						trans_pc(m,n) = v_pc[4*i+m][n];
					}
				}
				sp.img_t = trans_img;
				sp.pc_t = trans_pc;

				pairs.push_back(sp);
			}
			is_loaded_pairs = true;
		}
	}

	return pose.size();
}

bool calibration::cal_pairs()
{
	if(pose.size() < 3)
	{
		std::cout << "---ERROR:At least 3 pose data are needed" << std::endl;
		return false;
	}

	pairpose_struct pa;

	for(uint32_t i = 0; i < pose.size(); i++)
	{
		pa.pid1 = i;
		for(uint32_t j = 1; j < pair_factor; j++)
		{
			pa.pid2 = i + j;
			if(pa.pid2 < pose.size())
			{
				pairs.push_back(pa);
				std::cout << "add pair:  (" << pa.pid1 << "," << pa.pid2 << ")" << std::endl;
			}
		}
	}
	return true;
}

bool calibration::cal_img_trans()
{
	cv::ORB orb(2000);
	std::size_t cnt = 0;

	for(std::vector<pose_struct>::iterator it = pose.begin(); it != pose.end(); it++)
	{
		orb( *it->img, cv::Mat(), it->img_kps, it->img_desp );
		std::cout << "pose: "<< cnt++  << " find " << it->img_kps.size() << " features" << std::endl;
	}


	imgTransEstimator es(K, img_trans_threshold,
						 img_trans_iterations, img_trans_ba_iterations, img_trans_ba_verbose);

	double knn_match_ratio = 0.8;
	cv::Ptr<cv::DescriptorMatcher>  matcher = cv::DescriptorMatcher::create( "BruteForce-Hamming");

	auto it = pairs.begin();
	while (it != pairs.end() )
	{
		std::vector< std::vector<cv::DMatch> > matches_knn;
		std::vector< cv::DMatch > matches;

		matcher->knnMatch(pose[it->pid1].img_desp, pose[it->pid2].img_desp, matches_knn, 2);

		for ( size_t i = 0; i < matches_knn.size(); i++ )
		{
			if (matches_knn[i][0].distance < knn_match_ratio * matches_knn[i][1].distance )
			{
				matches.push_back( matches_knn[i][0] );
			}
		}

		if (matches.size() <= 20)
		{
			std::cout << "WARN: no enough matches, pair" << it->pid1 << "  " << it->pid2 << std::endl;
			it = pairs.erase(it);
			if(pairs.size() < 3 )
			{
				std::cout << "ERROR: no enough valid pairs"<< std::endl;
				return false;
			}
			continue;
		}

		for ( auto m : matches )
		{
			it->pts1.push_back( pose[it->pid1].img_kps[m.queryIdx].pt );
			it->pts2.push_back( pose[it->pid2].img_kps[m.trainIdx].pt );
		}

		std::cout<< "pair: "<< it->pid1 << " and "<<it->pid2<<std::endl;
		std::cout << "filtered find matches #" << matches.size() << std::endl;

		// T from pts2 to pts1
		es.estimate(it->pts2, it->pts1, it->img_t);

		it++;
	}

	return true;
}

bool calibration::cal_pc_trans()
{
	std::cout << "figure point cloud transformation" << std::endl;

	pcTransEstimator estimator(conf_pc.pcm_config_path);

	for (auto it=pairs.begin(); it!=pairs.end(); it++)
	{
		std::cout<< "pair: "<< it->pid1 << " and "<<it->pid2<<std::endl;
		// T from pid2 to pid1
		estimator.estimate(*pose[it->pid1].pc, *pose[it->pid2].pc, it->pc_t );
	}

	return true;
}


bool calibration::cal_img_lines()
{
	line2 line;
	cv::RNG rng(0xffffff);
	cv::Mat img, img_g;

	Eigen::Vector3d vec_y = K*T.rotation().inverse().col(1);
	if(vec_y(2) != 0)
	{
		vec_y(0) = vec_y(0)/vec_y(2);
		vec_y(1) = vec_y(1)/vec_y(2);
	}
	double dir_truth = std::atan2(vec_y(1), vec_y(0))/PI*180;
	if(dir_truth <0)
	{
		dir_truth  = dir_truth + 180;
	}
	std::cout<< "img line direction should be: "<< dir_truth<<std::endl;

	for(std::size_t i=0; i< pose.size(); i++)
	{
		pose[i].img->copyTo(img);
		cv::cvtColor(img, img_g, CV_BGR2GRAY);

		if(edl_detector_img.EDline(img_g) < 0)
		{
			std::cerr<<"pose # "<< i << " edline failed " << std::endl;
			continue;
		}

		for (std::size_t j=0; j < edl_detector_img.lineEndpoints_.size(); j++)
		{
			auto& it = edl_detector_img.lineEndpoints_[j];
			auto& eq = edl_detector_img.lineEquations_[j];

			if(eq[0]*eq[0]+eq[1]*eq[1]  == 0)
			{
				std::cerr<< "coff err "<< std::endl;
				continue;
			}
			line.dir = std::atan2(-eq[0], eq[1])/PI*180;
			if(line.dir<0)
			{
				line.dir += 180;
			}

			if( std::abs(line.dir - dir_truth) > conf_line.dir_err_max )
			{
				continue;
			}

			line.coef << eq[0], eq[1], eq[2];
			line.p0 << it[0], it[1];
			line.p1 << it[2], it[3];

			pose[i].img_lines.push_back(line);

			cv::Scalar color(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
			cv::line(img, cv::Point2f(it[0], it[1]), cv::Point2f(it[2], it[3]), color,1);
			cv::circle(img, cv::Point2f(it[0], it[1]),2, color,3);
			cv::circle(img, cv::Point2f(it[2], it[3]),2, color,3);
		}

		std::cout<< "img: "<<i<<" line: "<< pose[i].img_lines.size()<< std::endl;
		std::string fn = path+"/image_lines/"+std::to_string(i)+".png";
		cv::imwrite(fn, img);

	}
	return true;
}


bool calibration::cal_pc_lines()
{
	line3 line;
	std::string fn;
	cv::RNG rng(0xffffff);

	for (std::size_t pid = 0; pid< pose.size(); pid++)
	{
		auto& it = pose[pid];

		pcl::PointCloud<pcl::PointXYZRGB> pcc;
		pcl::copyPointCloud<pcl::PointXYZ, pcl::PointXYZRGB>(*it.pc, pcc);
		for(std::size_t i=0; i<pcc.points.size(); i++)
		{
			pcc.points[i].r = 255;
			pcc.points[i].g = 255;
			pcc.points[i].b = 255;
		}

		std::vector<std::vector<std::shared_ptr<std::vector<uint32_t>>>>
		project_pc {conf_pc.size, {conf_pc.size, nullptr}} ;
		Eigen::Vector4f xyz_range_min, xyz_range_max;

		pcl::getMinMax3D(*it.pc , xyz_range_min, xyz_range_max);

		double x_resolution = (xyz_range_max(0) - xyz_range_min(0)) / conf_pc.size;
		double z_resolution = (xyz_range_max(2) - xyz_range_min(2)) / conf_pc.size;

		auto& pts = it.pc->points;

		for (std::size_t i = 0; i < pts.size(); i++)
		{
			if (pts[i].y < (xyz_range_min(1) + 0.04))
			{
				continue;
			}
			uint32_t row = (pts[i].z - xyz_range_min(2)) / z_resolution;
			uint32_t col = (pts[i].x - xyz_range_min(0)) / x_resolution;

			if (row >= conf_pc.size)
			{
				row = conf_pc.size - 1;
			}
			if (col >= conf_pc.size)
			{
				col = conf_pc.size - 1;
			}

			if (project_pc[row][col] == nullptr)
			{
				project_pc[row][col] = std::make_shared<std::vector<uint32_t>>(1, i);
			}
			else
			{
				project_pc[row][col]->push_back(i);
			}

		}

		cv::Mat img = cv::Mat::zeros(conf_pc.size, conf_pc.size, CV_8U);

		for (std::size_t i = 0; i < conf_pc.size; i++)
		{
			for (std::size_t j = 0; j < conf_pc.size; j++)
			{
				if (project_pc[i][j] != nullptr && project_pc[i][j]->size() > conf_pc.num_min )
				{
					double tmp=std::pow(project_pc[i][j]->size(), conf_pc.exp);
					if(tmp > 255)
					{
						tmp = 255;
					}
					img.at<uint8_t>(i, j) = uint8_t(tmp);
				}
			}
		}

		fn = path +"/pointcloud_lines/"+std::to_string(pid)+".png";
		cv::imwrite(fn, img);

		cv::Mat img_color;
		cv::cvtColor(img, img_color, CV_GRAY2BGR);

		if(edl_detector_pc.EDline(img) < 0)
		{
			std::cerr << pid <<"edline failed " << std::endl;
			continue;
		}

		std::list<std::array<uint16_t,2>> endpoints;
		for (std::size_t i=0; i < edl_detector_pc.lineEndpoints_.size(); i++)
		{
			auto& kv = edl_detector_pc.lineEndpoints_[i];

			cv::Scalar color(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
			cv::line(img_color, cv::Point2f(kv[0], kv[1]), cv::Point2f(kv[2], kv[3]), color,1);
			cv::circle(img_color, cv::Point2f(kv[0], kv[1]),2, color,3);
			cv::circle(img_color, cv::Point2f(kv[2], kv[3]),2, color,3);

			std::array<uint16_t,2> tmp;
			tmp[0] = static_cast<uint16_t>(kv[0]);
			tmp[1] = static_cast<uint16_t>(kv[1]);
			endpoints.push_back(tmp);

			tmp[0] = static_cast<uint16_t>(kv[2]);
			tmp[1] = static_cast<uint16_t>(kv[3]);
			endpoints.push_back(tmp);
		}

		for(auto out=endpoints.begin(); out!=endpoints.end(); out++)
		{
			auto next = std::next(out);
			while(next != endpoints.end())
			{
				if(utils::point_distance(*out, *next) < conf_pc.dist_min)
				{
					next = endpoints.erase(next);
				}
				else
				{
					next++;
				}
			}
		}

		for(auto kv:endpoints)
		{
			cv::circle(img_color, cv::Point2f(kv[0], kv[1]),5, cv::Scalar(255,0,255),1);

			//check position
			bool is_in_img = false;
			Eigen::Vector4d ep_w;
			ep_w(3) = 1;
			ep_w(0) = kv[0]*x_resolution+xyz_range_min(0);
			ep_w(1) = (xyz_range_min(1)+xyz_range_max(1))/2;
			ep_w(2) = kv[1]*z_resolution+xyz_range_min(2);

			Eigen::Vector4d ep_c = (T.matrix().inverse())*ep_w;

			// check if this line is in the img
			if ( ep_c(2) > 0 )
			{
				double u_img = ep_c(0)*K(0,0)/ep_c(2);
				double v_img = ep_c(1)*K(0,0)/ep_c(2);
				if(std::abs(u_img) <= K(0,2) && std::abs(v_img) <= K(0,2) )
				{
					is_in_img = true;
					cv::circle(img_color, cv::Point2f(kv[0], kv[1]),5, cv::Scalar(0,0,255),5);
				}

			}

			uint32_t col_0, col_1, row_0, row_1, col, row;
			uint32_t pt_size_min = 255;

			row = kv[1];
			row_0 = utils::index_around(kv[1]- conf_pc.square_min, conf_pc.size -1);
			row_1 = utils::index_around(kv[1]+ conf_pc.square_min, conf_pc.size -1);

			col = kv[0];
			col_0 = utils::index_around(kv[0]- conf_pc.square_min, conf_pc.size-1);
			col_1 = utils::index_around(kv[0]+ conf_pc.square_min, conf_pc.size-1);

			// find cnt_min around
			for(uint32_t i=row_0; i<=row_1; i++)
			{
				for (uint32_t j=col_0; j<=col_1; j++)
				{
					auto ptr = project_pc[i][j];
					if(ptr != nullptr)
					{
						if (ptr ->size() < pt_size_min)
						{
							pt_size_min = ptr->size();
							col = j;
							row = i;
						}
						for( auto m:(*ptr) )
						{
							if(is_in_img)
							{
								pcc.points[m].r = 255;
								pcc.points[m].g = 0;
								pcc.points[m].b = 0;
							}
							else
							{
								pcc.points[m].r = 0;
								pcc.points[m].g = 0;
								pcc.points[m].b = 255;
							}
						}
					}
				}
			}

			if(pt_size_min< 255 && is_in_img)
			{
				line.coef.setZero();
				auto ptr = project_pc[row][col];
				for( auto m:(*ptr) )
				{
					line.coef(0) += pts[m].x;
					line.coef(1)  += pts[m].y;
					line.coef(2) += pts[m].z;

					pcc.points[m].r = 0;
					pcc.points[m].g = 255;
					pcc.points[m].b = 0;
				}

				for(uint8_t i=0; i<3; i++)
				{
					line.coef(i) = line.coef(i)/ptr->size();
				}
				line.coef(3) = 1;

				it.pc_lines.push_back(line);
			}
		}

		std::cout<< "pc: " <<pid << " lines: "<< it.pc_lines.size()<< std::endl;
		fn = path +"/pointcloud_lines/"+std::to_string(pid)+"_color.png";
		cv::imwrite(fn, img_color);
		fn = path +"/pointcloud_lines/"+std::to_string(pid)+".pcd";
		pcl::io::savePCDFileBinary(fn, pcc);
	}

	return true;
}

// TODO: hand-eye calibration
bool calibration::cal_initial()
{
	std::list< const pairpose_struct* > pairs_f;
	// filter pairs which has big angle error
	for( auto it=pairs.begin(); it!=pairs.end(); it++ )
	{
		Eigen::Quaterniond lidar{ it->pc_t.rotation() };
		Eigen::Quaterniond cam{ it->img_t.rotation() };

		double err = std::abs(std::acos(lidar.w()) - std::acos(cam.w()) );
		err = err/PI*180*2;

		if( err < pair_angle_err_threshold_1 )
		{
			pairs_f.push_back(&*it);
			std::cout<<"[INFO] #1 small angle error: "<< err
					 <<" pid1: " << it->pid1
					 <<" pid2: " << it->pid2
					 <<std::endl;
		}
	}

	std::cout<<"[INFO] #1 after filter, pair size "<< pairs_f.size()<< std::endl;

	// calculate translation
	Eigen::MatrixXd A(4*pairs_f.size(), 4);
	Eigen::Matrix4d tl, tr;
	double w,x,y,z;
	std::size_t i =0;

	for(auto it:pairs_f)
	{
		Eigen::Quaterniond cam( it->img_t.rotation() );
		Eigen::Quaterniond lid( it->pc_t.rotation() );

		cam.normalize();
		lid.normalize();

		w=lid.w();
		x=lid.x();
		y=lid.y();
		z=lid.z();

		tl << w, -x, -y, -z,
		x, w, -z, y,
		y, z, w, -x,
		z, -y, x,w;

		w=cam.w();
		x=cam.x();
		y=cam.y();
		z=cam.z();

		tr << w, -x, -y, -z,
		x, w, z, -y,
		y, -z, w, x,
		z, y, -x,w;

		A.block<4,4>(4*i,0) = tl - tr;
		i++;
	}

	Eigen::JacobiSVD<Eigen::MatrixXd> svd(A,Eigen::ComputeThinV);
	Eigen::Vector4d vec_q = svd.matrixV().block<4,1>(0,3);

	//assign to T
	T.matrix().setIdentity();
	Eigen::Quaterniond q( vec_q(0), vec_q(1), vec_q(2), vec_q(3) );
	T.matrix().block<3,3>(0,0) = q.matrix();

	Eigen::VectorXd vec_err(4*pairs_f.size());
	vec_err = A*vec_q;
	i = 0;

	// filter large angle error pairs
	auto it = pairs_f.begin();
	while( it != pairs_f.end() )
	{
		double err =  vec_err.segment<4>(4*i).norm();

		if(  err < pair_angle_err_threshold_2 )
		{
			it++;
		}
		else
		{
			it = pairs_f.erase(it);
		}

		i++;
	}

	// calculate translation
	std::cout<<"[INFO] #2 after filter, pair size "<< pairs_f.size()<< std::endl;
	A.resize(3*pairs_f.size(), pairs_f.size()+3);
	A.setZero();
	vec_err.resize(3*pairs_f.size(), 1);

	i=0;;
	for(auto its:pairs_f)
	{
		A.block<3,3>(3*i,0) = Eigen::Matrix3d::Identity() - its->pc_t.rotation();
		A.block<3,1>(3*i, 3+i) = q.matrix() * its->img_t.translation();
		vec_err.segment<3>(3*i) = its->pc_t.translation();
		i++;
	}

	svd = Eigen::JacobiSVD<Eigen::MatrixXd>(A,Eigen::ComputeFullU|Eigen::ComputeThinV);
	Eigen::VectorXd s = svd.singularValues();
	Eigen::MatrixXd st( 3+pairs_f.size(), 3*pairs_f.size() );
	for (std::size_t i=0; i<(3+pairs_f.size()); i++)
	{
		if (s(i)!=0)
		{
			st(i,i)= 1.0/s(i);
		}
	}

	Eigen::VectorXd res = svd.matrixV()*st*(svd.matrixU().transpose())*vec_err;

	T.matrix().block<3,1>(0,3) = res.head(3);

	std::cout <<"matrix T:"<<std::endl<< T.matrix() <<std::endl;

	return true;
}



bool calibration::cal_line_match()
{
	Eigen::Matrix4d T_i = T.matrix().inverse();
	int id = -1;

	std::cout<< "matched lines start "<< std::endl;

	for (auto it:pose )
	{
		id++;

		if(it.pc_lines.size()==0 || it.img_lines.size()==0)
		{
			std::cout<< "id: "<<id
					 <<" img "<< it.img_lines.size()
					 <<" pc "<< it.pc_lines.size()<< std::endl;
			continue;
		}

		for(auto itpl:it.pc_lines)
		{
			double dist_min = K(0,2);
			std::size_t index_min = 0;
			std::size_t index = 0;
			for(auto itil:it.img_lines)
			{
				Eigen::Vector3d pt_i = K*((T_i*itpl.coef).block<3,1>(0,3));
				pt_i(0) = pt_i(0)/pt_i(2);
				pt_i(1) = pt_i(1)/pt_i(2);
				double dist = std::abs( pt_i(0)*itil.coef(0) + pt_i(1)*itil.coef(1)+itil.coef(2) );
				dist = dist / std::sqrt( itil.coef(0)*itil.coef(0) + itil.coef(1)*itil.coef(1) );

				if(dist<conf_line.dist_err_max && dist< dist_min)
				{
					dist_min = dist;
					index_min = index;
					std::cout<< "id: "<<id<<" dist "<< dist<< std::endl;
				}
				index++;
			}

			if(dist_min < K(0,2))
			{
				// find one line pair with dist err min
				Eigen::Matrix<double,1,8> tmp;
				tmp(0,3) = 0;
				tmp.block<1,3>(0,0) = it.img_lines[index_min].coef;
				tmp.block<1,4>(0,4) = itpl.coef.transpose();
				matched_lines.push_back(tmp);
			}

		}

	}

	std::cout<< "Total matched lines: "<< matched_lines.size()<< std::endl;
	return true;
}

bool calibration::cal_optimal()
{

	return true;
}

bool calibration::calibrate()
{
	if( !is_loaded_pairs )
	{
		std::cout<<"[INFO] re-calculate transpose"<< std::endl;
		cal_pairs();
		cal_img_trans();
		cal_pc_trans();

		// save un-filtered data
		log_pairs("mpairs.csv");
	}

	cal_initial();
	log_pairs("mpairs_f.csv");

	//cal_img_lines();
	//cal_pc_lines();

	//cal_line_match();

	//std::string fn = path +"/lines.csv";
	//std::ofstream f(fn);
	//f.precision(15);
	//for (auto it:matched_lines)
	//{
	//f<< it<<std::endl;
	//}
	//f.close();


	return true;
}


void calibration::log_pairs(std::string const& fn)
{
	// pid0 pid 1  pc: q[wxyz] pc:t  img:q1 img:t1
	std::cout << "pairs #" << pairs.size() << " to record" << std::endl;
	std::ofstream f(fn);

	for(auto& it:pairs)
	{
		f << it.pid1 << ","
		  << it.pid2 << ",";

		Eigen::Quaterniond pq(it.pc_t.rotation());
		Eigen::Vector3d pt(it.pc_t.translation());

		f<<pq.w() << ","
		 << pq.x() << ","
		 << pq.y() << ","
		 << pq.z() << ","
		 << pt(0) << ","
		 << pt(1) << ","
		 << pt(2) << ",";

		Eigen::Quaterniond mq(it.img_t.rotation());
		Eigen::Vector3d mt(it.img_t.translation());

		f<< mq.w() << ","
		 << mq.x() << ","
		 << mq.y() << ","
		 << mq.z() << ","
		 << mt(0) << ","
		 << mt(1) << ","
		 << mt(2) << std::endl;
	}

	f.close();
}
