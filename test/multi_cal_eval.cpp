/**
  ******************************************************************************
  * @file	mutil_cal_eval.cpp
  * @author	Nick.Liao
  * @version V1.0.0
  * @date	2017-11-23
  * @brief	mutil_cal_eval.cpp
  ******************************************************************************
  * @attention
  *
  * Copyright (C) 2017 Nick.Liao <simplelife_nick@hotmail.com>
  * Distributed under terms of the MIT license.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "mutil_cal_eval.h"

#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/


/**
  * @brief
  * @param
  * @note
  * @return None
  */

MutilCalEval::MutilCalEval():
	pm_(nullptr),
	id_base_(0),
	calibration_error_(0),
	is_valid_(false)
{
	pcs_.clear();
}


MutilCalEval::MutilCalEval(const std::string& fn_config,
						   std::shared_ptr<lqh::utils::pm> pm):
	pm_(pm),
	id_base_(0),
	calibration_error_(0),
	is_valid_(false)
{
	YAML::Node nd = YAML::LoadFile(fn_config);
	config(nd);
}

MutilCalEval::MutilCalEval(const YAML::Node& nd,
						   std::shared_ptr<lqh::utils::pm> pm):
	pm_(pm),
	id_base_(0),
	calibration_error_(0),
	is_valid_(false)
{
	config(nd);
}


void MutilCalEval::config(const YAML::Node& nd)
{
	if(!pm_)
	{
		pm_ = std::make_shared<lqh::utils::pm>();
	}

	auto& pf = nd["plane_fit"];

	plane_fit_distance_threshold_ =
		pf["distance_threshold"] ? pf["distance_threshold"].as<double>() : 0.05;
	plane_fit_points_min_num_ =
		pf["poins_min_num"] ? pf["points_min_num"].as<uint32_t>() : 50;

	ground_offset_ = pf["ground_offset"] ? pf["ground_offset"].as<double>() : 0.25;


	auto& pls = nd["planes"];

	if( !pls.IsSequence() )
	{
		std::cerr << "E[yaml]: node(planes) is not array"<< std::endl;
		return;
	}

	plane_boxs_.reserve(pls.size());

	for(auto it=pls.begin(); it != pls.end(); it++)
	{
		plane_boxs_.emplace_back();
		auto& last = plane_boxs_.back();
		last.reserve(it->size());

		for(auto li=it->begin(); li != it->end(); li++)
		{
			last.emplace_back(
				(*li)["min"].as<double>(),
				(*li)["max"].as<double>(),
				(*li)["axis"].as<std::string>()
			);

			char axis = last.back().axis[0];
			if( axis == 'z' || axis == 'Z' )
			{
				last.back().is_ground = true;
			}
		}
	}

	is_valid_ = true;
}


MutilCalEval::operator bool() const
{
	return is_valid_;
}

void MutilCalEval::add_pm(std::shared_ptr<lqh::utils::pm> pm)
{
	pm_ = pm;
}


/**
 * @brief transform the limit-box to new frame
 *
 * @param tf 6dof transformation from in to out
 * @param in
 * @param out
 */
void MutilCalEval::transform_limit_box(const Eigen::Matrix4d& tf,
									   const plane_box_limit& in,
									   plane_box_limit& out)
{
	if(in.size() < 2)
	{
		// input error
		out.clear();
		return;
	}
	// col(0) min, col(1)max
	Eigen::Matrix<double,4,2> in_point, out_point;
	in_point.col(0).setConstant(-100);
	in_point.col(1).setConstant(200);
	in_point.row(3).setIdentity();
	out_point.setZero();

	for(const auto&it: in)
	{
		uint8_t axis = 0;
		if(it.axis == "x")
		{
			axis = 0;
		}
		else if(it.axis == "y")
		{
			axis = 1;
		}
		else if(it.axis == "z")
		{
			axis = 2;
		}
		else
		{
			std::cerr << "E: wrong axis, input: " << it.axis << std::endl;
			continue;
		}
		in_point(axis,0) = it.min;
		in_point(axis,1) = it.max;
	}

	out_point = tf*in_point;
    out.clear();
    out.emplace_back(out_point.row(0).minCoeff(), out_point.row(0).maxCoeff(), "x");
    out.emplace_back(out_point.row(1).minCoeff(), out_point.row(1).maxCoeff(), "y");
    out.emplace_back(out_point.row(2).minCoeff(), out_point.row(2).maxCoeff(), "z");
//	for(auto&it: out)
//	{
//		uint8_t axis = 0;
//		if(it.axis == "x")
//		{
//			axis = 0;
//		}
//		else if(it.axis == "y")
//		{
//			axis = 1;
//		}
//		else if(it.axis == "z")
//		{
//			axis = 2;
//		}
//		else
//		{
//			std::cerr << "E: wrong axis, input: " << it.axis << std::endl;
//			continue;
//		}
//		it.min = out_point(axis, 0);
//		it.max = out_point(axis, 1);
//	}

	return;
}


/**
 * @brief fit 3D plane in give box
 *	      not matter of success/failure, the pcs_[id_pc].planes[id_plane] has
 *	      to be set
 * @param id_pc 	pointcloud index for pcs_
 * @param id_plane  plane index for plane_boxs_
 *
 */
// TODO 
void MutilCalEval::fit_plane(uint32_t id_pc, uint32_t id_plane )
{
	using PCI = pcl::PointCloud<pcl::PointXYZI>;
	PCI::Ptr pc = pcs_[id_pc].pc;

	/**
	 *  clip the minimal box
	 */
	plane_box_limit box;
	if(id_pc != 0)
	{
		transform_limit_box(eps_[id_pc].matrix().inverse(),
							plane_boxs_[id_plane], box);
	}

	auto& limits = id_pc == 0 ? plane_boxs_[id_plane] : box;
	//auto& limits = plane_boxs_[id_plane];
	auto& plane = pcs_[id_pc].planes[id_plane];

	// VERY IMPORTANT
	// keep the indices of the original pointcloud
	pcl::IndicesPtr indices_i (new std::vector<int>);
	pcl::IndicesPtr indices_o (new std::vector<int>);

	bool flag = true;

	Eigen::Vector4f min_pt, max_pt;
	pcl::getMinMax3D(*pc, min_pt, max_pt);

	for(auto& it:limits)
	{
		pcl::PassThrough<pcl::PointXYZI> pass;
		pass.setInputCloud(pc);
		if(indices_i->size() )
		{
			pass.setIndices(indices_i);
		}
		pass.setFilterFieldName(it.axis);

		// special process of z-axis
		// we think z-axis is perpendicular to ground
		/**
		if(it.is_ground)
		{
			pass.setFilterLimits(min_pt(2) - ground_offset_,
								 min_pt(2) + ground_offset_);
		}
		else
		{

			pass.setFilterLimits(it.min, it.max);
		}
		*/
		pass.setFilterLimits(it.min, it.max);
		pass.filter(*indices_o);

		if(indices_o->size() < plane_fit_points_min_num_)
		{
			flag = false;
			break;
		}
		std::swap(indices_i, indices_o);
	}

	// TODO:
	if(flag)
	{
		/**
		 *	fit plane: estimate the best plane model
		 */
		pcl::ModelCoefficients coef;
		pcl::PointIndices ids;
		pcl::SACSegmentation<pcl::PointXYZI> seg;
		seg.setOptimizeCoefficients (true);
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setDistanceThreshold (plane_fit_distance_threshold_);
		seg.setInputCloud (pc);
		seg.setIndices(indices_i);
		seg.segment(ids, coef);

		Eigen::Vector4d coefs ( coef.values[0], coef.values[1],
								coef.values[2], coef.values[3]);

		// a good plane ?
		if(ids.indices.size() > plane_fit_points_min_num_ &&
				coefs.head(3).norm() > 1e-6)
		{
			// assemble data

			// @note: normalize the coefficients
			plane.coef = coefs/coefs.head(3).norm();

			plane.points.resize(Eigen::NoChange, ids.indices.size());
			for(uint32_t i=0; i<ids.indices.size(); i++)
			{
				plane.points(0,i) = pc->points[ids.indices[i]].x;
				plane.points(1,i) = pc->points[ids.indices[i]].y;
				plane.points(2,i) = pc->points[ids.indices[i]].z;
			}
			plane.points.bottomRows(1).setOnes();

			std::swap(ids.indices, plane.indices);

			// calculate error
			plane.calculate_error();
			// success, return
			return;
		}
	}

	// fail to fit plane
	plane.coef.setZero();
	plane.p2p_err_sum = 0;
	plane.indices.clear();
	return;
}

// TODO:
void MutilCalEval::add_pointcloud(const CalibPointCloud &cpc)
{
	MutilCalEval::pointcloud pc;
	pc.pc = boost::make_shared<pcl::PointCloud<pcl::PointXYZI> >(); // initialize a shared pointer
    // 	https://stackoverflow.com/questions/46650052/initialize-a-pcl-pointcloudptr
	*pc.pc = cpc.cloud_;
	for (size_t i = 0; i < cpc.v_plane_.size(); i++)
	{
		MutilCalEval::plane p;
		p.coef = cpc.v_coeff_[i].cast<double>();
		p.normal = cpc.v_normal_[i].cast<double>();
		p.points.resize(Eigen::NoChange, cpc.v_plane_[i].size());
		for(uint32_t j = 0; j < cpc.v_plane_[i].size(); j++)
		{
			p.points(0,j) = double(cpc.v_plane_[i].points[j].x);
			p.points(1,j) = double(cpc.v_plane_[i].points[j].y);
			p.points(2,j) = double(cpc.v_plane_[i].points[j].z);
			p.points(3,j) = 1.0;
		}
// 		std::cout << p.point_num() << std::endl;
// 		std::cout << p.coef.transpose() << std::endl;
		p.calculate_error();
		pc.planes.push_back(p);
	}
	pcs_.push_back(pc);
}


uint32_t MutilCalEval::add_pointcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, bool is_wait)
{
	if(!pc)
	{
		return INT_MAX;
	}

	pcs_.emplace_back();
	auto& last = pcs_.back();

	last.pc = pc;
	last.planes.resize( plane_boxs_.size() );

	uint32_t id_pc = pcs_.size() - 1;

	for(uint32_t i=0; i<plane_boxs_.size(); i++)
	{
		pm_->post( pm_helper_, boost::bind(&MutilCalEval::fit_plane, this, id_pc, i) );
	}

	if(is_wait)
	{
		// block here wait
		pm_->wait(pm_helper_);
	}

	return id_pc;
}

void MutilCalEval::add_pointclouds(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> pcs)
{

	if( !pcs.size() )
	{
		return;
	}

	for(auto& it: pcs)
	{
		add_pointcloud(it, false);
	}

	// block here wait for finish
	pm_->wait(pm_helper_);
}

double MutilCalEval::get_base_error()
{
	double error = 0;
	uint32_t num = 0;

	for(auto& it: pcs_)
	{
		for(auto& li: it.planes)
		{
			error += li.p2p_err_sum;
			num += li.point_num();
		}
	}
	
	std::cout << "Error: " << error << std::endl;
	std::cout << "Point Number: " << num << std::endl;

	return error/num;
}

double MutilCalEval::get_calibration_error()
{
	if(eps_.size() > 0 && calibration_error_ >0)
	{
		return calibration_error_;
	}
	else
	{
		std::cerr << "E: Call calibrate() first" << std::endl;
		return -1;
	}
};

bool MutilCalEval::get_parameters(std::vector<Eigen::Affine3d>& eps)
{
	if(pcs_.size() == 0 || eps_.size() == 0)
	{
		eps.clear();
		return false;
	}
	eps = eps_;
	return true;
}


bool MutilCalEval::set_base_id(uint32_t id_base)
{
	if( id_base >= pcs_.size() )
	{
		return false;
	}
	id_base_ = id_base;
	return true;
}


bool MutilCalEval::set_parameters(const std::vector<Eigen::Affine3d>& ep)
{
	eps_ = ep;
	return true;
}

uint32_t MutilCalEval::save_segmented_pointcloud(const std::string& out_dir,
		const std::string& prefix )
{
	std::string fn = out_dir + "/" + prefix;
	std::mutex mtx;
	uint32_t cnt = 0;

	auto handler_pointcloud = [this, fn](uint32_t id, uint32_t* counter, std::mutex* mx)
	{
		bool flag = false;

		auto& pc = pcs_[id];
		pcl::PointCloud<pcl::PointXYZRGB> pc_c;
		pcl::copyPointCloud(*pc.pc, pc_c);
		for(auto&it: pc_c.points)
		{
			it.rgba = 0xFFFFFFF;
		}

		lqh::utils::color::rgbs colors =
			lqh::utils::color::get_rgbs(pc.planes.size());

		auto c_t = colors.cbegin();
		for(auto& it: pc.planes)
		{
			if( it.point_num() )
			{
				for(auto li: it.indices)
				{
					pc_c.points[li].r = (*c_t)[0];
					pc_c.points[li].g = (*c_t)[1];
					pc_c.points[li].b = (*c_t)[2];
				}
				flag = true;
			}
			c_t++;
		}

		pcl::io::savePCDFileBinary( fn+std::to_string(id)+".pcd", pc_c);
		std::lock_guard<std::mutex> lg(*mx);
		(*counter)++;
	};

	auto handler_plane = [this, fn](uint32_t id_pc, uint32_t id_plane)
	{
		std::string csv = fn + std::to_string(id_pc) + "_" +
						  std::to_string(id_plane) + ".csv";
		save_plane_csv(csv, id_pc, id_plane);
	};

	auto handler_plane_coef = [this, fn](uint32_t id_pc)
	{
		if(pcs_[id_pc].planes.size() == 0)
		{
			return;
		}

		std::string fn_csv = fn + std::to_string(id_pc) + ".csv";
		std::ofstream f(fn_csv);
		if(!f.good())
		{
			return;
		}

		Eigen::IOFormat csv(NUMBER_PRECISION,0,",");

		for(auto& p:pcs_[id_pc].planes)
		{
			f << p.coef.transpose().format(csv) << std::endl;
		}
		f.flush();
		f.close();
	};

	for(uint32_t i=0; i<pcs_.size(); i++)
	{
		pm_->post(pm_helper_, boost::bind<void>(handler_pointcloud, i, &cnt, &mtx));

		pm_->post(pm_helper_, boost::bind<void>(handler_plane_coef,i));
		for(uint32_t j=0; j<pcs_[i].planes.size(); j++)
		{
			pm_->post(pm_helper_, boost::bind<void>(handler_plane, i, j));
		}
	}

	// block until finish
	pm_->wait(pm_helper_);

	return cnt;
}

bool MutilCalEval::save_pointcloud_csv(const std::string& fn,
									   const pcl::PointCloud<pcl::PointXYZI>& pc)
{
	if(pc.size() == 0)
	{
		std::cerr << "E: PointCloud has no point" << std::endl;
		return false;
	}

	std::ofstream f(fn);
	if(!f.good())
	{
		std::cerr << "E: Fail to open file: " << fn << std::endl;
		return false;
	}

	f.precision(NUMBER_PRECISION);

	for(auto& p:pc.points)
	{
		f << p.x << "," << p.y << "," << p.z << "," << p.intensity << std::endl;
	}
	f.flush();
	f.close();
	return true;
}

bool MutilCalEval::save_pointcloud_csv(const std::string& fn, uint32_t id_pc)
{
	if(id_pc > pcs_.size())
	{
		return false;
	}
	return save_pointcloud_csv( fn, *(pcs_[id_pc].pc) );
}

bool MutilCalEval::save_plane_csv(const std::string& fn,
								  uint32_t id_pc, uint32_t id_plane)
{
	if(id_pc > pcs_.size())
	{
		return false;
	}
	if(id_plane > pcs_[id_pc].planes.size())
	{
		return false;
	}
	return save_plane_csv(fn, pcs_[id_pc].planes[id_plane]);
}

bool MutilCalEval::save_plane_csv(const std::string& fn, const plane& pl)
{
	if(pl.indices.size() == 0)
	{
		std::cerr << "E: Plane has no point" << std::endl;
		return false;
	}

	std::ofstream f(fn);
	if(!f.good())
	{
		std::cerr << "E: Fail to open file: " << fn << std::endl;
		return false;
	}

	Eigen::IOFormat csv(NUMBER_PRECISION,0,",");
	f << pl.points.transpose().format(csv);

	return true;
}

/**
 * @brief for ceres optimization
 */

struct MutilCalEval::PlanePlaneError
{
	const Eigen::Matrix4Xd& p0_;
	const Eigen::Matrix4Xd& p1_;
	const Eigen::Vector4d& coef0_;
	const Eigen::Vector4d& coef1_;

	/**
	 * @brief initialize PlanePlaneError, sum_{i=1}^{i<=n}point-to-plane()
	 * @input p0, p1: point
	 * @input coef0, coef1: plane coefficient 
	 */
	PlanePlaneError(const Eigen::Matrix4Xd& p0, const Eigen::Matrix4Xd& p1,
					const Eigen::Vector4d& coef0, const Eigen::Vector4d& coef1):
		p0_(p0),p1_(p1),
		coef0_(coef0),coef1_(coef1) {}

	template <typename T>
	void convert(const T* const p_ptr, const T* const q_ptr,
				 Eigen::Matrix<T,4,4>& tf,
				 Eigen::Matrix<T,4,4>& tf_inv) const
	{
		Eigen::Map<const Eigen::Matrix<T, 3, 1> > p(p_ptr);
		Eigen::Map<const Eigen::Quaternion<T> > q(q_ptr);

		Eigen::Quaternion<T> q_inv = q.conjugate();

		tf.setIdentity();
		tf.topLeftCorner(3,3) = q.matrix();
		tf.topRightCorner(3,1) = p;

		tf_inv.setIdentity();
		tf_inv.topLeftCorner(3,3) = q_inv.matrix();
		tf_inv.topRightCorner(3,1) = -q_inv.matrix()*p;
	}

	template <typename T>
	bool operator()(const T* const q_a_ptr, const T* const p_a_ptr,
					const T* const q_b_ptr,  const T* const p_b_ptr,
					T* residuals) const
	{
		using Matrix4T = Eigen::Matrix<T,4,4>;
		using VectorXT = Eigen::Matrix<T, Eigen::Dynamic, 1>;

		Matrix4T tf0, tf0_inv;
		convert<T>(p_a_ptr, q_a_ptr,tf0, tf0_inv);

		Matrix4T tf1, tf1_inv;
		convert<T>(p_b_ptr, q_b_ptr, tf1, tf1_inv);


		// tf:      0 -> 1
		// tf_inv:  1 -> 0
		Matrix4T tf = tf1_inv*tf0;
		Matrix4T tf_inv = tf0_inv*tf1;

		// residuals = sum(D_i)
		// D_i = |ax_i+by_i+cz_i+d|/sqrt(a*a +b*b+c*c) 
		// or
		// D_i = (ax_i+by_i+cz_i+d)^2/sqrt(a*a +b*b+c*c) 
		// tmp : 1xn
		VectorXT tmp01 = (coef1_.transpose().cast<T>())*tf*(p0_.cast<T>());
// 		T err01 = tmp01.cwiseAbs().sum();
		T err01 = tmp01.cwiseAbs2().sum();

		VectorXT tmp10 = (coef0_.transpose().cast<T>())*(tf_inv)*(p1_.cast<T>());
//		T err10 = tmp10.cwiseAbs().sum();
		T err10 = tmp10.cwiseAbs2().sum();

		//TODO: why?
		//@note: we don't divid err by points num
		// 		 points num can be the weight of its plane
		residuals[0] = err01 + err10;
		return true;
	}
};

/**
 * @brief using segmented plane to optimize the extrinsic parameter
 * n sensors(pointcloud) has C_n^2 = n*(n-1)/2 extrinsic parameter
 * but there are only (n-1) independent extrinsic parameter which
 * are optimized here.
 *
 * First, we define plane-to-plane:
 * We have two plane A,B, A has n points P_i i~[0,n-1], B has the
 * plane mode coeffiicients (a,b,c,d) ax+by+cz+d = 0;
 * point-to-plane distance:
 * 		D_i = |ax_i+by_i+cz_i+d|/sqrt(a*a +b*b+c*c)
 * plane-to-plane distance:
 * 		D   = sum(D_i) 		i~[0,n-1]
 * Hence, here plane A and B are not equal
 *
 * The objective function is sum(all plane-to-plane distance)
 * n pointclouds, m planes per poincloud
 * every two pointclouds: we project and backproject, so we have
 * 						  m*2 plane-to-plane error
 * Finally, we have n*(n-1)/2*m*2 plane-to-plane errors
 * And objective function is the sum of above errors
 *
 * @param eps_init 	initial extrinsic parameters, every one is Vector6d
 * 					first 3 are AngleAxis, last 3 are translation
 * 					note: eps_init[id_base] will be identity, set or not
 * 					doesn't matter
 * @param id_base 	The base sensor for optimization, thus all (n-1)
 * 		  		  	extrinsic parameters are transforamtion between
 * 		  		  	other sensors and base sensor
 * 		  		  	e.g. id_base=1, and here are 4 sensor(0,1,2,3)
 * 		  		  	extrinsic parameters: T_1^0, T_1^2, T_1^2
 */
double MutilCalEval::calibrate(const std::vector<Eigen::Affine3d>& eps_init,
							   uint32_t id_base, bool is_verbose)
{
	// number of pointcloud 
	uint32_t num = pcs_.size(); 
	if(num < 2)
	{
		std::cerr << "No enough pointcloud: " << num << std::endl;
		return -1;
	}
	if(eps_init.size() != num)
	{
		std::cerr << "E: no enough extrinsic parameters, pointclouds:#"
				  << num << std::endl;
		return -2;
	}

	if(id_base < num)
	{
		id_base_ = id_base;
	}

	/**
	 * set initial
	 */
	std::vector<Eigen::Quaterniond> ep_q(pcs_.size());
	std::vector<Eigen::Vector3d> ep_p(pcs_.size());
	for(uint32_t i = 0; i < eps_init.size(); i++)
	{
		ep_q[i] = Eigen::Quaterniond(eps_init[i].rotation());
		ep_p[i] = eps_init[i].translation();
	}
	ep_q[id_base_] = Eigen::Quaterniond::Identity();
	ep_p[id_base_].setZero();

	// TODO: 
	// build up optimization problem
	uint32_t total_inliers = 0;
	ceres::Problem problem;
	ceres::Solver::Summary summary;
	ceres::Solver::Options options;
	ceres::HuberLoss* loss_function (new ceres::HuberLoss(500));
	ceres::LocalParameterization* quaternion_local_parameterization =
		new ceres::EigenQuaternionParameterization;

	for(uint32_t i=0; i<num-1; i++)
	{
		for (uint32_t j=i+1; j<num; j++)
		{
			for(uint32_t k=0; k<pcs_[i].planes.size(); k++)
			{
				//@note: skip invalid plane (zero points)
				if(pcs_[i].planes[k].point_num() == 0 ||
						pcs_[j].planes[k].point_num() == 0)
				{
					continue;
				}

				total_inliers += pcs_[i].planes[k].point_num();
				total_inliers += pcs_[j].planes[k].point_num();

				// initialize
				ceres::CostFunction* cost =
					new ceres::AutoDiffCostFunction<PlanePlaneError, 1,4,3,4,3>(
					new PlanePlaneError( // operator() to calculate cost
						pcs_[i].planes[k].points,
						pcs_[j].planes[k].points,
						pcs_[i].planes[k].coef,
						pcs_[j].planes[k].coef));

				// add operator, variables
				problem.AddResidualBlock(cost, loss_function,
										 ep_q[i].coeffs().data(),
										 ep_p[i].data(),
										 ep_q[j].coeffs().data(),
										 ep_p[j].data());
			}
			problem.SetParameterization(ep_q[i].coeffs().data(),
										quaternion_local_parameterization);
		}
	}
	
	// TODO: fix invariant variables	
	problem.SetParameterBlockConstant( ep_q[id_base_].coeffs().data());
	problem.SetParameterBlockConstant( ep_p[id_base_].data());

	options.linear_solver_type = ceres::DENSE_SCHUR;
	// options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
	options.max_num_iterations = 1000;
	// options.function_tolerance = 1e-4;
	// or use all cpu cores
	options.num_threads = boost::thread::hardware_concurrency() - 1;
	// options.num_linear_solver_threads = options.num_threads;

	if(is_verbose)
	{
		options.minimizer_progress_to_stdout = true;
	}

	ceres::Solve(options, &problem, &summary);

	if(is_verbose)
	{
		std::cout << summary.BriefReport() << std::endl;
		std::stringstream ss;
		ss << summary.BriefReport() << std::endl;
		ss << "iter| mse| cost| cost_change| relative_decrease|" << std::endl;
		for (size_t i=0; i < summary.iterations.size(); i++)
		{
			ss << i << " " 
				<< summary.iterations[i].cost / total_inliers << " "
				<< summary.iterations[i].cost << " "
				<< summary.iterations[i].cost_change << " "
				<< summary.iterations[i].relative_decrease << std::endl;
		}
		ss << std::endl;
		str_opt_report = ss.str();
	}

	// TODO:
	// assign results to eps_
	eps_.resize(num);
	for(uint32_t i=0; i<pcs_.size(); i++)
	{
		eps_[i].matrix().topLeftCorner(3,3) = ep_q[i].matrix();
		eps_[i].matrix().topRightCorner(3,1) = ep_p[i];
	}

	// evaluation average error
	double cost;
	std::vector<double> residuals;

	ceres::Problem::EvaluateOptions eoptions;
	eoptions.num_threads = options.num_threads;

	problem.Evaluate(eoptions, &cost, &residuals, NULL, NULL);
	cost = std::accumulate(residuals.begin(), residuals.end(), 0);

	return cost/total_inliers;
}

double MutilCalEval::calibrate(uint32_t id_base, bool is_verbose)
{
	return calibrate(eps_, id_base, is_verbose);
}

double MutilCalEval::calibrate(const std::vector<Eigen::Matrix4d>& eps_init,
							   uint32_t id_base, bool is_verbose )
{
	if(eps_.size() > 0)
	{
		eps_.clear();
	}

	for(auto &it: eps_init)
	{
		eps_.emplace_back(it);
	}

	return calibrate(eps_, id_base, is_verbose);
}

template <typename T>
bool MutilCalEval::save_merged_pc(const std::string& out_fn,
								  const std::vector<T>& eps)
{
	if(eps.size() != pcs_.size() || pcs_.size() == 0 )
	{
		std::cerr << "E: no enough transformation or no pointclouds" << std::endl;
		return false;
	}

	auto colors = lqh::utils::color::get_rgbs(pcs_.size());

	using PCC = pcl::PointCloud<pcl::PointXYZRGB>;
	PCC pc;
	pc.reserve(pcs_[0].pc->points.size()*pcs_.size());

	PCC pc_c, pc_t;
	PCC* pc_p(&pc_c);
	for(uint32_t i=0; i< pcs_.size(); i++)
	{
		pc_c.clear();
		pc_t.clear();

		pcl::copyPointCloud(*pcs_[i].pc, pc_c);
		for(uint32_t j=0; j<pc_c.points.size(); j++)
		{
			pc_c.points[j].r = colors[i][0];
			pc_c.points[j].g = colors[i][1];
			pc_c.points[j].b = colors[i][2];
		}

		if(i != id_base_)
		{
			pcl::transformPointCloud(pc_c, pc_t, eps[i] );
			std::cout << "***************" << std::endl << eps[i].matrix() << std::endl;
			pc_p = &pc_t;
		}

		pc += *pc_p;
	}

	if(pc.points.size() == 0)
	{
		std::cerr << "E: " << __FUNCTION__ << " pointcloud has no point"
				  << std::endl;
		return false;
	}
	return ( !pcl::io::savePCDFileBinary(out_fn, pc) );
}

bool MutilCalEval::save_merged_pointcloud(const std::string& out_fn,
		const std::vector<Eigen::Matrix4d>& eps)
{
	return save_merged_pc<Eigen::Matrix4d>(out_fn, eps);
}

bool MutilCalEval::save_merged_pointcloud(const std::string& out_fn,
		const std::vector<Eigen::Affine3d>& eps)
{
	return save_merged_pc<Eigen::Affine3d>(out_fn, eps);
}
bool MutilCalEval::save_merged_pointcloud(const std::string& out_fn)
{
	return save_merged_pointcloud(out_fn, eps_);
};


/**
 * @brief evaluate builtin optimization result
 *
 * @return
 */
double MutilCalEval::evaluate()
{
	return evaluate(eps_, id_base_);
}

double MutilCalEval::evaluate(const std::vector<Eigen::Affine3d>& eps,
							  uint32_t id_base)
{
	//return evaluate_eps<Eigen::Affine3d>(eps);
	std::vector<Eigen::Matrix4d> tfs;
	tfs.reserve( eps.size() );

	for(auto& it:eps)
	{
		tfs.push_back(it.matrix());
	}

	return evaluate(tfs, id_base);
}

double MutilCalEval::evaluate(const std::vector<Eigen::Matrix4d>& eps,
							  uint32_t id_base)
{
	double err(0.0);
	uint32_t num(0);

	if(eps.size() != pcs_.size() || pcs_.size() < 2 )
	{
		std::cerr << "E: evaluate error, no enough extrinsic parameters or pointclouds"
				  << std::endl;
		return -1;
	}

	if( id_base > eps.size() )
	{
		id_base = id_base_;
	}

	for(uint32_t i=0; i < pcs_.size(); i++ )
	{
		auto& pi = pcs_[i].planes;
		for(uint32_t j=i+1; j<pcs_.size(); j++)
		{
			auto& pj = pcs_[j].planes;

			Eigen::Matrix4d tf_i2j, tf_j2i;
			if(i == id_base)
			{
				tf_i2j = eps[j].inverse();
				tf_j2i = eps[j];
			}
			else if(j == id_base)
			{
				tf_i2j = eps[i];
				tf_j2i = eps[i].inverse();
			}
			else
			{
				tf_i2j = (eps[j].inverse())*eps[i];
				tf_j2i = (eps[i].inverse())*eps[j];
			}

			for(uint32_t k=0; k<plane_boxs_.size(); k++)
			{
				// skip invalid plane
				if(pi[k].point_num() == 0 || pj[k].point_num() == 0)
				{
					continue;
				}

				// forward projection, point(i) --> coordinate(j)
				err += (pj[k].coef.transpose()*tf_i2j*pi[k].points)
					   .cwiseAbs().sum();
				num += pi[k].point_num();

				// backward projection, point(j) --> coordinate(i)
				err += (pi[k].coef.transpose()*tf_j2i*pj[k].points)
					   .cwiseAbs().sum();
				num += pj[k].point_num();
			}
		}
	}
	// normalize
	return err/num;
}

void MutilCalEval::save_opt_report(std::string str_file)
{
	std::ofstream fn_report(str_file);
	fn_report << str_opt_report;
	fn_report.close();
}

/*****************************END OF FILE**************************************/




