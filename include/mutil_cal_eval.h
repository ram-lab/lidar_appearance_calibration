/**
  ******************************************************************************
  * @file	mutil_cal_eval.h
  * @author	Nick.Liao
  * @version V1.0.0
  * @date	2017-11-23
  * @brief	mutil_cal_eval.h
  ******************************************************************************
  * @attention
  *
  * Copyright (C) 2017 Nick.Liao <simplelife_nick@hotmail.com>
  * Distributed under terms of the MIT license.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MUTIL_CAL_EVAL_H
#define __MUTIL_CAL_EVAL_H

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include <string>
#include <unordered_map>
#include <boost/shared_ptr.hpp>

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <yaml-cpp/yaml.h>

#include "calib_pointcloud.h"
#include "utils.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class MutilCalEval
{
private:
	// types
	struct plane
	{
		Eigen::Vector4d coef;
		Eigen::Vector3d normal;
		Eigen::Matrix4Xd points;
		std::vector<int> indices;
		double p2p_err_sum;

		inline uint32_t point_num()
		{
			return points.cols();
		}

		/**
		 * @brief calculate the sum of all point-to-plane distance
		 */
		void calculate_error()
		{
			// we assume the coefficients are normalized
			// coef/coef.head(3).norm()
			p2p_err_sum = (coef.transpose()*points).cwiseAbs().sum();
		}
	};

	struct pointcloud
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr pc;
		std::vector<plane> planes;
	};

	struct plane_box_axis_limit
	{
		double min;
		double max;
		std::string axis;
		bool is_ground; // has z-axis

		plane_box_axis_limit(double a, double b, std::string s):
			min(a),max(b),axis(s),is_ground(false) {};
	};
	using plane_box_limit = std::vector<plane_box_axis_limit>;

	struct PlanePlaneError;


	const uint32_t NUMBER_PRECISION = 10;

	/**
	 * @brief all input pointclouds
	 */
	std::vector<pointcloud> pcs_;
	/**
	 * @brief extrinsic parameters, n pointclouds(one is base), n-1 tf
	 */
	std::vector<Eigen::Affine3d> eps_;
	/**
	 * @brief have n plane box, every box have 1-3 axis limit
	 */
	std::vector<plane_box_limit> plane_boxs_;

	/**
	 * @brief multithreadpool map service
	 */
	std::shared_ptr<lqh::utils::pm> pm_;
	/**
	 * @brief multithreadpool map service helper
	 */
	lqh::utils::pm::helper pm_helper_;

	uint32_t id_base_;
	double calibration_error_;
	bool is_valid_;

	uint32_t plane_fit_points_min_num_;
	double plane_fit_distance_threshold_;
	double ground_offset_;



	void config(const YAML::Node& nd);
	void transform_limit_box(const Eigen::Matrix4d& tf, const plane_box_limit& in,
							 plane_box_limit& out);
	void fit_plane(uint32_t id_pc, uint32_t id_plane );
	bool save_pointcloud_csv(const std::string& fn,
							 const pcl::PointCloud<pcl::PointXYZI>& pc);
	bool save_pointcloud_csv(const std::string& fn, uint32_t id_pc);
	bool save_plane_csv(const std::string& fn,
						uint32_t id_pc, uint32_t id_plane);
	bool save_plane_csv(const std::string& fn, const plane& pl);

	template <typename T>
	bool save_merged_pc(const std::string& out_fn,
						const std::vector<T>& eps);

public:
	MutilCalEval();
	MutilCalEval(const std::string& fn_config,
				 std::shared_ptr<lqh::utils::pm> pm = nullptr);
	MutilCalEval(const YAML::Node& nd,
				 std::shared_ptr<lqh::utils::pm> pm = nullptr);

	operator bool() const;
	void 	 add_pm(std::shared_ptr<lqh::utils::pm> p);
	uint32_t add_pointcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, bool is_wait=true);
	void	 add_pointclouds(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> pcs);
	bool   set_base_id(uint32_t id_base);
	bool   set_parameters(const std::vector<Eigen::Affine3d>& ep);
	double get_base_error();
	double get_calibration_error();
	bool   get_parameters(std::vector<Eigen::Affine3d>& ep);
	double calibrate(uint32_t id_base = INT_MAX, bool is_verbose = true);
	double calibrate(const std::vector<Eigen::Matrix4d>& eps_init,
					 uint32_t id_base = INT_MAX, bool is_verbose = true);
	double calibrate(const std::vector<Eigen::Affine3d>& eps_init,
					 uint32_t id_base = INT_MAX, bool is_verbose = true);

	uint32_t save_segmented_pointcloud(const std::string& out_dir,
									   const std::string& prefix = "segmented_" );
	bool save_merged_pointcloud(const std::string& out_fn);
	bool save_merged_pointcloud(const std::string& out_fn,
								const std::vector<Eigen::Matrix4d>& eps);
	bool save_merged_pointcloud(const std::string& out_fn,
								const std::vector<Eigen::Affine3d>& eps);

	double evaluate();
	double evaluate(const std::vector<Eigen::Affine3d>& eps,
					uint32_t id_base = INT_MAX);
	double evaluate(const std::vector<Eigen::Matrix4d>& eps,
					uint32_t id_base = INT_MAX);
	
	void add_pointcloud(const CalibPointCloud &cpc);
	
	void save_opt_report(const std::string str_file);
	
	std::string str_opt_report;
};


/* Exported constants --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

#endif /* !__MUTIL_CAL_EVAL_H */

/*****************************END OF FILE**************************************/
