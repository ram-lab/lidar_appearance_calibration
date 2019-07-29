/**
  ******************************************************************************
  * @file	cal_icp.h
  * @author	Nick.Liao
  * @version V1.0.0
  * @date	2017-11-28
  * @brief	cal_icp.h
  ******************************************************************************
  * @attention
  *
  * Copyright (C) 2017 Nick.Liao <simplelife_nick@hotmail.com>
  * Distributed under terms of the MIT license.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAL_ICP_H
#define __CAL_ICP_H

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include <iostream>
#include <memory>

#include <yaml-cpp/yaml.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pointmatcher/PointMatcher.h>

#include "utils.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class CalICP
{
public:
	CalICP(std::shared_ptr<lqh::utils::pm> pm = nullptr);
	CalICP(const std::string& fn_config,
		   std::shared_ptr<lqh::utils::pm> pm = nullptr);
	void add_pointcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr pc,
						bool is_base = false);
	void add_pointcloud(const pcl::PointCloud<pcl::PointXYZI>& pc,
						bool is_base = false);
	void calibrate(std::vector<Eigen::Affine3d>& eps_ini,
							std::vector<Eigen::Affine3d>& eps_icp,
							uint32_t id_base = INT_MAX);					
	void calibrate(std::vector<Eigen::Affine3d>& eps, uint32_t id_base = INT_MAX);
	void calibrate(uint32_t id_base = INT_MAX);
	bool get_parameters( std::vector<Eigen::Affine3d>& eps);

private:
	bool is_icp_default_;
	std::shared_ptr<lqh::utils::pm> pm_;
	std::shared_ptr<lqh::utils::pm::helper> pm_helper_;
	std::string fn_config_;
	uint32_t id_base_;
	std::vector<PointMatcher<float>::DataPoints> pcs_;
	std::vector<Eigen::Affine3d> eps_;
	std::vector<Eigen::MatrixXd> covar_;

	void pcl2dp(const pcl::PointCloud<pcl::PointXYZI>& pc,
                PointMatcher<float>::DataPoints& dp);

};



/* Exported constants --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

#endif /* !__CAL_ICP_H */

/*****************************END OF FILE**************************************/
