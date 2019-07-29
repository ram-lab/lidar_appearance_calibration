/**
  ******************************************************************************
  * @file   cal_icp.c
  * @author Nick.Liao
  * @version V1.0.0
  * @date   2017-11-28
  * @brief  cal_icp.c
  ******************************************************************************
  * @attention
  *
  * Copyright (C) 2017 Nick.Liao <simplelife_nick@hotmail.com>
  * Distributed under terms of the MIT license.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "cal_icp.h"

#include <fstream>

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
CalICP::CalICP (std::shared_ptr<lqh::utils::pm> pm):
	is_icp_default_(true),
	id_base_(0)
{
	pm_ = (pm == nullptr) ? std::make_shared<lqh::utils::pm>() : pm;
	pm_helper_ = std::make_shared<lqh::utils::pm::helper>();
}

CalICP::CalICP (const std::string& fn_config,
				std::shared_ptr<lqh::utils::pm> pm):
	is_icp_default_(false),
	fn_config_(fn_config),
	id_base_(0)
{
	pm_ = (pm == nullptr) ? std::make_shared<lqh::utils::pm>() : pm;
	pm_helper_ = std::make_shared<lqh::utils::pm::helper>();
}

void CalICP::add_pointcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr pc,
							bool is_base)
{
	add_pointcloud(*pc, is_base);
}

void CalICP::add_pointcloud(const pcl::PointCloud<pcl::PointXYZI>& pc,
							bool is_base)
{
	if(pc.points.size() == 0)
	{
		return;
	}

	pcs_.emplace_back();
	pcl2dp(pc, pcs_.back());
	

	if(is_base)
	{
		id_base_ = pcs_.size() - 1;
	}
}

void CalICP::calibrate(std::vector<Eigen::Affine3d>& eps_ini,
						std::vector<Eigen::Affine3d>& eps_icp,
					    uint32_t id_base )
{
	eps_ = eps_ini;
	calibrate(id_base);
	eps_icp = eps_;
}

void CalICP::calibrate(std::vector<Eigen::Affine3d>& eps,
					   uint32_t id_base )
{
	calibrate(id_base);
	eps = eps_;
}

void CalICP::calibrate(uint32_t id_base)
{
	if(pcs_.size() < 2)
	{
		eps_.clear();
		return;
	}

	if(id_base < pcs_.size())
	{
		id_base_ = id_base;
	}

	// eps_.resize( pcs_.size() );
	// eps_[id_base_].setIdentity();

    covar_.resize( pcs_.size() );

	auto handler = [this](uint32_t id)
	{
		PointMatcher<float>::ICP icp;

		if(is_icp_default_)
		{
			icp.setDefault();
		}
		else
		{
			std::ifstream ifs(fn_config_);
			if( ifs.good() )
			{
				icp.loadFromYaml(ifs);
				ifs.close();
			}
			else
			{
				icp.setDefault();
			}
		}

        try
        {
            PointMatcher<float>::TransformationParameters T = icp(pcs_[id], pcs_[id_base_], eps_[id].matrix().cast<float>());
			std::cout << "ICP result:" << std::endl << T.block<3,3>(0,0) << std::endl << T.block<3,1>(0,3).transpose() << std::endl;
            Eigen::Matrix4f tmp = T;
            eps_[id] = Eigen::Affine3d( tmp.cast<double>() );
            covar_[id] = (icp.errorMinimizer->getCovariance()).cast<double>();
        }
        catch (const PointMatcher<float>::ConvergenceError& e)
        {
            std::cout << "id: " << id << std::endl
                      << e.what() << std::endl;
        }

	};

	for(uint32_t i=0; i<pcs_.size(); i++)
	{
		if(i == id_base_)
		{
			continue;
		}

		pm_->post(*pm_helper_, boost::bind<void>(handler, i));
	}

	pm_->wait(*pm_helper_);
}

bool CalICP::get_parameters( std::vector<Eigen::Affine3d>& eps)
{
	if(pcs_.size() >2 &&  pcs_.size() == eps_.size() )
	{
		eps = eps_;
		return true;
	}
	return false;
}


void CalICP::pcl2dp(const pcl::PointCloud<pcl::PointXYZI>& pc,
                    PointMatcher<float>::DataPoints& dp)
{
	if(pc.size() == 0)
	{
		return;
	}
    dp.features.resize(4, pc.points.size());
    Eigen::Matrix<float, 1, Eigen::Dynamic> intensity(pc.points.size());

	for(uint32_t i=0; i<pc.points.size(); i++)
	{
		dp.features(0,i) = pc.points[i].x;
		dp.features(1,i) = pc.points[i].y;
		dp.features(2,i) = pc.points[i].z;
		dp.features(3,i) = 1.0;

		intensity(i) = pc.points[i].intensity;
	}
	dp.addDescriptor("intensity", intensity);
}


/*****************************END OF FILE**************************************/
