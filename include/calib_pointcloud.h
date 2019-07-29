#ifndef _CALIB_POINTCLOUD_H
#define _CALIB_POINTCLOUD_H

#include <iostream>

#include <eigen3/Eigen/Dense>

#include <pcl/io/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class CalibPointCloud
{
public:
	
	CalibPointCloud()
	{
// 		v_plane_.reserve(3);
// 		v_coeff_.reserve(3);
// 		v_normal_.reserve(3);
	}
	friend std::ostream& operator<<(std::ostream& os, const CalibPointCloud& cpc);
	
	std::vector<pcl::PointCloud<pcl::PointXYZI> > v_plane_;
	std::vector<Eigen::Vector4f> v_coeff_;
	std::vector<Eigen::Vector3f> v_normal_;
	pcl::PointCloud<pcl::PointXYZI> cloud_;
	Eigen::Matrix4f tf_gt_;
	Eigen::Matrix4f tf_ini_;
	Eigen::Vector3f origin_;
};

std::ostream& operator<<(std::ostream& os, const CalibPointCloud& cpc) 
{
	os << "CalibPointCloud: " << std::endl;
	os << "Coefficients: a, b, c, d*****************"  << std::endl;
	for (size_t i = 0; i < cpc.v_coeff_.size(); i++)
	{
		os << cpc.v_coeff_[i].transpose() << std::endl;
	}	
	os << "Normal: nx, ny, nz*****************"  << std::endl;
	for (size_t i = 0; i < cpc.v_normal_.size(); i++)
	{
		os << cpc.v_normal_[i].transpose() << std::endl;
	}
	os << "Origin: x, y, z *****************" << std::endl;
	os << cpc.origin_.transpose() << std::endl;
	return os;
}

#endif