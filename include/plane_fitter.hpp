#ifndef __PLANE_FITTER_H
#define __PLANE_FITTER_H

// ROS
#include <ros/ros.h>
#include <ros/publisher.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <dynamic_reconfigure/server.h>

#include <eigen3/Eigen/Core>
#include <Eigen/Dense>

// PCL
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <yaml-cpp/yaml.h>

#include "ColorMap.h"

# define NPLANE 3 // do not change this parameter !!!
# define MAX_PLANE_NUMBER 3 // do not change this parameter !!!
# define MAX_DISTANCE 0.07
# define MIN_PERCENTAGE 0.02
# define COLOR_PC_WITH_ERROR false
# define B_EVALUATION true

double point2planedistnace(pcl::PointXYZ pt, pcl::ModelCoefficients coefficients)
{
    double f1 = fabs(coefficients.values[0]*pt.x+coefficients.values[1]*pt.y+
						coefficients.values[2]*pt.z+coefficients.values[3]);
    double f2 = sqrt(pow(coefficients.values[0],2)+pow(coefficients.values[1],2)+pow(coefficients.values[2],2));
    return f1/f2;
}

class planeFitter
{
public:
    planeFitter(ros::NodeHandle nh): _nh(nh)
	{
        initialize();
    }

    void initialize()
	{
        // Get node name
        _name = ros::this_node::getName();

        // Subscriber
        _subs = _nh.subscribe("/cloud_input",1,&planeFitter::pointCloudCb,this);

        // set parameters
        // ros::param::param<double>("~max_distance",_max_distance,0.05);
        // ros::param::param<double>("~min_percentage",_min_percentage,0.05);
        // ros::param::param<bool>("~color_pc_with_error",_color_pc_with_error,false);
		// ros::param::param<int>("~max_plane_num", _max_plane_num, 4);
		_max_distance = MAX_DISTANCE;
		_min_percentage = MIN_PERCENTAGE;
		_color_pc_with_error = COLOR_PC_WITH_ERROR;
		_max_plane_num = MAX_PLANE_NUMBER;		
		std::cout << _max_distance << " " << _min_percentage << " " << _max_plane_num << std::endl;
		
        // Create colors pallete
        createColors();

        // Inform initialized
        ROS_INFO("%s: node initialized.",_name.c_str());
    }


    void createColors()
	{
        uint8_t r = 0;
        uint8_t g = 0;
        uint8_t b = 0;
        for (int i=0;i<20;i++)
		{
            while (r<70 && g < 70 && b < 70)
			{
                r = rand()%(255);
                g = rand()%(255);
                b = rand()%(255);
            }
            Color c(r,g,b);
            r = 0;
            g = 0;
            b = 0;
            colors.push_back(c);
        }
    }    

    void pointCloudCb(const sensor_msgs::PointCloud2::ConstPtr &msg)
	{
        // Convert to pcl point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_msg(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg,*cloud_msg);
        ROS_DEBUG("%s: new ponitcloud (%i,%i)(%zu)",_name.c_str(),cloud_msg->width,cloud_msg->height,cloud_msg->size());
		std::cout << "/*************************************************/" << std::endl;

		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_max_plane(new pcl::PointCloud<pcl::PointXYZI>);
		Eigen::Vector3f P_origin = Eigen::Vector3f::Zero();
		std::vector<pcl::PointCloud<pcl::PointXYZ> > v_cloud_plane(NPLANE);        
		std::vector<Eigen::Vector3f> v_normal(NPLANE);
		std::vector<pcl::ModelCoefficients> v_coefficients(NPLANE);
        PlaneExtraction(cloud_msg, cloud_plane, cloud_max_plane, P_origin, v_cloud_plane, v_coefficients, v_normal);
    }

	void Process(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, 
		ros::Publisher &pub_inliers, 
		ros::Publisher &pub_inliers_max_plane,
		std::string &str_cfg_fold,
		std::string &str_yaml,
		std::string &str_cloud,
		std::string &str_data,
		Eigen::Matrix4f &tf_gt,
        bool b_yaml)
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_max_plane(new pcl::PointCloud<pcl::PointXYZI>);
		Eigen::Vector3f P_origin = Eigen::Vector3f::Zero();
		std::vector<pcl::PointCloud<pcl::PointXYZ> > v_cloud_plane(NPLANE);        
		std::vector<Eigen::Vector3f> v_normal(NPLANE);
		std::vector<pcl::ModelCoefficients> v_coefficients(NPLANE);
        PlaneExtraction(cloud_in, cloud_plane, cloud_max_plane, P_origin, v_cloud_plane, v_coefficients, v_normal);

        sensor_msgs::PointCloud2 cloud_plane_msg;
        pcl::toROSMsg(*cloud_plane, cloud_plane_msg);
        cloud_plane_msg.header.frame_id = "rslidar";
		cloud_plane_msg.header.stamp = ros::Time::now();
        pub_inliers.publish(cloud_plane_msg);		

		sensor_msgs::PointCloud2 cloud_max_plane_msg;
        pcl::toROSMsg(*cloud_max_plane, cloud_max_plane_msg);
        cloud_max_plane_msg.header.frame_id = "rslidar";
		cloud_max_plane_msg.header.stamp = ros::Time::now();
        pub_inliers_max_plane.publish(cloud_max_plane_msg);		
		
        // save coefficients and pcd name to a given yaml file
		if (b_yaml)
		{
			std::ofstream out_yaml(str_yaml.c_str());
			out_yaml << "cfg_fold:" << std::endl;
			out_yaml << "  - \"" << str_cfg_fold << "\"" << std::endl;	
			out_yaml << "cloud:" << std::endl;
			out_yaml << "  - \"" << str_cloud << "\"" << std::endl;
			out_yaml << "coef_plane:" << std::endl;
			out_yaml << "  - [" << v_coefficients[0].values[0] << ", " 
								<< v_coefficients[0].values[1] << ", "
								<< v_coefficients[0].values[2] << ", "
								<< v_coefficients[0].values[3] << "]" << std::endl;
			out_yaml << "  - [" << v_coefficients[1].values[0] << ", " 
								<< v_coefficients[1].values[1] << ", "
								<< v_coefficients[1].values[2] << ", "
								<< v_coefficients[1].values[3] << "]" << std::endl;
			out_yaml << "  - [" << v_coefficients[2].values[0] << ", " 
								<< v_coefficients[2].values[1] << ", "
								<< v_coefficients[2].values[2] << ", "
								<< v_coefficients[2].values[3] << "]" << std::endl;
			out_yaml << "normal_plane:" << std::endl;
			out_yaml << "  - [" << v_normal[0](0) << ", " 
								<< v_normal[0](1) << ", "
								<< v_normal[0](2) << "] " << std::endl;
			out_yaml << "  - [" << v_normal[1](0) << ", " 
								<< v_normal[1](1) << ", "
								<< v_normal[1](2) << "] " << std::endl;
			out_yaml << "  - [" << v_normal[2](0) << ", " 
								<< v_normal[2](1) << ", "
								<< v_normal[2](2) << "] " << std::endl;
			out_yaml << "planes:" << std::endl;
			out_yaml << "  - \"" << str_cfg_fold << "plane/" << str_data << "_plane_1.pcd\"" << std::endl
					 << "  - \"" << str_cfg_fold << "plane/" << str_data << "_plane_2.pcd\"" << std::endl
					 << "  - \"" << str_cfg_fold << "plane/" << str_data << "_plane_3.pcd\"" << std::endl;
			out_yaml << "origin:" << std::endl;
			out_yaml << "  - [" << P_origin(0) << ", " << P_origin(1) << ", " << P_origin(2) <<"]" << std::endl;
			out_yaml << "tf_gt:" << std::endl;
			for(uint32_t i = 0; i < 4; i++ )
			{
				out_yaml << " - [" << tf_gt(i,0) << ", " << tf_gt(i,1) << ", " << tf_gt(i,2) << ", " << tf_gt(i,3) << "]" << std::endl;
			}
			out_yaml.close();

			std::string str_pcd;

			str_pcd = str_cfg_fold + "plane/" + str_data + "_plane_1.pcd";
			pcl::io::savePCDFile(str_pcd, v_cloud_plane[0]);
			str_pcd = str_cfg_fold + "plane/" + str_data + "_plane_2.pcd";
			pcl::io::savePCDFile(str_pcd, v_cloud_plane[1]);
			str_pcd = str_cfg_fold + "plane/" + str_data + "_plane_3.pcd";
			pcl::io::savePCDFile(str_pcd, v_cloud_plane[2]);

			pcl::PointCloud<pcl::PointXYZI> plane_1, plane_2, plane_3, planes;			
			pcl::copyPointCloud<pcl::PointXYZ, pcl::PointXYZI>(v_cloud_plane[0], plane_1);
			for (auto &p : plane_1) p.intensity = 0;
			pcl::copyPointCloud<pcl::PointXYZ, pcl::PointXYZI>(v_cloud_plane[1], plane_2);
			for (auto &p : plane_2) p.intensity = 128;
			pcl::copyPointCloud<pcl::PointXYZ, pcl::PointXYZI>(v_cloud_plane[2], plane_3);
			for (auto &p : plane_3) p.intensity = 255;					
			planes += plane_1;
			planes += plane_2;
			planes += plane_3;
			str_pcd = str_cfg_fold + "plane/" + str_data + "_planes.pcd";
			pcl::io::savePCDFile(str_pcd, planes);			
		}		
	}

// 	TODO:
    void PlaneExtraction(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, 
						pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane,
						pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_max_plane,
						Eigen::Vector3f &P_origin,
						std::vector<pcl::PointCloud<pcl::PointXYZ> > &v_cloud_plane_order, 
                        std::vector<pcl::ModelCoefficients> &v_coefficients_order, 
						std::vector<Eigen::Vector3f> &v_normal_order)
    {
// STEP 1: initialize classes
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		*cloud = *cloud_in;
		
        pcl::ModelCoefficients coefficients;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
		
		// random seed: https://www.cnblogs.com/afarmer/archive/2011/05/01/2033715.html
		srand(12345);
        pcl::SACSegmentation<pcl::PointXYZ> seg(true);
        seg.setOptimizeCoefficients(true);
		// model explanation: http://docs.pointclouds.org/1.7.0/group__sample__consensus.html / https://blog.csdn.net/sdau20104555/article/details/40649101
		seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold(_max_distance); // m

        int original_size(cloud->height*cloud->width);
        int n_planes_extract(0);

// STEP 2: extract many planes from point cloud until the number of points < original_size*_min_percentage
		std::vector<pcl::PointCloud<pcl::PointXYZ> > v_cloud_plane;
		std::vector<pcl::ModelCoefficients> v_coefficients;
		
		std::cout << original_size << std::endl;
        while (cloud->height*cloud->width > original_size*_min_percentage)
		{
            // Fit a plane
            seg.setInputCloud(cloud);
            seg.segment(*inliers, coefficients); // inliers are the plane points
		
            // Iterate inliers
            double mean_error(0);
            double max_error(0);
            double min_error(100000);
            std::vector<double> err; // a vector to save d_error
            for (int i=0;i<inliers->indices.size();i++)
			{
                pcl::PointXYZ pt = cloud->points[inliers->indices[i]];

                double d = point2planedistnace(pt,coefficients);
                err.push_back(d);

                mean_error += d;
                if (d>max_error) max_error = d;
                if (d<min_error) min_error = d;
            }
            mean_error /= inliers->indices.size();
// 			std::cout << "Mean error: " << mean_error << " " << "Error: " << mean_error*inliers->indices.size() << std::endl;			

            // Compute Standard deviation
            ColorMap cm(min_error,max_error);
            double sigma(0);
            for (int i=0;i<inliers->indices.size();i++)
			{
                sigma += pow(err[i] - mean_error,2);

                // Get Point
                pcl::PointXYZ pt = cloud->points[inliers->indices[i]];
				pcl::PointXYZI pt_color;
                pt_color.x = pt.x;
                pt_color.y = pt.y;
                pt_color.z = pt.z;
				pt_color.intensity = (255 / 20.0) * n_planes_extract;
				cloud_plane->push_back(pt_color);
            }
            sigma = sqrt(sigma / inliers->indices.size());

            // Extract inliers
            extract.setInputCloud(cloud);
            extract.setIndices(inliers);
            pcl::PointCloud<pcl::PointXYZ> cloudF;
			
			extract.setNegative(false);
			extract.filter(cloudF);
			v_cloud_plane.push_back(cloudF);
			
// 			std::stringstream ss;
// 			ss << "/home/jjiao/catkin_ws/src/lidar_appearance_calibration/data/white_20190611/top_front/" << n_planes_extract << ".pcd";
// 			pcl::io::savePCDFile(ss.str(), cloudF);
// 			std::cout << cloudF.size() << std::endl;

            extract.setNegative(true);
            extract.filter(cloudF);
            cloud->swap(cloudF);			
			
			v_coefficients.push_back(coefficients);
            n_planes_extract++;
        }
      
// STEP 3: select the maximum three planes according to their size
        for (size_t i = 0; i < n_planes_extract-1; i++)
		{
			for (size_t j = i+1; j < n_planes_extract; j++)
			{
				if (v_cloud_plane[i].size() < v_cloud_plane[j].size())
				{
					pcl::PointCloud<pcl::PointXYZ> temp_cloud = v_cloud_plane[i];
					v_cloud_plane[i] = v_cloud_plane[j];
					v_cloud_plane[j] = temp_cloud;
					
					pcl::ModelCoefficients temp_coef = v_coefficients[i];
					v_coefficients[i] = v_coefficients[j];
					v_coefficients[j] = temp_coef;
				}
			}
		}

		std::vector<pcl::PointCloud<pcl::PointXYZ> > v_cloud_max_plane;
		for (size_t i = 0; i < std::min(_max_plane_num, n_planes_extract); i++)
		{
			v_cloud_max_plane.push_back(v_cloud_plane[i]);
			for (size_t j = 0; j < v_cloud_plane[i].size(); j++)
			{
				pcl::PointXYZI pt;
				pt.x = v_cloud_plane[i].points[j].x;
				pt.y = v_cloud_plane[i].points[j].y;
				pt.z = v_cloud_plane[i].points[j].z;
				pt.intensity = (255.0 / std::min(_max_plane_num, n_planes_extract)) * i;
				cloud_max_plane->push_back(pt);
			}
		}

// STEP 4: order the planes according to normal. This step is very important.
		Eigen::Vector3f coeff_a(v_coefficients[0].values[0], v_coefficients[1].values[0], v_coefficients[2].values[0]);
		Eigen::Vector3f coeff_b(v_coefficients[0].values[1], v_coefficients[1].values[1], v_coefficients[2].values[1]);
		Eigen::Vector3f coeff_c(v_coefficients[0].values[2], v_coefficients[1].values[2], v_coefficients[2].values[2]);
		Eigen::Vector3f coeff_d(v_coefficients[0].values[3], v_coefficients[1].values[3], v_coefficients[2].values[3]);
		
		Eigen::Matrix<float, 3, 4> C;
		C.col(0) = coeff_a;
		C.col(1) = coeff_b;
		C.col(2) = coeff_c;
		C.col(3) = coeff_d;
		Eigen::FullPivLU<Eigen::Matrix<float, 3, 4> > lu_decomp(C);
		Eigen::MatrixXf X = lu_decomp.kernel();
		Eigen::VectorXf x = X.col(0);
		if (x(3) == 0) 
        {
            x(3) = 1e-10;
        }
		P_origin(0) = x(0)/x(3);
		P_origin(1) = x(1)/x(3);
		P_origin(2) = x(2)/x(3);
		std::cout << "Intersaction: " << P_origin.transpose() << std::endl;

        // fix the sign of normal vectors
        std::vector<Eigen::Vector3f> v_normal(NPLANE);
		for (size_t i = 0; i < 2; i++)
		{
			for (size_t j = 0; j < 2; j++)
			{
				for (size_t k = 0; k < 2; k++)
				{
					Eigen::Vector3f temp_n1(coeff_a(0), coeff_b(0), coeff_c(0));
					temp_n1 = temp_n1 * pow(-1,i);
					Eigen::Vector3f temp_n2(coeff_a(1), coeff_b(1), coeff_c(1));
					temp_n2 = temp_n2 * pow(-1,j);
					Eigen::Vector3f temp_n3(coeff_a(2), coeff_b(2), coeff_c(2));
					temp_n3 = temp_n3 * pow(-1,k);
					Eigen::Matrix3f A;
					A.col(0) = temp_n1;
					A.col(1) = temp_n2;
					A.col(2) = temp_n3;
					Eigen::Vector3f u(-P_origin(0), -P_origin(1), -P_origin(2)+5); // TODO: some tricky modifications
					Eigen::Vector3f y = A.colPivHouseholderQr().solve(u);
					if ((y(0) >= 0) && (y(1) >= 0) && (y(2) >= 0))
					{
						v_normal[0] = temp_n1;
						v_normal[1] = temp_n2;
						v_normal[2] = temp_n3;
                        std::cout << "Fix the sign of normal vectors: " << std::endl;
						// std::cout << u.transpose() << std::endl;
						std::cout << temp_n1.transpose() << std::endl;
						std::cout << temp_n2.transpose() << std::endl;
						std::cout << temp_n3.transpose() << std::endl;
						// std::cout << y.transpose() << std::endl;
					}
				}
			}
		}

		// fix the order of normal vectors
		float max = -1;
		Eigen::Vector3f e_z = Eigen::Vector3f::Zero();
		e_z(2) = 1;
		int k = 0;
		for (size_t i = 0; i < 3; i++)
		{
			if (e_z.dot(v_normal[i]) > max)
			{
				max = e_z.dot(v_normal[i]);
				k = i;
			}
		}
		v_normal_order[2] = v_normal[k];
		v_coefficients_order[2] = v_coefficients[k];
		v_cloud_plane_order[2] = v_cloud_max_plane[k];
		
		v_normal.erase(v_normal.begin()+k);
		v_coefficients.erase(v_coefficients.begin()+k);
		v_cloud_max_plane.erase(v_cloud_max_plane.begin()+k);
		if (v_normal_order[2].dot(v_normal[0].cross(v_normal[1])) > 0)
		{
			v_normal_order[0] = v_normal[0];
			v_normal_order[1] = v_normal[1];
			v_coefficients_order[0] = v_coefficients[0];
			v_coefficients_order[1] = v_coefficients[1];
			v_cloud_plane_order[0] = v_cloud_max_plane[0];
			v_cloud_plane_order[1] = v_cloud_max_plane[1];
		}
		else
		{
			v_normal_order[0] = v_normal[1];
			v_normal_order[1] = v_normal[0];
			v_coefficients_order[0] = v_coefficients[1];
			v_coefficients_order[1] = v_coefficients[0];
			v_cloud_plane_order[0] = v_cloud_max_plane[1];
			v_cloud_plane_order[1] = v_cloud_max_plane[0];			
		}		
		std::cout << "Fix the order of normal vectors: " << std::endl;
        std::cout << v_normal_order[0].transpose() << std::endl; 
        std::cout << v_normal_order[1].transpose() << std::endl;
        std::cout << v_normal_order[2].transpose() << std::endl;        
    }

private:
    // Node
    ros::NodeHandle _nh;
    std::string _name;

    // Publishers
    ros::Publisher pub_inliers_;
    ros::Publisher pub_inliers_max_plane_;

    // Subscriber
    ros::Subscriber _subs;

    // Algorithm parameters
    double _min_percentage;
    double _max_distance;
    bool _color_pc_with_error;
	
	int _max_plane_num;
	double _check_plane_is_ground;

    // Colors
    std::vector<Color> colors;

};

#endif