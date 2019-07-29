#include <iostream>
#include <iomanip>
#include <string>
#include <map>
#include <random>
#include <cmath>

#include <yaml-cpp/yaml.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>

#define N_POSE 2
#define P_A 5
#define P_B 5
#define NOISE_MEAN 0
#define NOISE_STD 0.1
#define B_YAML true
#define N_NOISY_POINTS 2000

std::default_random_engine generator;

float deg_to_rad(float r)
{
	return r/180.0*M_PI;
}

void TFGenerator(int n_pose, std::vector<Eigen::Matrix4f> &v_tf)
{
	Eigen::Matrix4f tf;
	
	tf << -0.8474,   -0.4537,    0.2757,    0.8766,
		0.3662,   -0.8755,   -0.3152,    0.4672,
		0.3844,   -0.1661,    0.9081,    1.0474,
			0,         0,         0,    1.0000;
	v_tf.push_back(tf);
	tf << 0.8620,    0.5044,    0.0495,    1.3785,
		-0.4906,    0.8549,   -0.1684,   -1.3929,
		-0.1273,    0.1209,    0.9845,    1.3020,
		0,         0,         0,    1.0000;
	v_tf.push_back(tf);
}

void normal_variable(float mean, float std, Eigen::Vector3f &noise)
{
	std::normal_distribution<float> distribution(mean, std);
	noise = Eigen::Vector3f(distribution(generator), distribution(generator), distribution(generator));
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "simulation_data_generation");
    ros::NodeHandle nh("~");
	ros::Rate loop_rate(1);	
	
	ros::Publisher pub_ref_cloud = nh.advertise<sensor_msgs::PointCloud2>("/simulation/ref_cloud", 10);
	ros::Publisher pub_data_cloud = nh.advertise<sensor_msgs::PointCloud2>("/simulation/data_cloud", 10);
	
	while (ros::ok())
	{	
		std::vector<float> v_alpha = {60, 70, 80, 90, 100, 110, 120};
// 		std::vector<float> v_alpha = {90};
		for (size_t i = 0; i < v_alpha.size(); i++)
		{
			// STEP 1: randomly select planes
			float alpha = deg_to_rad(v_alpha[i]);
			Eigen::Vector3f o(6.08168, 0.23877, -1.02524);
			Eigen::Vector3f l2(-0.7071, 0.7071, 0);
			Eigen::Vector3f l3(0, 0, 1);
			Eigen::AngleAxisf rv(alpha, Eigen::Vector3f(0, 0, 1));
			Eigen::Vector3f l1 = rv * l2;
			std::vector< Eigen::Vector3f > v_l = {l2, l3, l1, l2};

			// STEP 2: randomly generate plane points
			pcl::PointCloud<pcl::PointXYZI> ref_cloud, data_cloud;
			std::vector< pcl::PointCloud<pcl::PointXYZI> > v_plane;
			v_plane.resize(3);
			for (size_t j = 0; j < v_plane.size(); j++)
			{
				pcl::PointCloud<pcl::PointXYZI> &plane = v_plane[j];
				Eigen::Vector3f &e1 = v_l[j];
				Eigen::Vector3f &e2 = v_l[j+1];
				for (float a = 0; a < P_A; a += 0.1)
				{
					for (float b = 0; b < P_B; b += 0.1)
					{
						Eigen::Vector3f noise;
						normal_variable(NOISE_MEAN, NOISE_STD, noise);
						// std::cout << noise.transpose() << std::endl;
						Eigen::Vector3f l = a*e1 + b*e2 + o + noise;
						pcl::PointXYZI p;
						p.x = l[0]; p.y = l[1]; p.z = l[2]; 
						p.intensity = 0.0;
// 						p.intensity = int(255.0*(j+1)/v_plane.size());
						plane.push_back(p);
					}
				}
				ref_cloud += v_plane[j];
			}
			
			for (size_t j = 0; j < N_NOISY_POINTS; j++)
			{
				pcl::PointXYZI p;
				Eigen::Vector3f noise;
				normal_variable(0, sqrt(P_A*P_A + P_B*P_B), noise);
				p.x = noise[0]; p.y = noise[1]; p.z = noise[2]; p.intensity = 0;
				ref_cloud.push_back(p);
			}
			
			std::vector<Eigen::Matrix4f> v_tf; 
			TFGenerator(N_POSE, v_tf);
			
			for (size_t j = 0; j < v_tf.size(); j++)
			{
				pcl::transformPointCloud(ref_cloud, data_cloud, v_tf[j].inverse());
				std::stringstream ss;
				ss << "../data/simulation_tf_" << j << "_alpha_" << int(v_alpha[i]) << "/";
				std::string str_cfg_fold = ss.str();
				if (B_YAML)
				{
					std::string str_yaml = str_cfg_fold + "cfg.yaml";
					std::ofstream out_yaml(str_yaml);
					out_yaml << "ref_cloud:" << std::endl;
					out_yaml << "  - \"" << str_cfg_fold << "raw/ref.pcd" << "\"" << std::endl;	
					out_yaml << "  - \"" << str_cfg_fold << "raw/ref.pcd" << "\"" << std::endl;	
					
					out_yaml << "data_cloud:" << std::endl;
					out_yaml << "  - \"" << str_cfg_fold << "raw/data.pcd" << "\"" << std::endl;	
					out_yaml << "  - \"" << str_cfg_fold << "raw/data.pcd" << "\"" << std::endl;					
					
					out_yaml << "cfg_fold:" << std::endl;
					out_yaml << "  - \"" << str_cfg_fold << "\"" << std::endl;
					Eigen::Matrix4f tf;
					tf.setIdentity();
					out_yaml << "tf_ref_gt:" << std::endl;
					for(uint32_t i = 0; i < 4; i++ )
					{
						out_yaml << " - [" << tf(i,0) << ", " << tf(i,1) << ", " << tf(i,2) << ", " << tf(i,3) << "]" << std::endl;
					}					
					tf = v_tf[j];
					out_yaml << "tf_data_gt:" << std::endl;
					for(uint32_t i = 0; i < 4; i++ )
					{
						out_yaml << " - [" << tf(i,0) << ", " << tf(i,1) << ", " << tf(i,2) << ", " << tf(i,3) << "]" << std::endl;
					}						
					out_yaml.close();
					
					std::string str_pcd;
					str_pcd = str_cfg_fold + "raw/ref.pcd";
					pcl::io::savePCDFile(str_pcd, ref_cloud);
					str_pcd = str_cfg_fold + "raw/data.pcd";
					pcl::io::savePCDFile(str_pcd, data_cloud);
// 					std::cout << j << std::endl;
				}

				sensor_msgs::PointCloud2 msg_cloud;
					
				pcl::toROSMsg(ref_cloud, msg_cloud);
				msg_cloud.header.frame_id = "rslidar";
				msg_cloud.header.stamp = ros::Time::now();
				pub_ref_cloud.publish(msg_cloud);
				
				pcl::toROSMsg(data_cloud, msg_cloud);
				msg_cloud.header.frame_id = "rslidar";
				msg_cloud.header.stamp = ros::Time::now();
				pub_data_cloud.publish(msg_cloud);
				
				static tf::TransformBroadcaster br;
				tf::Transform transform;
				transform.setOrigin(tf::Vector3(v_tf[j](0, 3), v_tf[j](1, 3), v_tf[j](2, 3)));
				
				Eigen::Matrix3f R = v_tf[j].topLeftCorner(3, 3);
				Eigen::Quaternionf q(R);
				tf::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());
				transform.setRotation(tf_q);
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "rslidar", "rslidar_2"));				
				
			}
		}	
		loop_rate.sleep();
	}
	return 0;
}


