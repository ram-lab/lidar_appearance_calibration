#include "plane_fitter.hpp"

Eigen::Matrix4f tf_ref_gt, tf_data_gt;
bool b_icp = false;

void ICPCallBack(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("%s", msg->data.c_str());
	b_icp = true;
}

int main(int argc,char** argv){

    // Initialize ROS
    ros::init(argc, argv, "planeFitter");
	
	if(argc < 3)
	{
		std::cerr << "E: no enough argument(s)" << std::endl
				  << "Usage: " << " select pointcloud2 or pcd, cfg.yaml" << std::endl;
		return 1;
	}
	
	std::string str_data(argv[1]);
    ros::NodeHandle nh("~");
	ros::Rate loop_rate(1);
	
	ros::Subscriber sub_icp = nh.subscribe("/contact/icp", 1000, ICPCallBack);
	ros::Publisher pub_plane = nh.advertise<std_msgs::String>("/contact/save_plane", 1000);
	
    planeFitter pf(nh);
	if (str_data == "pointcloud2")
	{
		while (ros::ok())
		{
			ros::spin();
			loop_rate.sleep();
		}
	} 
	else if (str_data == "pcd")
	{
		ros::Publisher pub_ref_cloud = nh.advertise< sensor_msgs::PointCloud2 >("/calib/ref_cloud", 10);
        ros::Publisher pub_ref_inliers = nh.advertise< sensor_msgs::PointCloud2 >("/calib/ref_plane", 10);
		ros::Publisher pub_ref_inliers_max_plane = nh.advertise<sensor_msgs::PointCloud2>("/calib/ref_max_plane", 10);
		ros::Publisher pub_data_cloud = nh.advertise< sensor_msgs::PointCloud2 >("/calib/data_cloud", 10);
        ros::Publisher pub_data_inliers = nh.advertise< sensor_msgs::PointCloud2 >("/calib/data_plane", 10);
		ros::Publisher pub_data_inliers_max_plane = nh.advertise<sensor_msgs::PointCloud2>("/calib/data_max_plane", 10);
		
		std::string fn(argv[2]);
		YAML::Node config = YAML::LoadFile(fn);
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr ref_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr data_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		
		if(!config["ref_cloud"] || !config["ref_cloud"].IsSequence())
		{
			std::cerr << "E: read yaml but no pointclouds" << std::endl;
			return 0;
		} else
		{
			if (pcl::io::loadPCDFile(config["ref_cloud"][1].as<std::string>(), *ref_cloud) == -1)
			{
				std::cout << "E: fail to read pcd file" << std::endl
						<< config["ref_cloud"][1].as<std::string>() << std::endl;
			} 
		}
		
		if(!config["data_cloud"] || !config["data_cloud"].IsSequence())
		{
			std::cerr << "E: read yaml but no pointclouds" << std::endl;
			return 0;
		} else
		{
			if (pcl::io::loadPCDFile(config["data_cloud"][1].as<std::string>(), *data_cloud) == -1)
			{
				std::cout << "E: fail to read pcd file" << std::endl
						<< config["data_cloud"][1].as<std::string>() << std::endl;
			} 		
		}
		
		if (config["tf_ref_gt"])
		{
			Eigen::Matrix4f tf;
			tf.setIdentity();
			for(uint32_t i=0; i<4; i++ )
			{
				for(uint32_t j=0; j<4; j++)
				{
					tf(i,j) = config["tf_ref_gt"][i][j].as<float>();
				}
			}
			tf_ref_gt = tf;	
		} else
		{
			tf_ref_gt.setIdentity();
		}
		
		if (config["tf_data_gt"])
		{
			Eigen::Matrix4f tf;
			tf.setIdentity();
			for(uint32_t i=0; i<4; i++ )
			{
				for(uint32_t j=0; j<4; j++)
				{
					tf(i,j) = config["tf_data_gt"][i][j].as<float>();
				}
			}
			tf_data_gt = tf;	
		} else
		{
			tf_data_gt.setIdentity();
		}					
		std::string str_cfg_fold = config["cfg_fold"][0].as<std::string>();		

		while (ros::ok())
		{
			ros::spinOnce();
			loop_rate.sleep();
			if (!b_icp) continue;
			std::string str_yaml, str_data;

			sensor_msgs::PointCloud2 ref_cloud_msg, data_cloud_msg;
			pcl::toROSMsg(*ref_cloud, ref_cloud_msg);
			ref_cloud_msg.header.frame_id = "rslidar";
			ref_cloud_msg.header.stamp = ros::Time::now();			
			
			pub_ref_cloud.publish(ref_cloud_msg);
			pcl::toROSMsg(*data_cloud, data_cloud_msg);
			data_cloud_msg.header.frame_id = "rslidar";
			data_cloud_msg.header.stamp = ros::Time::now();
			pub_data_cloud.publish(data_cloud_msg);			
		
			// ref
			std::cout << "/************* ref cloud ************" << std::endl;			
			std::string str_ref_cloud = config["ref_cloud"][1].as<std::string>();
			str_yaml = str_cfg_fold + "ref_cfg.yaml";
			str_data = "ref";
			pf.Process(ref_cloud, pub_ref_inliers, pub_ref_inliers_max_plane, str_cfg_fold, 
				str_yaml, str_ref_cloud, str_data, tf_ref_gt, true);

			// data
			std::cout << "/************* data cloud ************" << std::endl;
			std::string str_data_cloud = config["data_cloud"][1].as<std::string>();
			str_yaml = str_cfg_fold + "data_cfg.yaml";
			str_data = "data";
			pf.Process(data_cloud, pub_data_inliers, pub_data_inliers_max_plane, str_cfg_fold, 
				str_yaml, str_data_cloud, str_data, tf_data_gt, true);
	
			sleep(1);
			b_icp = false;
			std_msgs::String msg;
			std::stringstream ss;
			ss << "plane ...";
			msg.data = ss.str();
			pub_plane.publish(msg);
		}	
	}

    return 0;
}





