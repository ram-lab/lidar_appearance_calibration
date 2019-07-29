//
// Created by hyye on 3/16/18.
//

#ifndef COMMON_ROS_H_
#define COMMON_ROS_H_

#include <ros/ros.h>
#include <pointmatcher/Functions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

template <typename PointT>
/**
 * @brief Publish pcl::PointCloud as a ros topic
 * 
 * @param publisher ...
 * @param cloud ...
 * @param stamp ...
 * @param frame_id ...
 * @return void
 */
inline void PublishCloudMsg(ros::Publisher& publisher,
                            const pcl::PointCloud<PointT>& cloud,
                            const ros::Time& stamp,
                            std::string frame_id) 
{
	sensor_msgs::PointCloud2 msg;
	pcl::toROSMsg(cloud, msg);
	msg.header.stamp = stamp;
	msg.header.frame_id = frame_id;
	publisher.publish(msg);
}

#endif 