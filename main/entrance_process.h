//
// Created by ghm on 2021/10/25.
//

#ifndef CVR_LSE_ENTRANCE_PROCESS_H
#define CVR_LSE_ENTRANCE_PROCESS_H
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <thread>
#include <deque>
#include <string>

//
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/registration/icp.h>
#include <pcl/search/impl/search.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include "cvr_lse_process.h"
namespace cvr_lse {
	using namespace ground_segmentation;
	using namespace lse_map;
class EntranceProcess {
public:
	explicit EntranceProcess(const ros::NodeHandle &nh) : nh_(nh) {};

	~EntranceProcess() = default;

	void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);

	void PointLabelCallback(const cvr_lse::cloud_label::ConstPtr& label_msg);

	void LidarOdometryCallback(const nav_msgs::Odometry::ConstPtr& lidar_odo_msg);

	bool Start();

	void Stop();

	EntranceProcess(const EntranceProcess &) = delete;

	EntranceProcess &operator=(const EntranceProcess &) = delete;

	EntranceProcess(EntranceProcess &&) = delete;

	EntranceProcess &operator=(EntranceProcess &&) = delete;

private:
	void MainProcess();

	bool LoadParameterForCvrLse();

	bool FindCorrespondingInfo(CloudInfo& cloud_info);

	std::thread main_processer_;

	std::shared_ptr<CvrLseProcess> cvr_lse_processer_;

	volatile bool start_flag_ = false;

	CvrLseParameters cvr_lse_params_;

	ros::NodeHandle nh_;

	std::mutex cloud_lock_;

	std::mutex label_lock_;

	std::deque<cvr_lse::cloud_label> cloud_label_msg_;

	std::deque<sensor_msgs::PointCloud2> cloud_msg_;

#ifdef Ground
	ros::Publisher ground_pub_;
	ros::Publisher obstacle_pub_;

	void VisualizationGroundPoint(const pcl::PointCloud<PointXYZIRT>& cloud,
		const cvr_lse::cloud_label& label_info);
#else
#define VisualizationGroundPoint(cloud, labelInfo);
#endif
	};
}
#endif //CVR_LSE_ENTRANCE_PROCESS_H
