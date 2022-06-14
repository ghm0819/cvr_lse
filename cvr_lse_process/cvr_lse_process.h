//
// Created by ghm on 2021/10/25.
//

#ifndef CVR_LSE_PROCESS_H
#define CVR_LSE_PROCESS_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <thread>
#include <deque>
#include <string>

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
#include <utility>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include "odometry.h"
#include "ground_segmentation.h"
#include <visualization_msgs/MarkerArray.h>

#include "post_process.h"

namespace cvr_lse {
using CloudInfo = std::pair<pcl::PointCloud<PointXYZIRT>, cvr_lse::cloud_label>;
namespace lse_map {
class CvrLseProcess {
public:
	CvrLseProcess(const ros::NodeHandle &nh, CvrLseParameters params) :
			nh_(nh),
			param_(std::move(params)) {
		InitialTransformationInfo();
		InitialThresholdInfo();
		obstacle_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("obstacle", 1);
		map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
		convex_Marker_Pub_ = nh_.advertise<visualization_msgs::MarkerArray>("convex", 1);
	};

	~CvrLseProcess() = default;

	void Process(const CloudInfo &cloud_info, const PoseInfo &pose_info);

private:
	void InitCvrLse();

	void InitialTransformationInfo();

	void TransformFromOdomToVeh(const PoseInfo& cur_pose,
		std::vector<std::vector<cv::Point2f>>& find_contour);

	void InitialThresholdInfo();

	PoseInfo TranformFromVehicleToLidar(const PoseInfo& vehPose);

	PoseInfo TranformFromLidarToVehicle(const PoseInfo& lidar_pose);

	void QuaternionToEuler(const Eigen::Quaterniond& q_info, double& roll, double& pitch, double& yaw);

	grid_map::Position ObtainCurrentMapPosition(const PoseInfo& vehPose) const;

	void PreProcessPointCloud(const CloudInfo& point_a, ScanAll& point_b, ScanAll& scan_ground_info);

	size_t ObtainIndexOfPoint(const PointXYZIRT& point) const;

	void UpdateRay(const ScanPoint& point);

	void UpdateInvalidScanPoint(const double angleInfo);

	void UpdateStitchPolygon(const ScanPoint& pointLast, const ScanPoint& pointCurrent);

	void UpdateGridMap(ScanAll& scan_obstacle_Info);

	void PublishGridMap(const grid_map::GridMap& submap);

	void PublishContours(const std::vector<std::vector<cv::Point2f>>& contours);

	void TranformScanPointToOdom(ScanPoint& point);

	ros::NodeHandle nh_;

	CvrLseParameters param_;

	std::vector<std::string> layer_{"prob", "max_height", "min_height", "mean_height", "num"}; // map layer

	std::shared_ptr<grid_map::GridMap> map_;

	std::shared_ptr<postprocess::PostProcess> post_process_;

	CloudInfo cloud_info_;

	// pose info
	PoseInfo cur_lidar_pos_;
	PoseInfo last_lidar_pos_;
	PoseInfo cur_veh_pos_;
	PoseInfo last_veh_pos_;
	grid_map::Position map_position_;
	volatile bool initial_flag_ = true;

	// grid map info
	float occupancy_threshold_{};
	float free_threshold_{};
	float occupancy_factor_{};
	float free_factor_{};

	// ext parameters
	Eigen::Quaterniond ext_rot_lidar_veh_;
	Eigen::Vector3d ext_trans_lidar_veh_;
	Eigen::Isometry3d ext_transform_matrix_lidar_veh_ = Eigen::Isometry3d::Identity();
	Eigen::Quaterniond ext_rot_veh_lidar_;
	Eigen::Vector3d ext_trans_veh_lidar_;
	Eigen::Isometry3d ext_transform_matrix_veh_lidar_ = Eigen::Isometry3d::Identity();
	// visualization
	ros::Publisher convex_Marker_Pub_;
	ros::Publisher obstacle_pub_;
	ros::Publisher map_pub_;

	Eigen::MatrixXi update_flag_; // 0 unknown, 1 free, 2 occupancy
};
}
}
#endif //CVR_LSE_PROCESS_H
