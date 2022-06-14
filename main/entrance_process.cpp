//
// Created by ghm on 2021/10/25.
//

#include "entrance_process.h"
#include <unistd.h>

namespace cvr_lse {
bool EntranceProcess::Start() {
	if (!LoadParameterForCvrLse()) {
		ROS_ERROR("param load error");
		return false;
	}

	cvr_lse_processer_ = std::make_shared<CvrLseProcess>(nh_, cvr_lse_params_);
#ifdef Ground
	std::string ground_topic;
	std::string obstacle_topic;
	nh_.param<std::string>("/cvr_lse/ground_output_topic", ground_topic, "/cvr_lse/ground_cloud");
	nh_.param<std::string>("/cvr_lse/obstacle_output_topic", obstacle_topic,
		"/cvr_lse/obstacle_cloud");
	ground_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(ground_topic, 1);
	obstacle_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(obstacle_topic, 1);
#endif
	start_flag_ = true;
	main_processer_ = std::thread(&EntranceProcess::MainProcess, this);
}

void EntranceProcess::Stop() {
	start_flag_ = false;
	main_processer_.join();
}

bool EntranceProcess::LoadParameterForCvrLse()
{
	nh_.param<double>("cvr_lse/offset_x", cvr_lse_params_.offset_x,
	                  cvr_lse_params_.offset_x);
	nh_.param<double>("cvr_lse/offset_y", cvr_lse_params_.offset_y,
	                  cvr_lse_params_.offset_y);
	nh_.param<double>("cvr_lse/map_width", cvr_lse_params_.map_width,
	                  cvr_lse_params_.map_width);
	nh_.param<double>("cvr_lse/map_height", cvr_lse_params_.map_height,
	                  cvr_lse_params_.map_height);
	nh_.param<double>("cvr_lse/map_output_width", cvr_lse_params_.map_output_width,
	                  cvr_lse_params_.map_output_width);
	nh_.param<double>("cvr_lse/map_output_height", cvr_lse_params_.map_output_height,
	                  cvr_lse_params_.map_output_height);
	nh_.param<double>("cvr_lse/map_resolution", cvr_lse_params_.map_resolution,
	                  cvr_lse_params_.map_resolution);
	nh_.param<double>("cvr_lse/occupancy_factor", cvr_lse_params_.occupancy_factor,
	                  cvr_lse_params_.occupancy_factor);
	nh_.param<double>("cvr_lse/free_factor", cvr_lse_params_.free_factor,
	                  cvr_lse_params_.free_factor);
	nh_.param<double>("cvr_lse/occupancy_threshold", cvr_lse_params_.occupancy_threshold,
	                  cvr_lse_params_.occupancy_threshold);
	nh_.param<double>("cvr_lse/free_threshold", cvr_lse_params_.free_threshold,
	                  cvr_lse_params_.free_threshold);
	nh_.param<double>("cvr_lse/max_odd_threshold", cvr_lse_params_.max_odd_threshold,
	                  cvr_lse_params_.max_odd_threshold);
	nh_.param<double>("cvr_lse/min_odd_threshold", cvr_lse_params_.min_odd_threshold,
	                  cvr_lse_params_.min_odd_threshold);
	nh_.param<double>("cvr_lse/max_distance", cvr_lse_params_.max_distance,
	                  cvr_lse_params_.max_distance);
	nh_.param<std::vector<double>>("cvr_lse/extrinsic_trans", cvr_lse_params_.extTran,
	                               cvr_lse_params_.extTran);
	nh_.param<std::vector<double>>("cvr_lse/extrinsic_rot", cvr_lse_params_.extRot,
	                               cvr_lse_params_.extRot);
	return true;
}

void EntranceProcess::LidarOdometryCallback(const nav_msgs::Odometry::ConstPtr &lidar_odo_msg) {
	auto &odometry = odometry::Odometry::GetOdometry();
	odometry.SetNewOdometryMsg(lidar_odo_msg);
}

void EntranceProcess::PointLabelCallback(const cvr_lse::cloud_label::ConstPtr &label_msg) {
	std::lock_guard<std::mutex> lock(label_lock_);
	cloud_label_msg_.emplace_back(*label_msg);
	if (cloud_label_msg_.size() > 7U) { // size 7U
		cloud_label_msg_.pop_front();
	}
}

void EntranceProcess::PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
	std::lock_guard<std::mutex> lock(cloud_lock_);
	cloud_msg_.emplace_back(*cloud_msg);
	if (cloud_msg_.size() > 7U) { // size 7U
		cloud_msg_.pop_front();
	}
}

bool EntranceProcess::FindCorrespondingInfo(CloudInfo& cloud_info) {
	while ((!cloud_msg_.empty()) && (!cloud_label_msg_.empty())) {
		if (cloud_msg_.front().header.stamp.toSec() == cloud_label_msg_.front().header.stamp.toSec()) {
			pcl::PointCloud<PointXYZIRT> cloud;
			pcl::fromROSMsg(cloud_msg_.front(), cloud);
			cloud_info = std::make_pair(cloud, cloud_label_msg_.front());
			cloud_msg_.pop_front();
			cloud_label_msg_.pop_front();
			return true;
		} else if (cloud_msg_.front().header.stamp.toSec() > cloud_label_msg_.front().header.stamp.toSec()) {
			cloud_label_msg_.pop_front();
		} else {
			cloud_msg_.pop_front();
		}
	}
	return false;
}

void EntranceProcess::MainProcess() {
	auto &odometry = odometry::Odometry::GetOdometry();
	while (start_flag_) {
		if (cloud_msg_.empty() || cloud_label_msg_.empty()) {
			usleep(50000);
			continue;
		}
		CloudInfo cloud_info;
		{
			std::lock_guard<std::mutex> lock1(cloud_lock_);
			std::lock_guard<std::mutex> lock2(label_lock_);
			if (!FindCorrespondingInfo(cloud_info)) {
				usleep(50000);
				continue;
			}
		}
		PoseInfo cur_pose;
		if (odometry.GetPoseInfoFromTimestamp(static_cast<double>(cloud_info.first.header.stamp) / 1e6,
			cur_pose)) {
			if (cloud_info.first.size() != cloud_info.second.height.size()) {
				continue;
			}
			cvr_lse_processer_->Process(cloud_info, cur_pose);
			VisualizationGroundPoint(cloud_info.first, cloud_info.second);
		}
	}
}

#ifdef Ground
void EntranceProcess::VisualizationGroundPoint(const pcl::PointCloud<PointXYZIRT>& cloud,
	const cvr_lse::cloud_label& label_info) {
	pcl::PointCloud<PointXYZIRT> ground_cloud;
	pcl::PointCloud<PointXYZIRT> obstacle_cloud;
	ground_cloud.header = cloud.header;
	obstacle_cloud.header = cloud.header;
	for (size_t i = 0; i < cloud.size(); ++i) {
		if (label_info.label[i] == 1U) {
			ground_cloud.push_back(cloud[i]);
		} else {
			obstacle_cloud.push_back(cloud[i]);
		}
	}
	sensor_msgs::PointCloud2 temp_cloud;
	pcl::toROSMsg(ground_cloud, temp_cloud);
	ground_pub_.publish(temp_cloud);
	pcl::toROSMsg(obstacle_cloud, temp_cloud);
	obstacle_pub_.publish(temp_cloud);
}
#endif
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "cvr_lse");
	ros::NodeHandle nh("~");

	ros::Subscriber point_cloud_sub_;
	ros::Subscriber point_label_sub_;
	ros::Subscriber lidar_odometry_sub_;
	cvr_lse::EntranceProcess enter_node(nh);
	enter_node.Start();
	std::string lidar_topic;
	std::string cloud_label_topic;
	std::string lidar_odom_topic;
	nh.param<std::string>("/cvr_lse/lidar_topic", lidar_topic, "/hesai40p_points_xyzirt");
	nh.param<std::string>("/cvr_lse/cloud_label_topic", cloud_label_topic, "/ground_segmentation/cloud_label");
	nh.param<std::string>("/cvr_lse/lidar_odom_topic", lidar_odom_topic, "/odometry/imu");
	point_cloud_sub_ = nh.subscribe(lidar_topic, 1, &cvr_lse::EntranceProcess::PointCloudCallback, &enter_node);
	point_label_sub_ = nh.subscribe(cloud_label_topic, 1, &cvr_lse::EntranceProcess::PointLabelCallback, &enter_node);
	lidar_odometry_sub_ = nh.subscribe(lidar_odom_topic, 1, &cvr_lse::EntranceProcess::LidarOdometryCallback,
		&enter_node);
	ros::spin();
	enter_node.Stop();
}