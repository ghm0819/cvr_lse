//
// Created by ghm on 2021/12/9.
//

#include "entrance_ground.h"

namespace cvr_lse {
bool EntranceGround::Start() {
	// for segmentation
	if (!LoadParameterForGroundSegmentation()) {
		ROS_ERROR("params load error!");
		return false;
	}

	ground_seg_ = std::make_shared<GroundSegmentation>(ground_segmentation_params_);
	std::string cloud_label_topic;
	nh_.param<std::string>("/ground_segmentation/cloud_label_topic", cloud_label_topic,
		"/ground_segmentation/cloud_label");
	ground_label_ =  nh_.advertise<cvr_lse::cloud_label>(cloud_label_topic, 1);
}

bool EntranceGround::LoadParameterForGroundSegmentation() {
	nh_.param<int>("ground_segmentation/n_bins", ground_segmentation_params_.n_bins,
	               ground_segmentation_params_.n_bins);
	nh_.param<int>("ground_segmentation/n_segments", ground_segmentation_params_.n_segments,
	               ground_segmentation_params_.n_segments);
	nh_.param<double>("ground_segmentation/max_dist_to_line", ground_segmentation_params_.max_dist_to_line,
	                  ground_segmentation_params_.max_dist_to_line);
	nh_.param<double>("ground_segmentation/max_slope", ground_segmentation_params_.max_slope,
	                  ground_segmentation_params_.max_slope);
	nh_.param<double>("ground_segmentation/long_threshold", ground_segmentation_params_.long_threshold,
	                  ground_segmentation_params_.long_threshold);
	nh_.param<double>("ground_segmentation/max_long_height", ground_segmentation_params_.max_long_height,
	                  ground_segmentation_params_.max_long_height);
	nh_.param<double>("ground_segmentation/max_start_height", ground_segmentation_params_.max_start_height,
	                  ground_segmentation_params_.max_start_height);
	nh_.param<double>("ground_segmentation/sensor_height", ground_segmentation_params_.sensor_height,
	                  ground_segmentation_params_.sensor_height);
	nh_.param<double>("ground_segmentation/line_search_angle", ground_segmentation_params_.line_search_angle,
	                  ground_segmentation_params_.line_search_angle);
	nh_.param<int>("ground_segmentation/n_threads", ground_segmentation_params_.n_threads,
	               ground_segmentation_params_.n_threads);
	// Params that need to be squared.
	double r_min, r_max, max_fit_error;
	if (nh_.getParam("ground_segmentation/r_min", r_min)) {
		ground_segmentation_params_.r_min_square = r_min * r_min;
	}
	if (nh_.getParam("ground_segmentation/r_max", r_max)) {
		ground_segmentation_params_.r_max_square = r_max * r_max;
	}
	if (nh_.getParam("ground_segmentation/max_fit_error", max_fit_error)) {
		ground_segmentation_params_.max_error_square = max_fit_error * max_fit_error;
	}
	return true;
}

void EntranceGround::PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloudMsg) {
	cvr_lse::cloud_label::Ptr label_ptr = boost::make_shared<cvr_lse::cloud_label>();
	label_ptr->header = cloudMsg->header;
	pcl::PointCloud<PointXYZIRT> cloud;
	pcl::fromROSMsg(*cloudMsg, cloud);
	ground_seg_->SegmentProcess(cloud, label_ptr);
	ground_label_.publish(*label_ptr);
	ground_seg_->Reset();
}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "ground");
	ros::NodeHandle nh("~");
	ros::Subscriber point_cloud_sub_;
	cvr_lse::EntranceGround enter_node(nh);
	enter_node.Start();
	std::string lidar_topic;
	nh.param<std::string>("/ground_segmentation/lidar_topic", lidar_topic, "/hesai40p_points_xyzirt");
	point_cloud_sub_ = nh.subscribe(lidar_topic, 1, &cvr_lse::EntranceGround::PointCloudCallback,
		&enter_node);
	ros::spin();
}