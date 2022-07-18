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
	ground_label_ =  nh_.advertise<cvr_lse::multi_cloud_label>(cloud_label_topic, 1);
#ifdef Ground
    std::string ground_topic;
    std::string obstacle_topic;
    nh_.param<std::string>("/ground_segmentation/ground_output_topic", ground_topic,
                           "/ground_segmentation/ground_cloud");
    nh_.param<std::string>("/ground_segmentation/obstacle_output_topic", obstacle_topic,
                           "/ground_segmentation/obstacle_cloud");
    ground_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(ground_topic, 1);
    obstacle_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(obstacle_topic, 1);
#endif
    start_flag_ = true;
    main_processer_ = std::thread(&EntranceGround::MainProcess, this);
    return true;
}

void EntranceGround::Stop() {
    start_flag_ = false;
    main_processer_.join();
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
    nh_.param<double>("ground_segmentation/line_search_angle", ground_segmentation_params_.line_search_angle,
	                  ground_segmentation_params_.line_search_angle);
	nh_.param<int>("ground_segmentation/n_threads", ground_segmentation_params_.n_threads,
	               ground_segmentation_params_.n_threads);
	// extrinsic parameters
    nh_.param<std::vector<float>>("ground_segmentation/extrinsic_trans_left",
        ground_segmentation_params_.extrinsic_trans_left, ground_segmentation_params_.extrinsic_trans_left);
    nh_.param<std::vector<float>>("ground_segmentation/extrinsic_rot_left",
        ground_segmentation_params_.extrinsic_rot_left, ground_segmentation_params_.extrinsic_rot_left);

    nh_.param<std::vector<float>>("ground_segmentation/extrinsic_trans_right",
        ground_segmentation_params_.extrinsic_trans_right, ground_segmentation_params_.extrinsic_trans_right);
    nh_.param<std::vector<float>>("ground_segmentation/extrinsic_rot_right",
        ground_segmentation_params_.extrinsic_rot_right, ground_segmentation_params_.extrinsic_rot_right);

    nh_.param<std::vector<float>>("ground_segmentation/extrinsic_trans_rear",
        ground_segmentation_params_.extrinsic_trans_rear, ground_segmentation_params_.extrinsic_trans_rear);
    nh_.param<std::vector<float>>("ground_segmentation/extrinsic_rot_rear",
        ground_segmentation_params_.extrinsic_rot_rear, ground_segmentation_params_.extrinsic_rot_rear);

    nh_.param<std::vector<float>>("ground_segmentation/sensor_height", ground_segmentation_params_.sensor_height,
        ground_segmentation_params_.sensor_height);

    nh_.param<float>("ground_segmentation/x_range_min", ground_segmentation_params_.x_range_min,
        ground_segmentation_params_.x_range_min);

    nh_.param<float>("ground_segmentation/x_range_max", ground_segmentation_params_.x_range_max,
        ground_segmentation_params_.x_range_max);

    nh_.param<float>("ground_segmentation/y_range_min", ground_segmentation_params_.y_range_min,
        ground_segmentation_params_.y_range_min);

    nh_.param<float>("ground_segmentation/y_range_max", ground_segmentation_params_.y_range_max,
        ground_segmentation_params_.y_range_max);

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

void EntranceGround::PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    std::lock_guard<std::mutex> lock(cloud_lock_);
    cloud_msg_.emplace_back(*cloud_msg);
    if (cloud_msg_.size() > 7U) { // size 7U
        cloud_msg_.pop_front();
    }
}

void EntranceGround::LeftPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    std::lock_guard<std::mutex> lock(left_cloud_lock_);
    left_cloud_msg_.emplace_back(*cloud_msg);
    if (left_cloud_msg_.size() > 7U) { // size 7U
        left_cloud_msg_.pop_front();
    }
}

void EntranceGround::RightPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    std::lock_guard<std::mutex> lock(right_cloud_lock_);
    right_cloud_msg_.emplace_back(*cloud_msg);
    if (right_cloud_msg_.size() > 7U) { // size 7U
        right_cloud_msg_.pop_front();
    }
}

void EntranceGround::RearPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    std::lock_guard<std::mutex> lock(rear_cloud_lock_);
    rear_cloud_msg_.emplace_back(*cloud_msg);
    if (rear_cloud_msg_.size() > 7U) { // size 7U
        rear_cloud_msg_.pop_front();
    }
}

bool EntranceGround::FindCorrespondingInfo(std::vector<pcl::PointCloud<PointXYZIRT>>& point_cloud_info,
    std_msgs::Header& header_info) {
    sensor_msgs::PointCloud2 sensor_point_left;
    sensor_msgs::PointCloud2 sensor_point_right;
    sensor_msgs::PointCloud2 sensor_point_rear;
    while (!cloud_msg_.empty()) {
        sensor_msgs::PointCloud2 point_cloud_main;
        {
            std::lock_guard<std::mutex> lock_main(cloud_lock_);
            std::swap(point_cloud_main, cloud_msg_.front());
            cloud_msg_.pop_front();
        }

        {
            std::lock_guard<std::mutex> lock_left(left_cloud_lock_);
            if(!FindCorrespondingBetweenTwoMsg(point_cloud_main, sensor_point_left, left_cloud_msg_)) {
                continue;
            }
        }

        {
            std::lock_guard<std::mutex> lock_right(right_cloud_lock_);
            if(!FindCorrespondingBetweenTwoMsg(point_cloud_main, sensor_point_right, right_cloud_msg_)) {
                continue;
            }
        }

        {
            std::lock_guard<std::mutex> lock_rear(rear_cloud_lock_);
            if(!FindCorrespondingBetweenTwoMsg(point_cloud_main, sensor_point_rear, rear_cloud_msg_)) {
                continue;
            }
        }

        point_cloud_info.resize(LidarSize);
        pcl::PointCloud<PointXYZIRT> cloud_main;
        pcl::fromROSMsg(point_cloud_main, cloud_main);
        point_cloud_info[Main] = cloud_main;

        pcl::PointCloud<PointXYZIRT> cloud_left;
        pcl::fromROSMsg(sensor_point_left, cloud_left);
        point_cloud_info[Left] = cloud_left;

        pcl::PointCloud<PointXYZIRT> cloud_right;
        pcl::fromROSMsg(sensor_point_right, cloud_right);
        point_cloud_info[Right] = cloud_right;

        pcl::PointCloud<PointXYZIRT> cloud_rear;
        pcl::fromROSMsg(sensor_point_rear, cloud_rear);
        point_cloud_info[Rear] = cloud_rear;

        header_info = point_cloud_main.header;
        return true;
    }
    return false;
}

bool EntranceGround::FindCorrespondingBetweenTwoMsg(const sensor_msgs::PointCloud2& point_cloud_main,
    sensor_msgs::PointCloud2& point_cloud_side, std::deque<sensor_msgs::PointCloud2>& point_cloud_msg) {
    while (!point_cloud_msg.empty()) {
        if (std::abs(point_cloud_main.header.stamp.toSec() - point_cloud_msg.front().header.stamp.toSec()) < 0.08) {
            point_cloud_side = point_cloud_msg.front();
            point_cloud_msg.pop_front();
            return true;
        } else if (point_cloud_main.header.stamp.toSec() > point_cloud_msg.front().header.stamp.toSec()) {
            point_cloud_msg.pop_front();
        } else {
            return false;
        }
    }
    return false;
}

void EntranceGround::MainProcess() {
    while (start_flag_) {
        if (cloud_msg_.empty() || left_cloud_msg_.empty() || right_cloud_msg_.empty() || rear_cloud_msg_.empty()) {
            usleep(50000);
            continue;
        }
        std::vector<pcl::PointCloud<PointXYZIRT>> point_cloud_info;
        std_msgs::Header header_info;
        if (!FindCorrespondingInfo(point_cloud_info, header_info)) {
            usleep(50000);
        }
        cvr_lse::multi_cloud_label::Ptr multi_label_ptr = boost::make_shared<cvr_lse::multi_cloud_label>();
        multi_label_ptr->header = header_info;
        ground_seg_->SegmentProcess(multi_label_ptr, point_cloud_info);
        ground_label_.publish(*multi_label_ptr);
        ground_seg_->Reset();
    }
}

#ifdef Ground
void EntranceGround::VisualizationGroundPoint(const pcl::PointCloud<PointXYZIRT>& cloud,
    const cvr_lse::multi_cloud_label& label_info) {
    pcl::PointCloud<PointXYZIRT> ground_cloud;
    pcl::PointCloud<PointXYZIRT> obstacle_cloud;
    ground_cloud.header = cloud.header;
    obstacle_cloud.header = cloud.header;
    for (size_t i = 0; i < cloud.size(); ++i) {
        if (label_info.main_label_cloud[i].label == 1U) {
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
	ros::init(argc, argv, "ground");
	ros::NodeHandle nh("~");
	ros::Subscriber point_cloud_sub_;
	ros::Subscriber left_point_cloud_sub_;
	ros::Subscriber right_point_cloud_sub_;
	ros::Subscriber rear_point_cloud_sub_;
	cvr_lse::EntranceGround enter_node(nh);
	static_cast<void>(enter_node.Start());
	std::string lidar_topic;
	std::string left_lidar_topic;
	std::string right_lidar_topic;
	std::string rear_lidar_topic;
	nh.param<std::string>("/ground_segmentation/lidar_topic", lidar_topic, "/apollo_sensor_hesai40_PointCloud2");
    nh.param<std::string>("/ground_segmentation/left_lidar_topic", left_lidar_topic,
                          "/apollo_driver_RoboSensor_front_left_rs16_PointCloud2");
    nh.param<std::string>("/ground_segmentation/right_lidar_topic", right_lidar_topic,
                          "/apollo_driver_RoboSensor_front_right_rs16_PointCloud2");
    nh.param<std::string>("/ground_segmentation/rear_lidar_topic", rear_lidar_topic,
                          "/apollo_driver_RoboSensor_rear_center_rs16_PointCloud2");
	point_cloud_sub_ = nh.subscribe(lidar_topic, 1, &cvr_lse::EntranceGround::PointCloudCallback,
                                    &enter_node);
    left_point_cloud_sub_ = nh.subscribe(left_lidar_topic, 1, &cvr_lse::EntranceGround::LeftPointCloudCallback,
                                    &enter_node);
    right_point_cloud_sub_ = nh.subscribe(right_lidar_topic, 1, &cvr_lse::EntranceGround::RightPointCloudCallback,
                                    &enter_node);
    rear_point_cloud_sub_ = nh.subscribe(rear_lidar_topic, 1, &cvr_lse::EntranceGround::RearPointCloudCallback,
                                    &enter_node);
	ros::spin();
    enter_node.Stop();
}