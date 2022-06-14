//
// Created by ghm on 2021/10/25.
//

#include "odometry.h"
namespace cvr_lse {
namespace odometry {
void Odometry::QuaternionToEuler(const geometry_msgs::Pose &curMsg, PoseInfo& cur_pose) {
	constexpr double epsilon = 0.0009765625f;
	constexpr double threshold = 0.5f - epsilon;
	const auto& q_info = curMsg.orientation;
	auto test_value = q_info.w * q_info.y - q_info.x * q_info.z;

	if (std::abs(test_value) > threshold) {
		auto sign = test_value / std::abs(test_value);
		cur_pose.yaw = -2 * sign * std::atan2(q_info.x, q_info.w); // yaw
		cur_pose.pitch = sign * (M_PI_2); // pitch
		cur_pose.roll = 0; // roll
	} else {
		cur_pose.roll = std::atan2(2 * (q_info.y * q_info.z + q_info.w * q_info.x),
			q_info.w * q_info.w - q_info.x * q_info.x - q_info.y * q_info.y +
			q_info.z * q_info.z);
		cur_pose.pitch = std::asin(2 * (q_info.w * q_info.y - q_info.x * q_info.z));
		cur_pose.yaw = std::atan2(2 * (q_info.x * q_info.y + q_info.w * q_info.z),
			q_info.w * q_info.w + q_info.x * q_info.x - q_info.y * q_info.y - q_info.z * q_info.z);
	}

	cur_pose.x = curMsg.position.x;
	cur_pose.y = curMsg.position.y;
	cur_pose.z = curMsg.position.z;
}

void Odometry::SetNewOdometryMsg(const nav_msgs::OdometryConstPtr &odom_msg) {
	const auto& pose = odom_msg->pose.pose;
	PoseInfo cur_pose;
	QuaternionToEuler(pose, cur_pose);
	cur_pose.time = odom_msg->header.stamp.toSec();
	{
		std::lock_guard<std::mutex> lock(lock_);
		pose_vec_.emplace_back(cur_pose);
		if (pose_vec_.size() > pose_num_) {
			pose_vec_.pop_front();
		}
	}
}


bool Odometry::GetPoseInfoFromTimestamp(const double timestamp, PoseInfo& pose_value) {
	std::lock_guard<std::mutex> lock(lock_);
	if (pose_vec_.empty()) {
		return false;
	}
	for (size_t i = 0U; i < pose_vec_.size() - 1U; ++i) {
		if ((pose_vec_[i].time <= timestamp) && (pose_vec_[i + 1].time > timestamp)) {
			pose_value = pose_vec_[i];
			return true;
		}
	}
	if (std::abs(pose_vec_.back().time - timestamp) < 0.05) { // lidar is in the front of odom
		pose_value = pose_vec_.back();
		return true;
	}
	return false;
}
}
}