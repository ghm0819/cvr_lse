//
// Created by ghm on 2021/10/25.
//

#include "cvr_lse_process.h"
#include <fstream>
namespace cvr_lse {
namespace lse_map {

void CvrLseProcess::InitCvrLse() {
	map_ = std::make_shared<grid_map::GridMap>(layer_);
	map_->setFrameId("base_link");
	map_->setGeometry(grid_map::Length(param_.map_width, param_.map_height), param_.map_resolution,
	                  map_position_);
	post_process_ = std::make_shared<postprocess::PostProcess>(nh_, param_);
	update_flag_.resize(map_->getSize()(0U), map_->getSize()(1U));
}

void CvrLseProcess::Process(const CloudInfo& cloud_info, const PoseInfo& pose_info) {
	// the received pose is lidar pose
	ScanAll scan_obstacle_info;
	ScanAll scan_ground_info;
	scan_obstacle_info.resize(static_cast<size_t>(360.0 / param_.horizontal_resolution));
	scan_ground_info.resize(static_cast<size_t>(360.0 / param_.horizontal_resolution));
	PreProcessPointCloud(cloud_info, scan_obstacle_info, scan_ground_info);
	//
	pcl::PointCloud<PointXYZIRT> obstacle_cloud;
	obstacle_cloud.header = cloud_info.first.header;
	obstacle_cloud.header.frame_id = "lidar_link";
	for (const auto& obstacle : scan_obstacle_info) {
		if (!obstacle.empty()) {
			PointXYZIRT point{};
			point.x = static_cast<float>(obstacle.front().x());
			point.y = static_cast<float>(obstacle.front().y());
			point.z = static_cast<float>(obstacle.front().z());
			obstacle_cloud.push_back(point);
		}
	}
	obstacle_pub_.publish(obstacle_cloud);
	//
	if (initial_flag_) {
		cur_lidar_pos_ = pose_info;
		cur_veh_pos_ = TranformFromLidarToVehicle(pose_info);
		map_position_ = ObtainCurrentMapPosition(cur_veh_pos_);
		InitCvrLse();
		update_flag_.setConstant(0U);
		UpdateGridMap(scan_obstacle_info);
		initial_flag_ = false;
		return;
	}
	last_lidar_pos_ = cur_lidar_pos_;
	last_veh_pos_ = cur_veh_pos_;

	cur_lidar_pos_ = pose_info;
	cur_veh_pos_ = TranformFromLidarToVehicle(pose_info);
	map_position_ = ObtainCurrentMapPosition(cur_veh_pos_);
	map_->move(map_position_);
	update_flag_.setConstant(0U);
	UpdateGridMap(scan_obstacle_info);
	PublishGridMap(*map_);

	// process the submap from the complete map information
	bool isSuccess = false;
	auto submap = map_->getSubmap(map_position_, grid_map::Length(param_.map_output_width, param_.map_output_height),
	                              isSuccess);
	std::vector<std::vector<cv::Point2f>> find_contour;
	post_process_->ProcessGridMap(submap, find_contour);
	TransformFromOdomToVeh(cur_veh_pos_, find_contour);
	PublishContours(find_contour);
}

void CvrLseProcess::InitialTransformationInfo() {
	ext_trans_veh_lidar_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(param_.extTran.data(), 3, 1);
	ext_rot_veh_lidar_ = Eigen::Quaterniond(param_.extRot[0U], param_.extRot[1U], param_.extRot[2U], param_.extRot[3U]);
	ext_rot_lidar_veh_ = ext_rot_veh_lidar_.inverse();
	ext_trans_lidar_veh_ = -ext_rot_lidar_veh_.matrix() * ext_trans_veh_lidar_;

	ext_transform_matrix_veh_lidar_.rotate(ext_rot_veh_lidar_);
	ext_transform_matrix_veh_lidar_.pretranslate(ext_trans_veh_lidar_);

	ext_transform_matrix_lidar_veh_.rotate(ext_rot_lidar_veh_);
	ext_transform_matrix_lidar_veh_.pretranslate(ext_trans_lidar_veh_);
}

void CvrLseProcess::TransformFromOdomToVeh(const PoseInfo& cur_pose,
	std::vector<std::vector<cv::Point2f>>& find_contour) {
	const float cos_value = std::cos(cur_pose.yaw);
	const float sin_value = std::sin(cur_pose.yaw);
	const float xt = -cos_value * cur_pose.x - sin_value * cur_pose.y;
	const float yt = sin_value * cur_pose.x - cos_value * cur_pose.y;
	for (auto& contour : find_contour) {
		for (auto& pt : contour) {
			auto point = pt;
			pt.x = point.x * cos_value + point.y * sin_value + xt;
			pt.y = -point.x * sin_value + point.y * cos_value + yt;
		}
	}
}

void CvrLseProcess::InitialThresholdInfo() {
	occupancy_threshold_ = static_cast<float>(std::log(param_.occupancy_threshold / (1.0 - param_.occupancy_threshold)));
	free_threshold_ = static_cast<float>(std::log(param_.free_threshold / (1.0 - param_.free_threshold)));
	occupancy_factor_ = static_cast<float>(std::log(param_.occupancy_factor / (1.0 - param_.occupancy_factor)));
	free_factor_ = static_cast<float>(std::log(param_.free_factor / (1.0 - param_.free_factor)));
}

PoseInfo CvrLseProcess::TranformFromLidarToVehicle(const PoseInfo& lidar_pose) {
	auto lidar_quat = Eigen::AngleAxisd(lidar_pose.yaw, Eigen::Vector3d::UnitZ()) *
	                  Eigen::AngleAxisd(lidar_pose.pitch, Eigen::Vector3d::UnitY()) *
	                  Eigen::AngleAxisd(lidar_pose.roll, Eigen::Vector3d::UnitX());
	Eigen::Isometry3d lidar_matrix = Eigen::Isometry3d::Identity();
	lidar_matrix.rotate(lidar_quat);
	lidar_matrix.pretranslate(Eigen::Vector3d(lidar_pose.x, lidar_pose.y, lidar_pose.z));
	auto veh_matrix = lidar_matrix * ext_transform_matrix_lidar_veh_;
	double roll = 0.0;
	double pitch = 0.0;
	double yaw = 0;
	QuaternionToEuler(Eigen::Quaterniond(veh_matrix.rotation().matrix()), roll, pitch, yaw);
	return PoseInfo {veh_matrix.translation().x(), veh_matrix.translation().y(), veh_matrix.translation().z(),
	                 roll, pitch, yaw, lidar_pose.time};
}

void CvrLseProcess::QuaternionToEuler(const Eigen::Quaterniond& q_info, double& roll, double& pitch, double& yaw) {
	constexpr double epsilon = 0.0009765625f;
	constexpr double threshold = 0.5f - epsilon;
	auto testValue = q_info.w() * q_info.y() - q_info.x() * q_info.z();

	if (std::abs(testValue) > threshold) {
		auto sign = testValue / std::abs(testValue);
		yaw = -2 * sign * std::atan2(q_info.x(), q_info.w()); // yaw
		pitch = sign * (M_PI_2); // pitch
		roll = 0; // roll
	} else {
		roll = std::atan2(2 * (q_info.y() * q_info.z() + q_info.w() * q_info.x()),
			q_info.w() * q_info.w() - q_info.x() * q_info.x() - q_info.y() * q_info.y() +
			q_info.z() * q_info.z());
		pitch = std::asin(2 * (q_info.w() * q_info.y() - q_info.x() * q_info.z()));
		yaw = std::atan2(2 * (q_info.x() * q_info.y() + q_info.w() * q_info.z()),
			q_info.w() * q_info.w() + q_info.x() * q_info.x() - q_info.y() * q_info.y() - q_info.z() * q_info.z());
	}
}

PoseInfo CvrLseProcess::TranformFromVehicleToLidar(const PoseInfo &vehPose) {
	auto lidar_quat = Eigen::AngleAxisd(vehPose.yaw, Eigen::Vector3d::UnitZ()) *
	                  Eigen::AngleAxisd(vehPose.pitch, Eigen::Vector3d::UnitY()) *
	                  Eigen::AngleAxisd(vehPose.roll, Eigen::Vector3d::UnitX());
	Eigen::Isometry3d veh_matrix = Eigen::Isometry3d::Identity();
	veh_matrix.rotate(lidar_quat);
	veh_matrix.pretranslate(Eigen::Vector3d(vehPose.x, vehPose.y, vehPose.z));
	auto lidar_matrix = veh_matrix * ext_transform_matrix_lidar_veh_;
	double roll = 0.0;
	double pitch = 0.0;
	double yaw = 0;
	QuaternionToEuler(Eigen::Quaterniond(lidar_matrix.rotation().matrix()), roll, pitch, yaw);
	return PoseInfo {lidar_matrix.translation().x(), lidar_matrix.translation().y(), lidar_matrix.translation().z(),
		roll, pitch, yaw, vehPose.time};
}

grid_map::Position CvrLseProcess::ObtainCurrentMapPosition(const PoseInfo &vehPose) const {
	auto vehQuat = Eigen::AngleAxisd(vehPose.yaw, Eigen::Vector3d::UnitZ()) *
		Eigen::AngleAxisd(vehPose.pitch, Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxisd(vehPose.roll, Eigen::Vector3d::UnitX());
	Eigen::Vector3d mapPose = vehQuat.matrix() * Eigen::Vector3d(param_.offset_x, param_.offset_y, 0.0) +
		Eigen::Vector3d(vehPose.x, vehPose.y, vehPose.z);
	return grid_map::Position {mapPose.x(), mapPose.y()};
}

void CvrLseProcess::PreProcessPointCloud(const CloudInfo& cloud_info, ScanAll& scan_obstacle_info,
	ScanAll& scan_ground_info) {
	const auto& label_info = cloud_info.second;
	const auto& clou_info = cloud_info.first;
	for (size_t i = 0U; i < clou_info.size(); ++i) {
		const auto& point = clou_info[i];
		auto point_index = ObtainIndexOfPoint(point);
		if (label_info.label[i] == 0U) {
			scan_obstacle_info[point_index].emplace_back(point.x, point.y, point.z, 0U, point.timestamp,
				label_info.isHeightValid[i], label_info.height[i], point.ring);
		} else {
			scan_ground_info[point_index].emplace_back(point.x, point.y, point.z, 1U, point.timestamp,
				label_info.isHeightValid[i], label_info.height[i], point.ring);
		}
	}

	for (auto& scan_ray : scan_obstacle_info) {
		std::sort(scan_ray.begin(), scan_ray.end(), [](const ScanPoint& point_a, const ScanPoint&
			point_b) {
			return point_a.GetDistance() < point_b.GetDistance();
		});
	}

	for (auto& scan_ray : scan_ground_info) {
		std::sort(scan_ray.begin(), scan_ray.end(), [](const ScanPoint& point_a, const ScanPoint&
			point_b) {
			return point_a.GetDistance() < point_b.GetDistance();
		});
	}
}

size_t CvrLseProcess::ObtainIndexOfPoint(const PointXYZIRT& point) const {
	auto angle = std::atan2(point.y, point.x) + M_PI;
	if (std::abs(angle - 2 * M_PI) < 1e-6) {
		angle = 0.0;
	}
	return static_cast<size_t>(angle / M_PI * 180 / param_.horizontal_resolution);
}

void CvrLseProcess::TranformScanPointToOdom(ScanPoint &point) {
	Eigen::Vector3d veh_point = ext_rot_veh_lidar_.matrix() * point.GetLidarPointInfo() + ext_trans_veh_lidar_;
	point.SetVehiclePointInfo(veh_point);
	auto veh_quat = Eigen::AngleAxisd(cur_lidar_pos_.yaw, Eigen::Vector3d::UnitZ()) *
	                Eigen::AngleAxisd(cur_lidar_pos_.pitch, Eigen::Vector3d::UnitY()) *
	                Eigen::AngleAxisd(cur_lidar_pos_.roll, Eigen::Vector3d::UnitX());
	Eigen::Vector3d odom_point = veh_quat.matrix() * point.GetLidarPointInfo() +
		Eigen::Vector3d(cur_lidar_pos_.x, cur_lidar_pos_.y, cur_lidar_pos_.z);
	point.SetOdomPointInfo(odom_point);
}

void CvrLseProcess::UpdateGridMap(ScanAll& scan_obstacle_Info) {
	constexpr double height_threshold = 1.8;
	const double angle_step = param_.horizontal_resolution / 180.0 * M_PI;
	double current_angle = -M_PI - angle_step;
	for (auto& scan_Obstacle : scan_obstacle_Info) {
		bool scan_valid_flag = false;
		current_angle += angle_step;
		for (auto& point : scan_Obstacle) {
			TranformScanPointToOdom(point);
			if (point.GetHeightDistance() > height_threshold) {
				continue;
			}
			scan_valid_flag = true;
			UpdateRay(point);
			break;
		}
		if (!scan_valid_flag) {
			UpdateInvalidScanPoint(current_angle);
		}
	}
}

void CvrLseProcess::UpdateInvalidScanPoint(const double angleInfo) {
	Eigen::Vector3d lidar_invalid_point(param_.max_distance * std::cos(angleInfo),
		param_.max_distance * std::sin(angleInfo), 0.0);
	auto vehQuat = Eigen::AngleAxisd(cur_lidar_pos_.yaw, Eigen::Vector3d::UnitZ()) *
	               Eigen::AngleAxisd(cur_lidar_pos_.pitch, Eigen::Vector3d::UnitY()) *
	               Eigen::AngleAxisd(cur_lidar_pos_.roll, Eigen::Vector3d::UnitX());
	Eigen::Vector3d odom_invalid_point = vehQuat.matrix() * lidar_invalid_point +
	                                     Eigen::Vector3d(cur_lidar_pos_.x, cur_lidar_pos_.y, cur_lidar_pos_.z);

	grid_map::Position start_pose(cur_lidar_pos_.x, cur_lidar_pos_.y);
	grid_map::Position end_pose(odom_invalid_point.x(), odom_invalid_point.y());

	for (grid_map::LineIterator iterator(*map_, start_pose, end_pose);
	     !iterator.isPastEnd(); ++iterator) {
		const auto& index = *iterator;
		if (update_flag_(index(0U), index(1U)) != 0U) {
			continue;
		}
		map_->at("prob", *iterator) += free_factor_;
		if (map_->at("prob", *iterator) < param_.min_odd_threshold) {
			map_->at("prob", *iterator) = param_.min_odd_threshold;
		}
		update_flag_(index(0U), index(1U)) = 1U;
	}
}

void CvrLseProcess::UpdateRay(const ScanPoint &point) {
	grid_map::Position start_pose(cur_lidar_pos_.x, cur_lidar_pos_.y);
	grid_map::Position end_pose(point.GetOdomPointInfo().x(), point.GetOdomPointInfo().y());

	for (grid_map::LineIterator iterator(*map_, start_pose, end_pose);
		!iterator.isPastEnd(); ++iterator) {
		const auto& index = *iterator;
		if (update_flag_(index(0U), index(1U)) != 0U) {
			continue;
		}
		map_->at("prob", *iterator) += free_factor_;
		if (map_->at("prob", *iterator) < param_.min_odd_threshold) {
			map_->at("prob", *iterator) = param_.min_odd_threshold;
		}
		update_flag_(index(0U), index(1U)) = 1U;
	}
	grid_map::Index point_index;
	if (map_->getIndex(end_pose, point_index)) {
		if (update_flag_(point_index(0U), point_index(1U)) == 2U) {
			return;
		}
		map_->at("prob", point_index) -= free_factor_;
		map_->at("prob", point_index) += occupancy_factor_;
		if (map_->at("prob", point_index) > param_.max_odd_threshold) {
			map_->at("prob", point_index) = param_.max_odd_threshold;
		}
		map_->at("num", point_index) += 1;
		update_flag_(point_index(0U), point_index(1U)) = 2U;
	}
}

void CvrLseProcess::UpdateStitchPolygon(const ScanPoint& pointLast, const ScanPoint& pointCurrent) {
	grid_map::Polygon polygon;
	polygon.addVertex(grid_map::Position(cur_lidar_pos_.x, cur_lidar_pos_.y));
	polygon.addVertex(grid_map::Position(pointLast.x(), pointLast.y()));
	polygon.addVertex(grid_map::Position(pointCurrent.x(), pointCurrent.y()));

	for (grid_map::PolygonIterator iterator(*map_, polygon); !iterator.isPastEnd(); ++iterator) {
		const auto& index = *iterator;
		if (update_flag_(index(0U), index(1U)) != 0U) {
			continue;
		}
		map_->at("prob", *iterator) += free_factor_;
		if (map_->at("prob", *iterator) < param_.min_odd_threshold) {
			map_->at("prob", *iterator) = param_.min_odd_threshold;
		}
		update_flag_(index(0U), index(1U)) = 1U;
	}
}

void CvrLseProcess::PublishGridMap(const grid_map::GridMap &submap) {
	nav_msgs::OccupancyGrid occupancy_grid;
	occupancy_grid.header.frame_id = "map";
	occupancy_grid.header.stamp = ros::Time::now();

	const double half_length_x = map_->getLength().x() * 0.5;
	const double half_length_y = map_->getLength().y() * 0.5;

	occupancy_grid.info.origin.position.x = map_->getPosition().x() - half_length_x;
	occupancy_grid.info.origin.position.y = map_->getPosition().y() - half_length_y;
	occupancy_grid.info.origin.position.z = cur_veh_pos_.z;
	occupancy_grid.info.origin.orientation.x = 0.0;
	occupancy_grid.info.origin.orientation.y = 0.0;
	occupancy_grid.info.origin.orientation.z = 0.0;
	occupancy_grid.info.origin.orientation.w = 1.0;
	occupancy_grid.info.resolution = map_->getResolution();
	occupancy_grid.info.width = map_->getSize().x();
	occupancy_grid.info.height = map_->getSize().y();
	occupancy_grid.data.resize(occupancy_grid.info.width * occupancy_grid.info.height);

	grid_map::Matrix& data = (*map_)["prob"];
	for (grid_map::GridMapIterator iterator(*map_); !iterator.isPastEnd(); ++iterator) {
		const int i = iterator.getLinearIndex();
		auto indexPoint = iterator.getUnwrappedIndex();
		auto index = (map_->getSize().y() - 1U - indexPoint.x()) + (map_->getSize().x() - 1U - indexPoint.y()) *
				map_->getSize().x();
		if (data(i) > occupancy_threshold_) {
			occupancy_grid.data[index] = -1;
		} else if (data(i) < free_threshold_) {
			occupancy_grid.data[index] = 100;
		} else {
			occupancy_grid.data[index] = 0;
		}
	}
	map_pub_.publish(occupancy_grid);
}

// for visualization
void CvrLseProcess::PublishContours(const std::vector<std::vector<cv::Point2f>>& contours) {
	visualization_msgs::MarkerArray line_markers;
	int32_t currentId = 0U;
	for (const auto& contour : contours) {
		visualization_msgs::Marker line_marker;
		line_marker.ns = "convex";
		line_marker.id = currentId++;
		line_marker.type = visualization_msgs::Marker::LINE_LIST;
		line_marker.scale.x = 0.1;
		line_marker.scale.y = 0.1;
		line_marker.color.r = 0.8;
		line_marker.color.g = 0.0;
		line_marker.color.b = 0.8;
		line_marker.color.a = 1.0;
		line_marker.lifetime = ros::Duration(0.1);
		for (size_t i = 0U; i < contour.size() - 1U; ++i) {
			geometry_msgs::Point p_start;
			p_start.x = contour[i].x;
			p_start.y = contour[i].y;
			p_start.z = 0.3;
			line_marker.points.emplace_back(p_start);
			geometry_msgs::Point p_end;
			p_end.x = contour[i + 1U].x;
			p_end.y = contour[i + 1U].y;
			p_end.z = 0.3;
			line_marker.points.emplace_back(p_end);

			p_start.x = contour[i].x;
			p_start.y = contour[i].y;
			p_start.z = 2.0;
			line_marker.points.emplace_back(p_start);
			p_end.x = contour[i + 1U].x;
			p_end.y = contour[i + 1U].y;
			p_end.z = 2.0;
			line_marker.points.emplace_back(p_end);

			p_start.x = contour[i].x;
			p_start.y = contour[i].y;
			p_start.z = 0.3;
			line_marker.points.emplace_back(p_start);
			p_end.x = contour[i].x;
			p_end.y = contour[i].y;
			p_end.z = 2.0;
			line_marker.points.emplace_back(p_end);
		}
		geometry_msgs::Point p_start;
		p_start.x = contour.back().x;
		p_start.y = contour.back().y;
		p_start.z = 0.3;
		line_marker.points.emplace_back(p_start);
		geometry_msgs::Point p_end;
		p_end.x = contour.front().x;
		p_end.y = contour.front().y;
		p_end.z = 0.3;
		line_marker.points.emplace_back(p_end);

		p_start.x = contour.back().x;
		p_start.y = contour.back().y;
		p_start.z = 2.0;
		line_marker.points.emplace_back(p_start);
		p_end.x = contour.front().x;
		p_end.y = contour.front().y;
		p_end.z = 2.0;
		line_marker.points.emplace_back(p_end);

		p_start.x = contour.back().x;
		p_start.y = contour.back().y;
		p_start.z = 0.3;
		line_marker.points.emplace_back(p_start);
		p_end.x = contour.back().x;
		p_end.y = contour.back().y;
		p_end.z = 2.0;
		line_marker.points.emplace_back(p_end);

		line_marker.header.frame_id = "base_link";
		line_marker.header.stamp = ros::Time::now();
		line_markers.markers.emplace_back(line_marker);
	}
	convex_Marker_Pub_.publish(line_markers);
}
}
}
