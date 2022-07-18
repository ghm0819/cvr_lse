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

/*
 * parameter: cloud_info, contains the point and label info
 * parameter: pose_info, the main lidar pose
 */
void CvrLseProcess::Process(const multi_cloud_label& cloud_info, const PoseInfo& pose_info) {
	// the received pose is lidar pose
	std::vector<ScanAll> scan_obstacle_info;
	std::vector<ScanAll> scan_ground_info;
	PreProcessPointCloud(cloud_info, scan_obstacle_info, scan_ground_info);

	if (initial_flag_) {
		cur_lidar_pos_[ground_segmentation::Main] = pose_info;
		cur_veh_pos_ = TranformFromLidarToVehicle(pose_info, ground_segmentation::Main);
        TransformFromVehicleToAuxiliaryLidars(cur_veh_pos_);
		map_position_ = ObtainCurrentMapPosition(cur_veh_pos_);
		InitCvrLse();
		update_flag_.setConstant(0U);
		for (auto lidar_id = ground_segmentation::Main; lidar_id < ground_segmentation::LidarSize;
		    lidar_id = static_cast<ground_segmentation::LidarID>(lidar_id + 1)) {
            UpdateGridMap(lidar_id, scan_obstacle_info, scan_ground_info);
		}
		initial_flag_ = false;
		return;
	}
	last_lidar_pos_ = cur_lidar_pos_;
	last_veh_pos_ = cur_veh_pos_;

	cur_lidar_pos_[ground_segmentation::Main] = pose_info;
	cur_veh_pos_ = TranformFromLidarToVehicle(pose_info, ground_segmentation::Main);
	TransformFromVehicleToAuxiliaryLidars(cur_veh_pos_);
	map_position_ = ObtainCurrentMapPosition(cur_veh_pos_);
	map_->move(map_position_);
	update_flag_.setConstant(0U);
    for (auto lidar_id = ground_segmentation::Main; lidar_id < ground_segmentation::LidarSize;
         lidar_id = static_cast<ground_segmentation::LidarID>(lidar_id + 1)) {
        UpdateGridMap(lidar_id, scan_obstacle_info, scan_ground_info);
    }
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
    ext_transform_matrix_lidar_veh_.resize(ground_segmentation::LidarSize);
    ext_transform_matrix_veh_lidar_.resize(ground_segmentation::LidarSize);
    for (auto lidar_id = ground_segmentation::Main; lidar_id < ground_segmentation::LidarSize;
        lidar_id = static_cast<ground_segmentation::LidarID>(lidar_id + 1)) {
        ext_transform_matrix_lidar_veh_[lidar_id] = Eigen::Isometry3d::Identity();
        ext_transform_matrix_veh_lidar_[lidar_id] = Eigen::Isometry3d::Identity();
    }
    // transform from Main to Vehicle
    Eigen::Vector3d tran_temp;
    Eigen::Quaterniond rot_temp;
    tran_temp = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(param_.extrinsic_trans.data(), 3, 1);
    rot_temp = Eigen::Quaterniond(param_.extrinsic_rot[0U], param_.extrinsic_rot[1U],
                                  param_.extrinsic_rot[2U], param_.extrinsic_rot[3U]);
	ext_transform_matrix_veh_lidar_[ground_segmentation::Main].rotate(rot_temp);
	ext_transform_matrix_veh_lidar_[ground_segmentation::Main].pretranslate(tran_temp);

	ext_transform_matrix_lidar_veh_[ground_segmentation::Main] =
        ext_transform_matrix_veh_lidar_[ground_segmentation::Main].inverse();

	// transform from Left to Vehicle
    tran_temp = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
        param_.extrinsic_trans_left.data(), 3, 1);
    rot_temp = Eigen::Quaterniond(param_.extrinsic_rot_left[0U], param_.extrinsic_rot_left[1U],
                                  param_.extrinsic_rot_left[2U], param_.extrinsic_rot_left[3U]);
    ext_transform_matrix_veh_lidar_[ground_segmentation::Left].rotate(rot_temp);
    ext_transform_matrix_veh_lidar_[ground_segmentation::Left].pretranslate(tran_temp);
    ext_transform_matrix_veh_lidar_[ground_segmentation::Left] =
        ext_transform_matrix_veh_lidar_[ground_segmentation::Main] *
        ext_transform_matrix_veh_lidar_[ground_segmentation::Left];
    ext_transform_matrix_lidar_veh_[ground_segmentation::Left] =
        ext_transform_matrix_veh_lidar_[ground_segmentation::Left].inverse();

    // transform from right to Vehicle
    tran_temp = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
        param_.extrinsic_trans_right.data(), 3, 1);
    rot_temp = Eigen::Quaterniond(param_.extrinsic_rot_right[0U], param_.extrinsic_rot_right[1U],
                                  param_.extrinsic_rot_right[2U], param_.extrinsic_rot_right[3U]);
    ext_transform_matrix_veh_lidar_[ground_segmentation::Right].rotate(rot_temp);
    ext_transform_matrix_veh_lidar_[ground_segmentation::Right].pretranslate(tran_temp);
    ext_transform_matrix_veh_lidar_[ground_segmentation::Right] =
        ext_transform_matrix_veh_lidar_[ground_segmentation::Main] *
        ext_transform_matrix_veh_lidar_[ground_segmentation::Right];
    ext_transform_matrix_lidar_veh_[ground_segmentation::Right] =
        ext_transform_matrix_veh_lidar_[ground_segmentation::Right].inverse();

    // transform from Rear to Vehicle
    tran_temp = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
            param_.extrinsic_trans_rear.data(), 3, 1);
    rot_temp = Eigen::Quaterniond(param_.extrinsic_rot_rear[0U], param_.extrinsic_rot_rear[1U],
                                  param_.extrinsic_rot_rear[2U], param_.extrinsic_rot_rear[3U]);
    ext_transform_matrix_veh_lidar_[ground_segmentation::Rear].rotate(rot_temp);
    ext_transform_matrix_veh_lidar_[ground_segmentation::Rear].pretranslate(tran_temp);
    ext_transform_matrix_veh_lidar_[ground_segmentation::Rear] =
            ext_transform_matrix_veh_lidar_[ground_segmentation::Main] *
            ext_transform_matrix_veh_lidar_[ground_segmentation::Rear];
    ext_transform_matrix_lidar_veh_[ground_segmentation::Rear] =
        ext_transform_matrix_veh_lidar_[ground_segmentation::Rear].inverse();
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

PoseInfo CvrLseProcess::TranformFromLidarToVehicle(const PoseInfo& lidar_pose,
    const ground_segmentation::LidarID& lidar_id) {
	auto lidar_quat = Eigen::AngleAxisd(lidar_pose.yaw, Eigen::Vector3d::UnitZ()) *
	                  Eigen::AngleAxisd(lidar_pose.pitch, Eigen::Vector3d::UnitY()) *
	                  Eigen::AngleAxisd(lidar_pose.roll, Eigen::Vector3d::UnitX());
	Eigen::Isometry3d lidar_matrix = Eigen::Isometry3d::Identity();
	lidar_matrix.rotate(lidar_quat);
	lidar_matrix.pretranslate(Eigen::Vector3d(lidar_pose.x, lidar_pose.y, lidar_pose.z));
	auto veh_matrix = lidar_matrix * ext_transform_matrix_lidar_veh_[lidar_id];
	double roll = 0.0;
	double pitch = 0.0;
	double yaw = 0;
	QuaternionToEuler(Eigen::Quaterniond(veh_matrix.rotation().matrix()), roll, pitch, yaw);
	return PoseInfo {veh_matrix.translation().x(), veh_matrix.translation().y(), veh_matrix.translation().z(),
	                 roll, pitch, yaw, lidar_pose.time};
}

void CvrLseProcess::TransformFromVehicleToAuxiliaryLidars(const PoseInfo& veh_pose) {
    for (auto lidar_id = ground_segmentation::Left; lidar_id < ground_segmentation::LidarSize;
        lidar_id = static_cast<ground_segmentation::LidarID>(lidar_id + 1)) {
        cur_lidar_pos_[lidar_id] = TranformFromVehicleToLidar(veh_pose, lidar_id);
    }
}

PoseInfo CvrLseProcess::TranformFromVehicleToLidar(const PoseInfo &veh_pose,
    const ground_segmentation::LidarID& lidar_id) {
    auto veh_quat = Eigen::AngleAxisd(veh_pose.yaw, Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(veh_pose.pitch, Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(veh_pose.roll, Eigen::Vector3d::UnitX());
    Eigen::Isometry3d veh_matrix = Eigen::Isometry3d::Identity();
    veh_matrix.rotate(veh_quat);
    veh_matrix.pretranslate(Eigen::Vector3d(veh_pose.x, veh_pose.y, veh_pose.z));
    auto lidar_matrix = veh_matrix * ext_transform_matrix_veh_lidar_[lidar_id];
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0;
    QuaternionToEuler(Eigen::Quaterniond(lidar_matrix.rotation().matrix()), roll, pitch, yaw);
    return PoseInfo {lidar_matrix.translation().x(), lidar_matrix.translation().y(), lidar_matrix.translation().z(),
        roll, pitch, yaw, veh_pose.time};
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

grid_map::Position CvrLseProcess::ObtainCurrentMapPosition(const PoseInfo &vehPose) const {
	auto vehQuat = Eigen::AngleAxisd(vehPose.yaw, Eigen::Vector3d::UnitZ()) *
		Eigen::AngleAxisd(vehPose.pitch, Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxisd(vehPose.roll, Eigen::Vector3d::UnitX());
	Eigen::Vector3d mapPose = vehQuat.matrix() * Eigen::Vector3d(param_.offset_x, param_.offset_y, 0.0) +
		Eigen::Vector3d(vehPose.x, vehPose.y, vehPose.z);
	return grid_map::Position {mapPose.x(), mapPose.y()};
}

void CvrLseProcess::PreProcessPointCloud(const multi_cloud_label& cloud_info, std::vector<ScanAll>& scan_obstacle_info,
	std::vector<ScanAll>& scan_ground_info) {
    scan_obstacle_info.resize(ground_segmentation::LidarSize);
    scan_ground_info.resize(ground_segmentation::LidarSize);
    for (auto lidar_id = ground_segmentation::Main; lidar_id < ground_segmentation::LidarSize;
        lidar_id = static_cast<ground_segmentation::LidarID>(lidar_id + 1)) {
        scan_obstacle_info[lidar_id].resize(static_cast<size_t>(360.0 / param_.horizontal_resolution));
        scan_ground_info[lidar_id].resize(static_cast<size_t>(360.0 / param_.horizontal_resolution));
        pcl::PointCloud<PointXYZIRT> point_info;
        std::vector<cvr_lse::label_point> label_info;
        if (lidar_id == ground_segmentation::Main) {
            label_info = cloud_info.main_label_cloud;
            pcl::fromROSMsg(cloud_info.main_point_cloud, point_info);
        } else if (lidar_id == ground_segmentation::Left) {
            label_info = cloud_info.left_label_cloud;
            pcl::fromROSMsg(cloud_info.left_point_cloud, point_info);
        } else if (lidar_id == ground_segmentation::Right) {
            label_info = cloud_info.right_label_cloud;
            pcl::fromROSMsg(cloud_info.right_point_cloud, point_info);
        } else if (lidar_id == ground_segmentation::Rear) {
            label_info = cloud_info.rear_label_cloud;
            pcl::fromROSMsg(cloud_info.rear_point_cloud, point_info);
        } else {
            continue;
        }
        PreProcessSinglePointCloud(label_info, point_info, scan_obstacle_info[lidar_id],
            scan_ground_info[lidar_id]);
    }
    //
//    pcl::PointCloud<PointXYZIRT> obstacle_cloud;
//    obstacle_cloud.header.stamp = cloud_info.header.stamp.toNSec();
//    obstacle_cloud.header.frame_id = "lidar_link";
//    for (const auto& obstacle : scan_obstacle_info) {
//        if (!obstacle.empty()) {
//            PointXYZIRT point{};
//            point.x = static_cast<float>(obstacle.front().x());
//            point.y = static_cast<float>(obstacle.front().y());
//            point.z = static_cast<float>(obstacle.front().z());
//            obstacle_cloud.push_back(point);
//        }
//    }
//    obstacle_pub_.publish(obstacle_cloud);
    //
}

void CvrLseProcess::PreProcessSinglePointCloud(const std::vector<cvr_lse::label_point>& label_point_info,
    const pcl::PointCloud<PointXYZIRT>& point_info, ScanAll& obstacle_points, ScanAll& ground_points) {
    for (size_t i = 0U; i < point_info.size(); ++i) {
        const auto& point = point_info[i];
        auto point_index = ObtainIndexOfPoint(point);
        if (label_point_info[i].label == 0U) {
            obstacle_points[point_index].emplace_back(point.x, point.y, point.z, 0U, point.timestamp,
                label_point_info[i].is_height_valid, label_point_info[i].height, point.ring);
        } else {
            ground_points[point_index].emplace_back(point.x, point.y, point.z, 1U, point.timestamp,
                label_point_info[i].is_height_valid, label_point_info[i].height, point.ring);
        }
    }

    for (auto& scan_ray : obstacle_points) {
        std::sort(scan_ray.begin(), scan_ray.end(), [](const ScanPoint& point_a, const ScanPoint&
        point_b) {
            return point_a.GetDistance() < point_b.GetDistance();
        });
    }

    for (auto& scan_ray : ground_points) {
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

void CvrLseProcess::TranformScanPointToOdom(const ground_segmentation::LidarID& lidar_id, ScanPoint &point) {
	Eigen::Vector3d veh_point = ext_transform_matrix_veh_lidar_[lidar_id] * point.GetLidarPointInfo();
	point.SetVehiclePointInfo(veh_point);
	auto lidar_quat = Eigen::AngleAxisd(cur_lidar_pos_[lidar_id].yaw, Eigen::Vector3d::UnitZ()) *
	                Eigen::AngleAxisd(cur_lidar_pos_[lidar_id].pitch, Eigen::Vector3d::UnitY()) *
	                Eigen::AngleAxisd(cur_lidar_pos_[lidar_id].roll, Eigen::Vector3d::UnitX());
	Eigen::Vector3d odom_point = lidar_quat.matrix() * point.GetLidarPointInfo() +
		Eigen::Vector3d(cur_lidar_pos_[lidar_id].x, cur_lidar_pos_[lidar_id].y, cur_lidar_pos_[lidar_id].z);
	point.SetOdomPointInfo(odom_point);
}

void CvrLseProcess::UpdateGridMap(const ground_segmentation::LidarID& lidar_id,
    std::vector<ScanAll>& scan_obstacle_Info, std::vector<ScanAll>& scan_ground_info) {
	constexpr double height_threshold = 1.8;
	const double angle_step = param_.horizontal_resolution / 180.0 * M_PI;
	double current_angle = -M_PI - angle_step;
	for (size_t i = 0U; i < scan_obstacle_Info[lidar_id].size(); ++i) {
		if (!scan_obstacle_Info[lidar_id][i].empty()) {
            bool scan_valid_flag = false;
            current_angle += angle_step;
            for (auto& point : scan_obstacle_Info[lidar_id][i]) {
                TranformScanPointToOdom(lidar_id, point);
                if (point.GetHeightDistance() > height_threshold) {
                    continue;
                }
                scan_valid_flag = true;
                UpdateRay(lidar_id, point);
                break;
            }
            if (!scan_valid_flag) {
                UpdateInvalidScanPoint(lidar_id, current_angle);
            }
		} else if (!scan_ground_info[lidar_id][i].empty()) {
		    auto& ground_point = scan_ground_info[lidar_id][i].back();
            TranformScanPointToOdom(lidar_id, ground_point);
            UpdateValidGroundPoint(lidar_id, ground_point);
		} else {
		    // do nothing
		}
	}
}

void CvrLseProcess::UpdateValidGroundPoint(const ground_segmentation::LidarID& lidar_id, const ScanPoint& point) {
    grid_map::Position start_pose(cur_lidar_pos_[lidar_id].x, cur_lidar_pos_[lidar_id].y);
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
}

void CvrLseProcess::UpdateInvalidScanPoint(const ground_segmentation::LidarID& lidar_id, const double angleInfo) {
	Eigen::Vector3d lidar_invalid_point(param_.max_distance * std::cos(angleInfo),
		param_.max_distance * std::sin(angleInfo), 0.0);
	auto lidar_quat = Eigen::AngleAxisd(cur_lidar_pos_[lidar_id].yaw, Eigen::Vector3d::UnitZ()) *
	               Eigen::AngleAxisd(cur_lidar_pos_[lidar_id].pitch, Eigen::Vector3d::UnitY()) *
	               Eigen::AngleAxisd(cur_lidar_pos_[lidar_id].roll, Eigen::Vector3d::UnitX());
	Eigen::Vector3d odom_invalid_point = lidar_quat.matrix() * lidar_invalid_point +
        Eigen::Vector3d(cur_lidar_pos_[lidar_id].x, cur_lidar_pos_[lidar_id].y, cur_lidar_pos_[lidar_id].z);

	grid_map::Position start_pose(cur_lidar_pos_[lidar_id].x, cur_lidar_pos_[lidar_id].y);
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

void CvrLseProcess::UpdateRay(const ground_segmentation::LidarID& lidar_id, const ScanPoint &point) {
	grid_map::Position start_pose(cur_lidar_pos_[lidar_id].x, cur_lidar_pos_[lidar_id].y);
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

void CvrLseProcess::UpdateStitchPolygon(const ground_segmentation::LidarID& lidar_id, const ScanPoint& pointLast,
    const ScanPoint& pointCurrent) {
	grid_map::Polygon polygon;
	polygon.addVertex(grid_map::Position(cur_lidar_pos_[lidar_id].x, cur_lidar_pos_[lidar_id].y));
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