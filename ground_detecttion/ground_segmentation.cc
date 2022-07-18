#include "ground_segmentation.h"

#include <chrono>
#include <cmath>
#include <list>
#include <memory>
#include <thread>
namespace cvr_lse {
namespace ground_segmentation {
GroundSegmentation::GroundSegmentation(const GroundSegmentationParams& params) : params_(params) {
    segments_.resize(LidarSize);
    bins_indexs_.resize(LidarSize);
    segments_coordinates_.resize(LidarSize);
    for (auto lidar_id = Main; lidar_id < LidarSize; lidar_id = static_cast<LidarID>(lidar_id + 1)) {
        segments_[lidar_id] = SegmentVector(params.n_segments, Segment(params.n_bins, params.max_slope,
            params.max_error_square, params.long_threshold, params.max_long_height, params.max_start_height,
            params.sensor_height[lidar_id]));
    }
    ObtainRangeInformation();
}

void GroundSegmentation::SegmentProcess(const cvr_lse::multi_cloud_label::Ptr& segmentation,
    std::vector<pcl::PointCloud<PointXYZIRT>>& point_cloud_info) {
    // clear the information
    if (point_cloud_info.size() != LidarSize) {
        ROS_ERROR("The lidar messages are not complete");
    }

    for (auto lidar_id = Main; lidar_id < LidarSize; lidar_id = static_cast<LidarID>(lidar_id + 1)) {
        RemoveSelfPointCloud(transform_[lidar_id], point_cloud_info[lidar_id]);
    }

    segmentation->main_label_cloud.resize(point_cloud_info[Main].size());
    segmentation->left_label_cloud.resize(point_cloud_info[Left].size());
    segmentation->right_label_cloud.resize(point_cloud_info[Right].size());
    segmentation->rear_label_cloud.resize(point_cloud_info[Rear].size());

    // resize the size information for each lidar grid map
    for (auto lidar_id = Main; lidar_id < LidarSize; lidar_id = static_cast<LidarID>(lidar_id + 1)) {
        bins_indexs_[lidar_id].resize(point_cloud_info[lidar_id].size());
        segments_coordinates_[lidar_id].resize(point_cloud_info[lidar_id].size());
    }
    // for each lidar
    for (auto lidar_id = Main; lidar_id < LidarSize; lidar_id = static_cast<LidarID>(lidar_id + 1)) {
        InsertPoints(point_cloud_info[lidar_id], lidar_id);
        GetLines(lidar_id);
        AssignCluster(lidar_id, point_cloud_info[lidar_id].size(), segmentation);
    }

    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(point_cloud_info[Main], tempCloud);
    segmentation->main_point_cloud = tempCloud;
    pcl::toROSMsg(point_cloud_info[Left], tempCloud);
    segmentation->left_point_cloud = tempCloud;
    pcl::toROSMsg(point_cloud_info[Right], tempCloud);
    segmentation->right_point_cloud = tempCloud;
    pcl::toROSMsg(point_cloud_info[Rear], tempCloud);
    segmentation->rear_point_cloud = tempCloud;
}

void GroundSegmentation::RemoveSelfPointCloud(const Eigen::Isometry3f& transform,
    pcl::PointCloud<PointXYZIRT>& point_cloud) {
    pcl::transformPointCloud(point_cloud, point_cloud, transform);
    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudInTemp = point_cloud.makeShared();
    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudInTempX;
    laserCloudInTempX.reset(new pcl::PointCloud<PointXYZIRT>());
    pcl::PassThrough<PointXYZIRT> pass;
    pass.setInputCloud(laserCloudInTemp);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(params_.x_range_min, params_.x_range_max);
    pass.setFilterLimitsNegative(true);
    pass.filter(*laserCloudInTempX);

    pass.setInputCloud(laserCloudInTempX);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(params_.y_range_min, params_.y_range_max);
    pass.setFilterLimitsNegative(true);
    pass.filter(point_cloud);
    pcl::transformPointCloud(point_cloud, point_cloud, transform.inverse());
}

void GroundSegmentation::ObtainRangeInformation() {
    transform_.resize(LidarSize);
    // for the main lidar
    transform_[Main] = Eigen::Isometry3f::Identity();
    // for the left lidar to the main
    Eigen::Isometry3f transform_main_left = Eigen::Isometry3f::Identity();
    Eigen::Vector3f trans_main_left = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(
        params_.extrinsic_trans_left.data(), 3, 1);
    auto rot_main_left = Eigen::Quaternionf(params_.extrinsic_rot_left[0U], params_.extrinsic_rot_left[1U],
        params_.extrinsic_rot_left[2U], params_.extrinsic_rot_left[3U]);
    transform_main_left.rotate(rot_main_left);
    transform_main_left.pretranslate(trans_main_left);
    std::swap(transform_[Left], transform_main_left);

    // for the right lidar to the main
    Eigen::Isometry3f transform_main_right = Eigen::Isometry3f::Identity();
    Eigen::Vector3f trans_main_right = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(
        params_.extrinsic_trans_right.data(), 3, 1);
    auto rot_main_right = Eigen::Quaternionf(params_.extrinsic_rot_right[0U], params_.extrinsic_rot_right[1U],
        params_.extrinsic_rot_right[2U], params_.extrinsic_rot_right[3U]);
    transform_main_right.rotate(rot_main_right);
    transform_main_right.pretranslate(trans_main_right);
    std::swap(transform_[Right], transform_main_right);

    // for the rear lidar to the main
    Eigen::Isometry3f transform_main_rear = Eigen::Isometry3f::Identity();
    Eigen::Vector3f trans_main_rear = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(
        params_.extrinsic_trans_rear.data(), 3, 1);
    auto rot_main_rear = Eigen::Quaternionf(params_.extrinsic_rot_rear[0U], params_.extrinsic_rot_rear[1U],
        params_.extrinsic_rot_rear[2U], params_.extrinsic_rot_rear[3U]);
    transform_main_rear.rotate(rot_main_rear);
    transform_main_rear.pretranslate(trans_main_rear);
    std::swap(transform_[Rear], transform_main_rear);
}

void GroundSegmentation::Reset() {
    for (auto lidar_id = Main; lidar_id < LidarSize; lidar_id = static_cast<LidarID>(lidar_id + 1)) {
        segments_[lidar_id].clear();
        segments_[lidar_id] = SegmentVector(params_.n_segments, Segment(params_.n_bins, params_.max_slope,
        params_.max_error_square, params_.long_threshold, params_.max_long_height, params_.max_start_height,
        params_.sensor_height[lidar_id]));
        bins_indexs_[lidar_id].clear();
        segments_coordinates_[lidar_id].clear();
    }
}

void GroundSegmentation::Clear(const cvr_lse::multi_cloud_label::Ptr& segmentation) {
    segmentation->main_point_cloud.data.clear();
    segmentation->main_label_cloud.clear();

    segmentation->left_point_cloud.data.clear();
    segmentation->left_label_cloud.clear();

    segmentation->right_point_cloud.data.clear();
    segmentation->right_label_cloud.clear();

    segmentation->rear_point_cloud.data.clear();
    segmentation->rear_label_cloud.clear();
}

void GroundSegmentation::InsertPoints(const PointCloud& cloud, const LidarID lidar_id) {
	std::vector<std::thread> threads(params_.n_threads);
	const size_t points_per_thread = cloud.size() / params_.n_threads;
	// Launch threads.
	for (size_t i = 0U; i < params_.n_threads - 1U; ++i) {
		const size_t start_index = i * points_per_thread;
		const size_t end_index = (i + 1) * points_per_thread;
		threads[i] = std::thread(&GroundSegmentation::InsertionThread, this,
            cloud, lidar_id, start_index, end_index);
	}
	// Launch last thread which might have more points than others.
	const size_t start_index = (params_.n_threads - 1U) * points_per_thread;
	const size_t end_index = cloud.size();
	threads[params_.n_threads - 1U] = std::thread(&GroundSegmentation::InsertionThread, this,
        cloud, lidar_id, start_index, end_index);
	// Wait for threads to finish.
	for (auto& thread : threads) {
		thread.join();
	}
}

void GroundSegmentation::InsertionThread(const PointCloud& cloud, const LidarID lidar_id, const size_t start_index,
    const size_t end_index) {
	const double segment_step = 2 * M_PI / params_.n_segments;
	const double bin_step = (sqrt(params_.r_max_square) - sqrt(params_.r_min_square)) / params_.n_bins;
	const double r_min = sqrt(params_.r_min_square);
	for (unsigned int i = start_index; i < end_index; ++i) {
		PointXYZIRT point(cloud[i]);
		const double range_square = point.x * point.x + point.y * point.y;
		const double range = std::sqrt(range_square);
		if ((range_square < params_.r_max_square) && (range_square > params_.r_min_square)) {
			const double angle = std::atan2(point.y, point.x);
			auto bin_index = static_cast<size_t>((range - r_min) / bin_step);
			auto segment_index = static_cast<size_t>((angle + M_PI) / segment_step);
			size_t segment_index_clamped = (segment_index == params_.n_segments) ? 0 : segment_index;
			segments_[lidar_id][segment_index_clamped][bin_index].AddPoint(range, point.z);
            bins_indexs_[lidar_id][i] = std::make_pair(segment_index_clamped, bin_index);
		} else {
            bins_indexs_[lidar_id][i] = std::make_pair<int, int>(-1, -1);
		}
        segments_coordinates_[lidar_id][i] = Bin::MinZPoint(range, point.z);
	}
}

void GroundSegmentation::GetLines(const LidarID lidar_id) {
	std::vector<std::thread> thread_vec(params_.n_threads);
	for (size_t i = 0U; i < params_.n_threads - 1U; ++i) {
		const unsigned int start_index = params_.n_segments / params_.n_threads * i;
		const unsigned int end_index = params_.n_segments / params_.n_threads * (i + 1U);
		thread_vec[i] = std::thread(&GroundSegmentation::LineFitThread, this, lidar_id, start_index, end_index);
	}
	const unsigned int start_index = params_.n_segments / params_.n_threads * (params_.n_threads - 1U);
	const unsigned int end_index = params_.n_segments;
	thread_vec[params_.n_threads - 1U] = std::thread(&GroundSegmentation::LineFitThread, this, lidar_id,
        start_index, end_index);
	for (auto& it : thread_vec) {
		it.join();
	}
}

void GroundSegmentation::LineFitThread(const LidarID lidar_id, const unsigned int start_index,
    const unsigned int end_index) {
	for (unsigned int i = start_index; i < end_index; ++i) {
		segments_[lidar_id][i].FitSegmentLines();
	}
}

void GroundSegmentation::AssignCluster(const LidarID lidar_id, const size_t point_size,
    const cvr_lse::multi_cloud_label::Ptr& segmentation) {
	std::vector<std::thread> thread_vec(params_.n_threads);
	for (size_t i = 0U; i < params_.n_threads - 1U; ++i) {
		const unsigned int start_index = point_size / params_.n_threads * i;
		const unsigned int end_index = point_size / params_.n_threads * (i + 1U);
		thread_vec[i] = std::thread(&GroundSegmentation::AssignClusterThread, this,
			lidar_id, start_index, end_index, segmentation);
	}
	const unsigned int start_index = point_size / params_.n_threads * (params_.n_threads - 1U);
	const unsigned int end_index = point_size;
	thread_vec[params_.n_threads - 1U] = std::thread(&GroundSegmentation::AssignClusterThread, this,
		lidar_id, start_index, end_index, segmentation);
	for (auto &it : thread_vec) {
		it.join();
	}
}

void GroundSegmentation::AssignClusterThread(const LidarID lidar_id, const unsigned int& start_index,
    const unsigned int& end_index, const cvr_lse::multi_cloud_label::Ptr& segmentation) {
    if (lidar_id >= LidarSize) {
        return;
    }
    auto& label_points = ObtainsCorrespondingLabelPoints(lidar_id, segmentation);
	const double segment_step = 2 * M_PI / params_.n_segments;
	for (unsigned int i = start_index; i < end_index; ++i) {
		const auto& point_2d = segments_coordinates_[lidar_id][i];
		const int segment_index = bins_indexs_[lidar_id][i].first;
		if (segment_index >= 0) {
			double dist = segments_[lidar_id][segment_index].VerticalDistanceToLine(point_2d.d, point_2d.z);
			// Search neighboring segments.
			int steps = 1;
			while (dist == -1 && steps * segment_step < params_.line_search_angle) {
				// Fix indices that are out of bounds.
				int index_1 = segment_index + steps;
				while (index_1 >= params_.n_segments) index_1 -= params_.n_segments;
				int index_2 = segment_index - steps;
				while (index_2 < 0) index_2 += params_.n_segments;
				// Get distance to neighboring lines.
				const double dist_1 = segments_[lidar_id][index_1].VerticalDistanceToLine(point_2d.d, point_2d.z);
				const double dist_2 = segments_[lidar_id][index_2].VerticalDistanceToLine(point_2d.d, point_2d.z);
				// Select larger distance if both segments return a valid distance.
				if (dist_1 > dist) {
					dist = dist_1;
				}
				if (dist_2 > dist) {
					dist = dist_2;
				}
				++steps;
			}
			if (dist != -1) {
                label_points[i].is_height_valid = true;
                label_points[i].height = static_cast<float>(dist);
				if (dist < params_.max_dist_to_line) {
                    label_points[i].label = 1U;
				} else {
                    label_points[i].label = 0U;
				}
			} else {
                label_points[i].is_height_valid = true;
                label_points[i].height = static_cast<float>(point_2d.z + params_.sensor_height[lidar_id]);
				if ((point_2d.d < 6.0) && (point_2d.z < (0.25 - params_.sensor_height[lidar_id]))) {
                    label_points[i].label = 1U;
				} else {
                    label_points[i].label = 0U;
				}
			}
		}
	}
}

std::vector<cvr_lse::label_point>& GroundSegmentation::ObtainsCorrespondingLabelPoints(const LidarID lidar_id,
     const cvr_lse::multi_cloud_label::Ptr& segmentation) {
    if (lidar_id == Main) {
        return segmentation->main_label_cloud;
    } else if (lidar_id == Left) {
        return segmentation->left_label_cloud;
    } else if (lidar_id == Right) {
        return segmentation->right_label_cloud;
    } else {
        return segmentation->rear_label_cloud;
    }
}

}
}