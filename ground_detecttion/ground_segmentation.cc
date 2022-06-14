#include "ground_segmentation.h"

#include <chrono>
#include <cmath>
#include <list>
#include <memory>
#include <thread>
namespace cvr_lse {
namespace ground_segmentation {
GroundSegmentation::GroundSegmentation(const GroundSegmentationParams& params) :
	params_(params),
	segments_(params.n_segments, Segment(params.n_bins,
		                                 params.max_slope,
		                                 params.max_error_square,
		                                 params.long_threshold,
		                                 params.max_long_height,
		                                 params.max_start_height,
		                                 params.sensor_height)) {}

void GroundSegmentation::SegmentProcess(const PointCloud& cloud, const cvr_lse::cloud_label::Ptr& segmentation)
{
	segmentation->label.clear();
	segmentation->isHeightValid.clear();
	segmentation->height.clear();
	segmentation->label.resize(cloud.size());
	segmentation->isHeightValid.resize(cloud.size());
	segmentation->height.resize(cloud.size());

	bin_index_.resize(cloud.size());
	segment_coordinates_.resize(cloud.size());

	InsertPoints(cloud);
	GetLines();
	AssignCluster(segmentation);
}

void GroundSegmentation::Reset() {
	bin_index_.clear();
	segment_coordinates_.clear();
	segments_.clear();
	segments_ = std::vector<Segment>(params_.n_segments, Segment(params_.n_bins, params_.max_slope,
		params_.max_error_square, params_.long_threshold, params_.max_long_height, params_.max_start_height,
		params_.sensor_height));
}

void GroundSegmentation::InsertPoints(const PointCloud& cloud) {
	std::vector<std::thread> threads(params_.n_threads);
	const size_t points_per_thread = cloud.size() / params_.n_threads;
	// Launch threads.
	for (size_t i = 0U; i < params_.n_threads - 1U; ++i) {
		const size_t start_index = i * points_per_thread;
		const size_t end_index = (i + 1) * points_per_thread;
		threads[i] = std::thread(&GroundSegmentation::InsertionThread, this,
		                         cloud, start_index, end_index);
	}
	// Launch last thread which might have more points than others.
	const size_t start_index = (params_.n_threads - 1U) * points_per_thread;
	const size_t end_index = cloud.size();
	threads[params_.n_threads - 1U] =
			std::thread(&GroundSegmentation::InsertionThread, this, cloud, start_index, end_index);
	// Wait for threads to finish.
	for (auto& thread : threads) {
		thread.join();
	}
}

void GroundSegmentation::InsertionThread(const PointCloud& cloud, const size_t start_index, const size_t end_index) {
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
			segments_[segment_index_clamped][bin_index].AddPoint(range, point.z);
			bin_index_[i] = std::make_pair(segment_index_clamped, bin_index);
		} else {
			bin_index_[i] = std::make_pair<int, int>(-1, -1);
		}
		segment_coordinates_[i] = Bin::MinZPoint(range, point.z);
	}
}

void GroundSegmentation::GetLines() {
	std::vector<std::thread> thread_vec(params_.n_threads);
	for (size_t i = 0U; i < params_.n_threads - 1U; ++i) {
		const unsigned int start_index = params_.n_segments / params_.n_threads * i;
		const unsigned int end_index = params_.n_segments / params_.n_threads * (i + 1U);
		thread_vec[i] = std::thread(&GroundSegmentation::LineFitThread, this, start_index, end_index);
	}
	const unsigned int start_index = params_.n_segments / params_.n_threads * (params_.n_threads - 1U);
	const unsigned int end_index = params_.n_segments;
	thread_vec[params_.n_threads - 1U] = std::thread(&GroundSegmentation::LineFitThread, this, start_index, end_index);
	for (auto& it : thread_vec) {
		it.join();
	}
}

void GroundSegmentation::LineFitThread(const unsigned int start_index, const unsigned int end_index) {
	for (unsigned int i = start_index; i < end_index; ++i) {
		segments_[i].FitSegmentLines();
	}
}

void GroundSegmentation::AssignCluster(const cvr_lse::cloud_label::Ptr& segmentation) {
	std::vector<std::thread> thread_vec(params_.n_threads);
	const size_t cloud_size = segmentation->label.size();
	for (size_t i = 0U; i < params_.n_threads - 1U; ++i) {
		const unsigned int start_index = cloud_size / params_.n_threads * i;
		const unsigned int end_index = cloud_size / params_.n_threads * (i + 1U);
		thread_vec[i] = std::thread(&GroundSegmentation::AssignClusterThread, this,
			start_index, end_index, segmentation);
	}
	const unsigned int start_index = cloud_size / params_.n_threads * (params_.n_threads - 1U);
	const unsigned int end_index = cloud_size;
	thread_vec[params_.n_threads - 1U] = std::thread(&GroundSegmentation::AssignClusterThread, this,
		start_index, end_index, segmentation);
	for (auto &it : thread_vec) {
		it.join();
	}
}

void GroundSegmentation::AssignClusterThread(const unsigned int& start_index, const unsigned int& end_index,
	const cvr_lse::cloud_label::Ptr& segmentation) {
	const double segment_step = 2 * M_PI / params_.n_segments;
	for (unsigned int i = start_index; i < end_index; ++i) {
		const auto& point_2d = segment_coordinates_[i];
		const int segment_index = bin_index_[i].first;
		if (segment_index >= 0) {
			double dist = segments_[segment_index].VerticalDistanceToLine(point_2d.d, point_2d.z);
			// Search neighboring segments.
			int steps = 1;
			while (dist == -1 && steps * segment_step < params_.line_search_angle) {
				// Fix indices that are out of bounds.
				int index_1 = segment_index + steps;
				while (index_1 >= params_.n_segments) index_1 -= params_.n_segments;
				int index_2 = segment_index - steps;
				while (index_2 < 0) index_2 += params_.n_segments;
				// Get distance to neighboring lines.
				const double dist_1 = segments_[index_1].VerticalDistanceToLine(point_2d.d, point_2d.z);
				const double dist_2 = segments_[index_2].VerticalDistanceToLine(point_2d.d, point_2d.z);
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
				segmentation->isHeightValid[i] = true;
				segmentation->height[i] = static_cast<float>(dist);
				if (dist < params_.max_dist_to_line) {
					segmentation->label[i] = 1U;

				} else {
					segmentation->label[i] = 0U;
				}
			} else {
				segmentation->isHeightValid[i] = true;
				segmentation->height[i] = static_cast<float>(point_2d.z + params_.sensor_height);
				if ((point_2d.d < 6.0) && (point_2d.z < (0.25 - params_.sensor_height))) {
					segmentation->label[i] = 1U;
				} else {
					segmentation->label[i] = 0U;
				}

			}
		}
	}
}
}
}