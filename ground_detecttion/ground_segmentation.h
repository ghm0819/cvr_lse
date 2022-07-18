#ifndef GROUND_DETECTION_SEGMENTATION_H_
#define GROUND_DETECTION_SEGMENTATION_H_

#include <mutex>
#include <ros/ros.h>
#include <glog/logging.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include "cvr_lse/multi_cloud_label.h"
#include "segment.h"
namespace cvr_lse {
namespace ground_segmentation {
struct GroundSegmentationParams {
	GroundSegmentationParams() :
			r_min_square(1.0 * 1.0),
			r_max_square(80 * 80),
			n_bins(200),
			n_segments(360),
			max_dist_to_line(0.3),
			max_slope(0.3),
			n_threads(4),
			max_error_square(0.09),
			long_threshold(3.0),
			max_long_height(0.3),
			max_start_height(0.3),
            x_range_min(-4.0),
            x_range_max(0.5),
            y_range_min(-1.2),
            y_range_max(1.2) {
        extrinsic_rot_left[0] = 1.0;
        extrinsic_rot_right[0] = 1.0;
        extrinsic_rot_rear[0] = 1.0;
	}

	// Minimum range of segmentation.
	double r_min_square;
	// Maximum range of segmentation.
	double r_max_square;
	// Number of radial bins.
	int n_bins;
	// Number of angular segments.
	int n_segments;
	// Maximum distance to a ground line to be classified as ground.
	double max_dist_to_line;
	// Max slope to be considered ground line.
	double max_slope;
	// Number of threads.
	int n_threads;
	// Max error for line fit.
	double max_error_square;
	// Distance at which points are considered far from each other.
	double long_threshold;
	// Maximum slope for
	double max_long_height;
	// Maximum heigh of starting line to be labelled ground.
	double max_start_height;
	// How far to search for a line in angular direction [rad].
	double line_search_angle;

    std::vector<float> extrinsic_rot_left{4, 0.0};
    std::vector<float> extrinsic_trans_left{3, 0.0};

    std::vector<float> extrinsic_rot_right{4, 0.0};
    std::vector<float> extrinsic_trans_right{3, 0.0};

    std::vector<float> extrinsic_rot_rear{4, 0.0};
    std::vector<float> extrinsic_trans_rear{3, 0.0};

    // Height of sensor above ground.
    std::vector<float> sensor_height{4, 0.0};

    float x_range_min;
    float x_range_max;
    float y_range_min;
    float y_range_max;
};

typedef pcl::PointCloud<PointXYZIRT> PointCloud;

typedef std::pair<PointXYZIRT, PointXYZIRT> PointLine;

enum LidarID {
    Main,
    Left,
    Right,
    Rear,
    LidarSize
};

using SegmentVector = std::vector<Segment>;
using BinIndexVector = std::vector<std::pair<int, int>>;
using SegmentCoordinates = std::vector<Bin::MinZPoint>;
class GroundSegmentation {

public:

	explicit GroundSegmentation(const GroundSegmentationParams &params = GroundSegmentationParams());

	void SegmentProcess(const cvr_lse::multi_cloud_label::Ptr& segmentation,
        std::vector<pcl::PointCloud<PointXYZIRT>>& point_cloud_info);

	void Reset();

private:

	const GroundSegmentationParams params_;

	std::vector<SegmentVector> segments_;
	std::vector<BinIndexVector> bins_indexs_;
    std::vector<SegmentCoordinates> segments_coordinates_;
    std::vector<Eigen::Isometry3f> transform_;

    static void Clear(const cvr_lse::multi_cloud_label::Ptr& segmentation);

    void ObtainRangeInformation();

    void RemoveSelfPointCloud(const Eigen::Isometry3f& transform, pcl::PointCloud<PointXYZIRT>& point_cloud);

	void AssignCluster(const LidarID lidar_id, const size_t point_size,
        const cvr_lse::multi_cloud_label::Ptr& segmentation);

    void AssignClusterThread(const LidarID lidar_id, const unsigned int &start_index, const unsigned int &end_index,
		const cvr_lse::multi_cloud_label::Ptr& segmentation);

    std::vector<cvr_lse::label_point>& ObtainsCorrespondingLabelPoints(const LidarID lidar_id,
        const cvr_lse::multi_cloud_label::Ptr& segmentation);

	void InsertPoints(const PointCloud& cloud, const LidarID lidar_id);

	void InsertionThread(const PointCloud& cloud, const LidarID lidar_id, size_t start_index, size_t end_index);

	void GetLines(const LidarID lidar_id);

	void LineFitThread(const LidarID lidar_id, unsigned int start_index, unsigned int end_index);
};
}
}
#endif // GROUND_DETECTION_SEGMENTATION_H_
