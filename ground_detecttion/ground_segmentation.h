#ifndef GROUND_DETECTION_SEGMENTATION_H_
#define GROUND_DETECTION_SEGMENTATION_H_

#include <mutex>

#include <glog/logging.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "cvr_lse/cloud_label.h"
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
			sensor_height(1.8),
			line_search_angle(0.3) {}

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
	// Height of sensor above ground.
	double sensor_height;
	// How far to search for a line in angular direction [rad].
	double line_search_angle;
};

typedef pcl::PointCloud<PointXYZIRT> PointCloud;

typedef std::pair<PointXYZIRT, PointXYZIRT> PointLine;

class GroundSegmentation {

public:

	explicit GroundSegmentation(const GroundSegmentationParams &params = GroundSegmentationParams());

	void SegmentProcess(const PointCloud &cloud, const cvr_lse::cloud_label::Ptr& segmentation);

	void Reset();

private:

	const GroundSegmentationParams params_;

	std::vector<Segment> segments_;

	std::vector<std::pair<int, int> > bin_index_;

	std::vector<Bin::MinZPoint> segment_coordinates_;

	void AssignCluster(const cvr_lse::cloud_label::Ptr& segmentation);

	void AssignClusterThread(const unsigned int &start_index, const unsigned int &end_index,
		const cvr_lse::cloud_label::Ptr& segmentation);

	void InsertPoints(const PointCloud &cloud);

	void InsertionThread(const PointCloud &cloud, size_t start_index, size_t end_index);

	void GetLines();

	void LineFitThread(unsigned int start_index, unsigned int end_index);
};
}
}
#endif // GROUND_DETECTION_SEGMENTATION_H_
