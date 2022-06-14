#ifndef GROUND_DETECTION_SEGMENTATION_H
#define GROUND_DETECTION_SEGMENTATION_H

#include <list>
#include <map>

#include "bin.h"
namespace cvr_lse {
namespace ground_segmentation {
class Segment {
public:
	typedef std::pair<Bin::MinZPoint, Bin::MinZPoint> Line;

	typedef std::pair<double, double> LocalLine;

private:
	// Parameters. Description in GroundSegmentation.
	std::vector<Bin> bins_;
	const double max_slope_;
	const double max_error_;
	const double long_threshold_;
	const double max_long_height_;
	const double max_start_height_;
	const double sensor_height_;

	std::list<Line> lines_;

	static LocalLine FitLocalLine(const std::list<Bin::MinZPoint>& points);

	static double GetMeanError(const std::list<Bin::MinZPoint>& points, const LocalLine& line);

	static double GetMaxError(const std::list<Bin::MinZPoint>& points, const LocalLine& line);

	static Line LocalLineToLine(const LocalLine& local_line, const std::list<Bin::MinZPoint>& line_points);


public:

	Segment(const size_t& n_bins, const double& max_slope, const double& max_error, const double& long_threshold,
		const double& max_long_height, const double& max_start_height, const double& sensor_height);

	double VerticalDistanceToLine(const double& d, const double& z);

	void FitSegmentLines();

	inline Bin& operator [] (const size_t& index) {
		return bins_[index];
	}

	inline std::vector<Bin>::iterator begin() {
		return bins_.begin();
	}

	inline std::vector<Bin>::iterator end() {
		return bins_.end();
	}
};
}
}
#endif /* GROUND_DETECTION_SEGMENTATION_H */
