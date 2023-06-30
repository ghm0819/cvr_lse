//
// Created by ghm on 2021/10/26.
//

#ifndef CVR_LSE_POST_PROCESS_H
#define CVR_LSE_POST_PROCESS_H

#include <ros/ros.h>

#include "grid_map_core.hpp"
#include "iterators/iterators.hpp"
#include "scan_point.h"
#include "concave_polygon.h"

#include <opencv2/opencv.hpp>
#include <utility>

namespace cvr_lse {
struct CvrLseParameters {
	CvrLseParameters() :
		offset_x(0.0),
		offset_y(10.0),
		map_width(100),
		map_height(100),
		map_output_width(80),
		map_output_height(80),
		map_resolution(0.2),
		occupancy_factor(0.7),
		free_factor(0.4),
		occupancy_threshold(0.95),
		free_threshold(0.3),
		horizontal_resolution(0.2),
		max_odd_threshold(20.0),
		min_odd_threshold(-10.0),
		max_distance(100.0) {}

	double offset_x;
	double offset_y;
	double map_width;
	double map_height;
	double map_output_width;
	double map_output_height;
	double map_resolution;
	double occupancy_factor;
	double free_factor;
	double occupancy_threshold;
	double free_threshold;
	double horizontal_resolution;
	double max_odd_threshold;
	double min_odd_threshold;
	double max_distance;
	std::vector<double> extrinsic_rot(4, 0.0);
	std::vector<double> extrinsic_trans(3, 0.0);

    	std::vector<double> extrinsic_rot_left(4, 0.0);
    	std::vector<double> extrinsic_trans_left(3, 0.0);

    	std::vector<double> extrinsic_rot_right(4, 0.0);
    	std::vector<double> extrinsic_trans_right(3, 0.0);

   	std::vector<double> extrinsic_rot_rear(4, 0.0);
    	std::vector<double> extrinsic_trans_rear(3, 0.0);
};

enum GridStatus {
	FREE,
	OCCUPANCY,
	UNKNOWN
};
namespace postprocess {

using ConvexHulli = std::vector<cv::Point>;
using ConvexHullf = std::vector<cv::Point2f>;

class PostProcess {
public:
	PostProcess(const ros::NodeHandle &nh, CvrLseParameters params) :
			nh_(nh),
			param_(std::move(params)) {
		InitialThresholdInfo();
	}

	~PostProcess() = default;

	void ProcessGridMap(const grid_map::GridMap& grid_map, std::vector<std::vector<cv::Point2f>>& find_contour) const;

private:
	inline void InitialThresholdInfo() {
		occupancy_threshold_ = std::log(param_.occupancy_threshold / (1.0 - param_.occupancy_threshold));
		free_threshold_ = std::log(param_.free_threshold / (1.0 - param_.free_threshold));
	}

	void ProcessOccupancyMap(const grid_map::GridMap& grid_map, const cv::Mat& occupancy_img,
        std::vector<std::vector<cv::Point2f>>& find_contour) const;

	void ProcessContourInfo(std::vector<ConvexHullf>& contours, std::vector<ConvexHullf>& convex_hull) const;

	static void GenerateSingleConvexHull(const ConvexHulli& contour, std::vector<ConvexHulli>& convexHull);

	static void CalculateLineParameters(const cv::Point2f& p1, const cv::Point2f& p2, double& a, double& b, double& c);

	static bool JudgePointLeft(const cv::Point& p1, const cv::Point& p2, const cv::Point& p3);

	static bool CheckConvexHullInvaild(const ConvexHulli& contour, const ConvexHulli& checkPoints,
		const std::vector<size_t>& curIndex);

    static bool IsConcavePolygon(const std::vector<cxd::Vertex>& vertices);

    static int Mod(const int x, const int m) {
        int r = x % m;
        return ((r < 0) ? (r + m) : r);
    }

    void SimplifyContour(ConvexHullf& contour) const;

    void ExpandContour(ConvexHullf& contour) const;

	ros::NodeHandle nh_;

	CvrLseParameters param_;

	double occupancy_threshold_{};

	double free_threshold_{};
};
}
}
#endif //CVR_LSE_POST_PROCESS_H
