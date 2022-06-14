//
// Created by ghm on 2021/10/26.
//

#include "post_process.h"
#include <fstream>
namespace cvr_lse {
namespace postprocess {

int num_img = 0;

void convert1to3Channel(cv::Mat& src, cv::Mat& dst) {
	std::vector<cv::Mat> channels;
	for (int i = 0; i < 3; i++) {
		channels.push_back(src);
	}
	cv::merge(&channels[0], channels.size(), dst);
}
void PostProcess::ProcessGridMap(const grid_map::GridMap& grid_map,
	std::vector<std::vector<cv::Point2f>>& find_contour) const {
	cv::Mat occupancy_map_info; // the obstacle information
	cv::Mat free_map_Info; // the free space information
	const auto& size = grid_map.getSize();
	if (grid_map.getSize()(0U) > 0 && grid_map.getSize()(1U) > 1) {
		occupancy_map_info = cv::Mat::zeros(size(0U), size(1U), CV_8UC1);
		free_map_Info = cv::Mat::zeros(size(0U), size(1U), CV_8UC1);
	} else {
		ROS_ERROR("Invaild grid map?");
		return;
	}

	const grid_map::Matrix& data = grid_map["prob"];

	for (grid_map::GridMapIterator iterator(grid_map); !iterator.isPastEnd(); ++iterator) {
		const grid_map::Index index(*iterator);
		const grid_map::Index image_index(iterator.getUnwrappedIndex());
		const float& value = data(index(0), index(1));
		if (std::isfinite(value)) {
			occupancy_map_info.data[image_index(0) * occupancy_map_info.rows + image_index(1)] =
					(value > occupancy_threshold_) ? 255U : 0U;
			free_map_Info.data[image_index(0) * free_map_Info.rows + image_index(1)] =
					(value < free_threshold_) ? 255U : 0U;
		}
	}

	ProcessOccupancyMap(grid_map, occupancy_map_info, find_contour);
}

void PostProcess::ProcessOccupancyMap(const grid_map::GridMap& grid_map, const cv::Mat& occupancy_img,
	std::vector<std::vector<cv::Point2f>>& find_contour) {
	static int num = 0;
	cv::Mat out;
	cv::Mat element_one = getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
	cv::dilate(occupancy_img, out, element_one);
	cv::erode(out, out, element_one);

	cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	cv::dilate(out, out, element);

	std::vector<cv::Vec4i> hierarchy;
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(out, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	std::vector<ConvexHull> convex_hull;
	ProcessContourInfo(contours, convex_hull);
	find_contour.clear();

	for (const auto& convex_single : convex_hull) {
		std::vector<cv::Point2f> points_temp;
		for (const auto& point : convex_single) {
			grid_map::Position pt;
			grid_map.getPosition(grid_map::Index(point.y, point.x), pt);
			points_temp.emplace_back(pt.x(), pt.y());
		}
		std::vector<cv::Point2f> final_contour;
		cv::convexHull(points_temp, final_contour);
		find_contour.emplace_back(final_contour);
	}
	num_img++;
}

void PostProcess::ProcessContourInfo(std::vector<std::vector<cv::Point>>& contours,
	std::vector<ConvexHull>& convex_hull) {
	convex_hull.clear();
	for(auto& contour : contours) {
		std::vector<ConvexHull> convex_hull_temp;

		SimplifyContour(contour);
		if (contour.size() <= 3U) {
			convex_hull.emplace_back(contour);
			continue;
		}

		std::vector<cxd::Vertex> vertices;
		for (auto iter = contour.rbegin(); iter != contour.rend(); ++iter) {
			vertices.emplace_back(cxd::Vec2({static_cast<float>(iter->x), static_cast<float>(iter->y)}));
		}
		cxd::ConcavePolygon concave_polygon(vertices);
		concave_polygon.ConvexDecomp();
		std::vector<cxd::ConcavePolygon> sub_polygon_list;
		concave_polygon.ReturnLowestLevelPolys(sub_polygon_list);
		for (const auto& polygon : sub_polygon_list) {
			ConvexHull convex_temp;
			convex_temp.clear();
			const auto& polygon_vertices = polygon.GetVertices();
			for (const auto& point : polygon_vertices) {
				convex_temp.emplace_back(static_cast<int>(point.position.x), static_cast<int>(point.position.y));
			}
			convex_hull_temp.emplace_back(convex_temp);
		}
		convex_hull.insert(convex_hull.end(), convex_hull_temp.begin(), convex_hull_temp.end());
	}
}

void PostProcess::GenerateSingleConvexHull(const std::vector<cv::Point>& contour, std::vector<ConvexHull>& convexHull) {
	convexHull.clear();
	if (contour.size() <= 3U) {
		convexHull.emplace_back(contour);
		return;
	}
	ConvexHull convexTemp;
	std::vector<size_t> convexIndex;
	convexTemp.emplace_back(contour[0U]);
	convexTemp.emplace_back(contour[1U]);
	convexIndex.emplace_back(0U);
	convexIndex.emplace_back(1U);
	for (size_t i = 2U; i < contour.size(); ++i) {
		if (JudgePointLeft(contour[i - 2U], contour[i - 1U], contour[i])) {
			convexHull.emplace_back(convexTemp);
			convexTemp.clear();
			convexIndex.clear();
			convexTemp.emplace_back(contour[i - 1U]);
			convexTemp.emplace_back(contour[i]);
			convexIndex.emplace_back(i - 1U);
			convexIndex.emplace_back(i);
		} else {
			convexTemp.emplace_back(contour[i]);
			convexIndex.emplace_back(i);
			if (CheckConvexHullInvaild(convexTemp, contour, convexIndex)) {
				convexTemp.pop_back();
				convexHull.emplace_back(convexTemp);
				convexTemp.clear();
				convexIndex.clear();
				convexTemp.emplace_back(contour[i - 1U]);
				convexTemp.emplace_back(contour[i]);
				convexIndex.emplace_back(i - 1U);
				convexIndex.emplace_back(i);
			}
		}
	}
	convexHull.emplace_back(convexTemp);
}

bool PostProcess::CheckConvexHullInvaild(const std::vector<cv::Point>& contour,
	const std::vector<cv::Point>& checkPoints, const std::vector<size_t>& curIndex) {
	constexpr size_t convexSize = 3U;
	if (contour.size() < convexSize) {
		return false;
	}
	std::vector<cv::Point> finalContour;
	cv::convexHull(contour, finalContour);
	for (size_t i = 0U; i < checkPoints.size(); ++i) {
		auto result = std::find(curIndex.begin(), curIndex.end(), i);
		if (result != curIndex.end()) {
			continue;
		}
		if (cv::pointPolygonTest(finalContour, checkPoints[i], false) == 1U) {
			return true;
		}
	}
	return false;
}

void PostProcess::SimplifyContour(std::vector<cv::Point>& contour) {
	if (contour.size() <= 2U) {
		return;
	}
	std::vector<cv::Point> cur_contour;
	cur_contour.swap(contour);
	// anti clock-wise
	std::deque<std::pair<size_t, size_t>> index_vec;
	index_vec.emplace_back(0U, cur_contour.size() - 1U);
	std::vector<size_t> valid_Index;
	constexpr double outer_threshold = 0.5;
	constexpr double inner_threshold = 1.5;
	valid_Index.emplace_back(0U);
	valid_Index.emplace_back(cur_contour.size() - 1U);
	while (!index_vec.empty()) {
		const auto& index_pair = index_vec.front();
		double a = 0.0;
		double b = 0.0;
		double c = 0.0;
		CalculateLineParameters(cur_contour[index_pair.first], cur_contour[index_pair.second], a, b, c);
		auto const_info = std::sqrt(a * a + b * b);
		double distance_max_inner = 0.0;
		double distance_max_outer = 0.0;
		size_t index_max_inner;
		size_t index_max_outer;
		for (size_t i = index_pair.first + 1U; i < index_pair.second; ++i) {
			auto distance = std::abs(a * cur_contour[i].x + b * cur_contour[i].y + c) / const_info;
			if (JudgePointLeft(cur_contour[index_pair.first], cur_contour[index_pair.second], cur_contour[i])) {
				if (distance > distance_max_outer) {
					distance_max_outer = distance;
					index_max_outer = i;
				}
			} else {
				if (distance > distance_max_inner) {
					distance_max_inner = distance;
					index_max_inner = i;
				}
			}
		}
		if (distance_max_inner > inner_threshold) {
			index_vec.emplace_back(index_pair.first, index_max_inner);
			index_vec.emplace_back(index_max_inner, index_pair.second);
			valid_Index.emplace_back(index_max_inner);
		} else if (distance_max_outer > outer_threshold) {
			index_vec.emplace_back(index_pair.first, index_max_outer);
			index_vec.emplace_back(index_max_outer, index_pair.second);
			valid_Index.emplace_back(index_max_outer);
		} else {
			// do nothing
		}
		index_vec.pop_front();
	}
	std::sort(valid_Index.begin(), valid_Index.end());
	for (const auto& index : valid_Index) {
		contour.emplace_back(cur_contour[index]);
	}
}

void PostProcess::CalculateLineParameters(const cv::Point& p1, const cv::Point& p2, double& a, double& b, double& c) {
	a = p2.y - p1.y;
	b = p1.x - p2.x;
	c = p2.x * p1.y - p1.x * p2.y;
}

bool PostProcess::JudgePointLeft(const cv::Point& p1, const cv::Point& p2, const cv::Point& p3) {
	auto value = (p1.x- p3.x) * (p2.y - p3.y) - (p1.y - p3.y) * (p2.x - p3.x);
	return (value >= 0);
}
}
}