//
// Created by ghm on 2021/10/26.
//

#include "post_process.h"
#include <fstream>
namespace cvr_lse {
namespace postprocess {

void convert1to3Channel(cv::Mat& src, cv::Mat& dst) {
	std::vector<cv::Mat> channels;
	for (int i = 0; i < 3; i++) {
		channels.push_back(src);
	}
	cv::merge(&channels[0], channels.size(), dst);
}
void PostProcess::ProcessGridMap(const grid_map::GridMap& grid_map,
	std::vector<ConvexHullf>& find_contour) const {
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
	std::vector<ConvexHullf>& find_contour) const {
	cv::Mat out;
	cv::Mat element_one = getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
	cv::dilate(occupancy_img, out, element_one);
	cv::erode(out, out, element_one);

	cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	cv::dilate(out, out, element);

	std::vector<cv::Vec4i> hierarchy;
	std::vector<ConvexHulli> contours;
	cv::findContours(out, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	// obtain the obstalce inforamtion in the odom
	std::vector<ConvexHullf> contour_temp;
    for (const auto& convex_single : contours) {
        ConvexHullf points_temp;
        for (const auto& point : convex_single) {
            grid_map::Position pt;
            grid_map.getPosition(grid_map::Index(point.y, point.x), pt);
            points_temp.emplace_back(pt.x(), pt.y());
        }
        contour_temp.emplace_back(points_temp);
    }

	ProcessContourInfo(contour_temp, find_contour);
}

void PostProcess::ProcessContourInfo(std::vector<ConvexHullf>& contours,
	std::vector<ConvexHullf>& convex_hull) const {
	convex_hull.clear();
    constexpr int iterator_num = 50;
    for(auto& contour : contours) {
		std::vector<ConvexHullf> convex_hull_temp;
		SimplifyContour(contour);
		if (contour.size() <= 3U) {
			convex_hull.emplace_back(contour);
			continue;
		}

		std::vector<cxd::Vertex> vertices;
		for (auto iter = contour.rbegin(); iter != contour.rend(); ++iter) {
			vertices.emplace_back(cxd::Vec2({static_cast<float>(iter->x), static_cast<float>(iter->y)}));
		}
        std::queue<std::vector<cxd::Vertex>> open_vertices;
        std::vector<std::vector<cxd::Vertex>> close_vertices;
        if (IsConcavePolygon(vertices)) {
            open_vertices.emplace(vertices);
        } else {
            close_vertices.emplace_back(vertices);
        }
        int current_num = 0;
        while(!open_vertices.empty()) {
            ++current_num;
            auto current_vertices = open_vertices.front();
            open_vertices.pop();
            cxd::ConcavePolygon concavePoly(current_vertices);
            concavePoly.ConvexDecomp();
            const auto& left_vertices = concavePoly.GetLeftVertices();
            const auto& right_vertices = concavePoly.GetRightVertices();
            if (left_vertices.empty() || right_vertices.empty()) {
                close_vertices.emplace_back(current_vertices);
                continue;
            }
            if ((left_vertices.size() <= 3) || (!IsConcavePolygon(left_vertices))) {
                close_vertices.emplace_back(left_vertices);
            } else {
                open_vertices.emplace(left_vertices);
            }

            if ((right_vertices.size() <= 3) || (!IsConcavePolygon(right_vertices))) {
                close_vertices.emplace_back(right_vertices);
            } else {
                open_vertices.emplace(right_vertices);
            }
            if (current_num > iterator_num) {
                break;
            }
        }
        while (!open_vertices.empty()) {
            close_vertices.emplace_back(open_vertices.front());
            open_vertices.pop();
        }

        for (const auto& polygon : close_vertices) {
            ConvexHullf convexTemp;
            convexTemp.clear();
            for (const auto& point : polygon) {
                convexTemp.emplace_back(point.position.x, point.position.y);
            }
            convex_hull_temp.emplace_back(convexTemp);
        }
        convex_hull.insert(convex_hull.end(), convex_hull_temp.begin(),
            convex_hull_temp.end());
	}
}

bool PostProcess::IsConcavePolygon(const std::vector<cxd::Vertex>& vertices) {
    for (size_t i = 0U; i < vertices.size(); ++i) {
        float handedness = cxd::Vertex::GetHandedness(
                vertices[Mod(static_cast<int>(i) - 1,
                             static_cast<int>(vertices.size()))],
                vertices[i],
                vertices[Mod(static_cast<int>(i) + 1,
                             static_cast<int>(vertices.size()))]);
        if (handedness < 0.0f) {
            return true;
        }
    }
    return false;
}

void PostProcess::GenerateSingleConvexHull(const ConvexHulli& contour, std::vector<ConvexHulli>& convexHull) {
	convexHull.clear();
	if (contour.size() <= 3U) {
		convexHull.emplace_back(contour);
		return;
	}
	ConvexHulli convexTemp;
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

bool PostProcess::CheckConvexHullInvaild(const ConvexHulli& contour, const ConvexHulli& checkPoints,
    const std::vector<size_t>& curIndex) {
	constexpr size_t convexSize = 3U;
	if (contour.size() < convexSize) {
		return false;
	}
    ConvexHulli finalContour;
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

void PostProcess::SimplifyContour(ConvexHullf& contour) const {
    // Scale polygons equally firstly
    ExpandContour(contour);
	if (contour.size() <= 2U) {
		return;
	}
    ConvexHullf cur_contour;
	cur_contour.swap(contour);
	// anti clock-wise
	std::deque<std::pair<size_t, size_t>> index_vec;
	index_vec.emplace_back(0U, cur_contour.size() - 1U);
	std::vector<size_t> valid_Index;
	double outer_threshold = 0.5 * param_.map_resolution;
	double inner_threshold = 1.5 * param_.map_resolution;
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

void PostProcess::CalculateLineParameters(const cv::Point2f& p1, const cv::Point2f& p2, double& a, double& b, double& c) {
	a = p2.y - p1.y;
	b = p1.x - p2.x;
	c = p2.x * p1.y - p1.x * p2.y;
}

bool PostProcess::JudgePointLeft(const cv::Point& p1, const cv::Point& p2, const cv::Point& p3) {
	auto value = (p1.x- p3.x) * (p2.y - p3.y) - (p1.y - p3.y) * (p2.x - p3.x);
	return (value >= 0);
}

/**
 * Scale polygons equally
 * final list: the input contour
 */
void PostProcess::ExpandContour(ConvexHullf& final_list) const {
    ConvexHullf contour;
    std::swap(contour, final_list);
    int count = static_cast<int>(contour.size());
    final_list.clear();
    final_list.resize(count);
    ConvexHullf dp_list(count);
    ConvexHullf ndp_list(count);

    for (int i = 0; i < count; ++i) {
        int next = (i == (count - 1) ? 0 : i + 1);
        dp_list[i] = (contour[next] - contour[i]);
        float unit_length = 1.0f / (std::sqrt(dp_list[i].dot(dp_list[i])));
        ndp_list[i] = (dp_list[i] * unit_length);
    }

    for (int i = 0; i < count; ++i) {
        int start_index = ((i == 0) ? (count - 1) : (i - 1));
        int end_index = i;
        auto sin_theta = ndp_list[start_index].cross(ndp_list[end_index]);
        cv::Point2f orient_vector = ndp_list[end_index] - ndp_list[start_index];
        final_list[i].x = contour[i].x + param_.map_resolution * 0.5 / sin_theta * orient_vector.x;
        final_list[i].y = contour[i].y + param_.map_resolution * 0.5 / sin_theta * orient_vector.y;
    }
}

}
}