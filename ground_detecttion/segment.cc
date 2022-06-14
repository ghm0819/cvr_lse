#include "segment.h"
namespace cvr_lse {
namespace ground_segmentation {
Segment::Segment(const size_t& n_bins, const double& max_slope, const double& max_error,
		const double& long_threshold, const double& max_long_height, const double& max_start_height,
		const double& sensor_height) :
		bins_(n_bins),
		max_slope_(max_slope),
		max_error_(max_error),
		long_threshold_(long_threshold),
		max_long_height_(max_long_height),
		max_start_height_(max_start_height),
		sensor_height_(sensor_height) {}

void Segment::FitSegmentLines() {
	auto line_start = bins_.begin();
	while (!line_start->HasPoint()) {
		++line_start;
		if (line_start == bins_.end()) {
			return;
		}
	}
	// Fill lines.
	bool is_long_line = false;
	double cur_ground_height = -sensor_height_;
	std::list<Bin::MinZPoint> current_line_points(1, line_start->GetMinZPoint());
	LocalLine cur_line = std::make_pair(0, 0);
	for (auto line_iter = line_start + 1; line_iter != bins_.end(); ++line_iter) {
		if (line_iter->HasPoint()) {
			Bin::MinZPoint cur_point = line_iter->GetMinZPoint();
      		// judge invalid point
      		double cure_height = cur_ground_height;
      		if (current_line_points.size() > 2U) {
		        cure_height = cur_line.first * cur_point.d + cur_line.second;
      		}
      		if (cur_point.z > (cure_height + sensor_height_ + 0.3)) {
	      		continue;
      		}
			if (cur_point.d - current_line_points.back().d > long_threshold_) {
				is_long_line = true;
			}
			if (current_line_points.size() >= 2) {
				// Get expected z value to possibly reject far away points.
				double expected_z = std::numeric_limits<double>::max();
				if (is_long_line && (current_line_points.size() > 2)) {
					expected_z = cur_line.first * cur_point.d + cur_line.second;
				}
				current_line_points.emplace_back(cur_point);
				cur_line = FitLocalLine(current_line_points);
				const double error = GetMaxError(current_line_points, cur_line);
				// Check if not a good line.
				if ((error > max_error_) || (std::fabs(cur_line.first) > max_slope_) ||
				    (is_long_line && (std::fabs(expected_z - cur_point.z) > max_long_height_))) {
					// Add line until previous point as ground.
					current_line_points.pop_back();
					// Don't let lines with 2 base points through.
					if (current_line_points.size() >= 3) {
						const LocalLine new_line = FitLocalLine(current_line_points);
						lines_.emplace_back(LocalLineToLine(new_line, current_line_points));
						cur_ground_height = new_line.first * current_line_points.back().d + new_line.second;
					}
					// Start new line.
					is_long_line = false;
					current_line_points.erase(current_line_points.begin(), --current_line_points.end());
					--line_iter;
				}
			} else {
				// Not enough points.
				if (cur_point.d - current_line_points.back().d < long_threshold_ &&
				    /*std::fabs*/(current_line_points.back().z - cur_ground_height) < max_start_height_) {
					// Add point if valid.
					current_line_points.emplace_back(cur_point);
				} else {
					// Start new line.
					current_line_points.clear();
					current_line_points.emplace_back(cur_point);
				}
			}
		}
	}
	// Add last line.
	if (current_line_points.size() > 2) {
		const LocalLine new_line = FitLocalLine(current_line_points);
		lines_.emplace_back(LocalLineToLine(new_line, current_line_points));
	}
}

Segment::Line Segment::LocalLineToLine(const LocalLine& local_line, const std::list<Bin::MinZPoint>& line_points) {
	Line line;
	const double first_d = line_points.front().d;
	const double second_d = line_points.back().d;
	const double first_z = local_line.first * first_d + local_line.second;
	const double second_z = local_line.first * second_d + local_line.second;
	line.first.z = first_z;
	line.first.d = first_d;
	line.second.z = second_z;
	line.second.d = second_d;
	return line;
}

double Segment::VerticalDistanceToLine(const double& d, const double& z) {
	static const double kMargin = 0.1;
	double distance = -1;
	for (const auto &line : lines_) {
		if ((line.first.d - kMargin < d) && (line.second.d + kMargin > d)) {
			const double delta_z = line.second.z - line.first.z;
			const double delta_d = line.second.d - line.first.d;
			const double expected_z = (d - line.first.d) / delta_d * delta_z + line.first.z;
			distance = std::fabs(z - expected_z);
		}
	}
	return distance;
}

double Segment::GetMeanError(const std::list<Bin::MinZPoint>& points, const LocalLine& line) {
	double error_sum = 0;
	for (const auto &point : points) {
		const double residual = (line.first * point.d + line.second) - point.z;
		error_sum += residual * residual;
	}
	return error_sum / static_cast<double>(points.size());
}

double Segment::GetMaxError(const std::list<Bin::MinZPoint>& points, const LocalLine& line) {
	double max_error = 0;
	for (const auto& point : points) {
		const double residual = (line.first * point.d + line.second) - point.z;
		const double error = residual * residual;
		if (error > max_error) max_error = error;
	}
	return max_error;
}

Segment::LocalLine Segment::FitLocalLine(const std::list<Bin::MinZPoint>& points) {
	const auto n_points = points.size();
	Eigen::MatrixXd X(n_points, 2);
	Eigen::VectorXd Y(n_points);
	unsigned int counter = 0;
	for (const auto& point : points) {
		X(counter, 0) = point.d;
		X(counter, 1) = 1;
		Y(counter) = point.z;
		++counter;
	}
	Eigen::VectorXd result = X.colPivHouseholderQr().solve(Y);
	LocalLine line_result;
	line_result.first = result(0);
	line_result.second = result(1);
	return line_result;
}

}
}