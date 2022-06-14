//
// Created by ghm on 2021/10/25.
//

#ifndef CVR_LSE_SCAN_POINT_H
#define CVR_LSE_SCAN_POINT_H

#include <Eigen/Core>
#include <utility>
#include <vector>

namespace cvr_lse {
class ScanPoint {
public:
	ScanPoint() = default;

	ScanPoint(Eigen::Vector3d pointIn, const uint8_t labelIn, const double timestampIn)
			: point_info_lidar_(std::move(pointIn)),
			  label_(labelIn),
			  timestamp_(timestampIn) {};

	ScanPoint(const double xIn, const double yIn, const double zIn, const uint8_t labelIn, const double timestampIn,
	          const bool groundValidIn, const double groundHeightIn, const uint16_t ringIdIn = 0U) {
		point_info_lidar_.x() = xIn;
		point_info_lidar_.y() = yIn;
		point_info_lidar_.z() = zIn;
		label_ = labelIn;
		timestamp_ = timestampIn;
		ground_vaild_ = groundValidIn;
		ground_height_ = groundHeightIn;
		ring_id_ = ringIdIn;
	}

	~ScanPoint() = default;

	inline void SetOdomPointInfo(const Eigen::Vector3d &pointIn) {
		point_info_odom_ = pointIn;
	}

	inline void SetVehiclePointInfo(const Eigen::Vector3d &pointIn) {
		piont_info_veh_ = pointIn;
	}

	inline void SetLidarPointInfo(const Eigen::Vector3d &pointIn) {
		point_info_lidar_ = pointIn;
	}

	inline const Eigen::Vector3d &GetOdomPointInfo() const {
		return point_info_odom_;
	}

	inline const Eigen::Vector3d &GetVehiclePointInfo() const {
		return piont_info_veh_;
	}

	inline const Eigen::Vector3d &GetLidarPointInfo() const {
		return point_info_lidar_;
	}

	inline double x() const {
		return point_info_lidar_.x();
	}

	inline double y() const {
		return point_info_lidar_.y();
	}

	inline double z() const {
		return point_info_lidar_.z();
	}

	inline bool IsGroundPoint() const {
		return (label_ == 1U);
	}

	inline bool IsGroundValid() const {
		return ground_vaild_;
	}

	inline double GetHeightDistance() const {
		return ground_height_;
	}

	inline double GetDistance() const {
		return std::sqrt(point_info_lidar_.x() * point_info_lidar_.x() + point_info_lidar_.y() * point_info_lidar_.y());
	}

	ScanPoint(const ScanPoint &) = default;

	ScanPoint &operator=(const ScanPoint &) = default;

	ScanPoint(ScanPoint &&) = default;

	ScanPoint &operator=(ScanPoint &&) = default;

private:
	Eigen::Vector3d point_info_lidar_;

	Eigen::Vector3d piont_info_veh_;

	Eigen::Vector3d point_info_odom_;

	double ground_height_ = std::numeric_limits<double>::max();

	bool ground_vaild_ = false;

	uint8_t label_{};

	double timestamp_{};

	uint16_t ring_id_{};
};

using ScanRay = std::vector<ScanPoint>;
using ScanAll = std::vector<ScanRay>;
}
#endif //CVR_LSE_SCAN_POINT_H
