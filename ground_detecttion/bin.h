#ifndef GROUND_DETECTION_BIN_H_
#define GROUND_DETECTION_BIN_H_

#include <atomic>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct PointXYZIRT {
	PCL_ADD_POINT4D; // quad-word XYZ
	float intensity; ///< laser intensity reading
	uint16_t ring;   ///< laser ring number
	double timestamp;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
	(float, x, x)
	(float, y, y)
	(float, z, z)
	(float, intensity, intensity)
	(uint16_t, ring, ring)
	(double, timestamp, timestamp))

namespace cvr_lse {
namespace ground_segmentation {
class Bin {
public:
	struct MinZPoint {
		MinZPoint() : z(0), d(0) {}

		MinZPoint(const double& d, const double& z) : z(z), d(d) {}

		inline bool operator == (const MinZPoint& comp) const {
			return ((z == comp.z) && (d == comp.d));
		}

		double z;
		double d;
	};

	Bin();

	Bin(const Bin& segment);

	void AddPoint(const double& d, const double& z);

	MinZPoint GetMinZPoint();

	inline bool HasPoint() {
		return has_point_;
	}

private:
	std::atomic<bool> has_point_ {};
	std::atomic<double> min_z_ {};
	std::atomic<double> min_z_range_ {};
};
}
}
#endif /* GROUND_DETECTION_BIN_H_ */
