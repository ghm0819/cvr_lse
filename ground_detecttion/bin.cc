#include "bin.h"

#include <limits>
namespace cvr_lse {
namespace ground_segmentation {

Bin::Bin() :
	min_z_(std::numeric_limits<double>::max()),
	has_point_(false) {}

Bin::Bin(const Bin& bin) :
	min_z_(std::numeric_limits<double>::max()),
	has_point_(false) {}

void Bin::AddPoint(const double& d, const double& z) {
	has_point_ = true;
	if (z < min_z_) {
		min_z_ = z;
		min_z_range_ = d;
	}
}

Bin::MinZPoint Bin::GetMinZPoint() {
	MinZPoint point;
	if (has_point_) {
		point.z = min_z_;
		point.d = min_z_range_;
	}
	return point;
}

}
}