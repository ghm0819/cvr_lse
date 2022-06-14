//
// Created by ghm on 2021/10/25.
//

#ifndef CVR_LSE_ODOMETRY_H
#define CVR_LSE_ODOMETRY_H
#include <nav_msgs/Odometry.h>
#include <deque>
#include <mutex>

struct PoseInfo {
	double x;
	double y;
	double z;
	double roll;
	double pitch;
	double yaw;
	double time;
	PoseInfo() : x(0.0), y(0.0), z(0.0), roll(0.0), pitch(0.0), yaw(0.0), time(0.0) {};

	PoseInfo(const double xIn, const double yIn, const double zIn, const double rollIn, const double pitchIn,
		const double yawIn, const double timeIn) :
		x(xIn), y(yIn), z(zIn), roll(rollIn), pitch(pitchIn), yaw(yawIn), time(timeIn) {};
};
namespace cvr_lse {
namespace odometry {
class Odometry {
public:
	static Odometry& GetOdometry() {
		static Odometry Odometry;
		return Odometry;
	}

	Odometry() = default;

	~Odometry() = default;

	void SetNewOdometryMsg(const nav_msgs::OdometryConstPtr& odom_msg);

	bool GetPoseInfoFromTimestamp(const double timestamp, PoseInfo& pose_value);

	static void QuaternionToEuler(const geometry_msgs::Pose &cur_msg, PoseInfo& cur_pose); // Z-Y-X Euler angles

public:
	std::deque<PoseInfo> pose_vec_;

	const size_t pose_num_ = 100U;

	std::mutex lock_;
};
}
}
#endif //CVR_LSE_ODOMETRY_H
