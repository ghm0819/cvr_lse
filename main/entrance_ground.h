//
// Created by ghm on 2021/12/9.
//

#ifndef CVR_LSE_ENTRANCE_GROUND_H
#define CVR_LSE_ENTRANCE_GROUND_H
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <thread>
#include <deque>
#include <string>

//
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/registration/icp.h>
#include <pcl/search/impl/search.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include "ground_segmentation.h"
namespace cvr_lse {
using namespace ground_segmentation;
class EntranceGround {
public:
	explicit EntranceGround(const ros::NodeHandle &nh) : nh_(nh) {};

	~EntranceGround() = default;

	void PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

	void LeftPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

    void RightPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

    void RearPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

	bool Start();

	void Stop();

	EntranceGround(const EntranceGround &) = delete;

	EntranceGround &operator=(const EntranceGround &) = delete;

	EntranceGround(EntranceGround &&) = delete;

	EntranceGround &operator=(EntranceGround &&) = delete;

private:
    void MainProcess();

    bool LoadParameterForGroundSegmentation();

    bool FindCorrespondingInfo(std::vector<pcl::PointCloud<PointXYZIRT>>& point_cloud_info,
        std_msgs::Header& header_info);

    bool FindCorrespondingBetweenTwoMsg(const sensor_msgs::PointCloud2& point_cloud_main,
        sensor_msgs::PointCloud2& point_cloud_side, std::deque<sensor_msgs::PointCloud2>& point_cloud_msg);

    std::shared_ptr<GroundSegmentation> ground_seg_;

	GroundSegmentationParams ground_segmentation_params_;

    std::thread main_processer_;

    volatile bool start_flag_ = false;

	ros::NodeHandle nh_;

	ros::Publisher ground_label_;

	//
    std::deque<sensor_msgs::PointCloud2> cloud_msg_;

    std::mutex cloud_lock_;

    std::deque<sensor_msgs::PointCloud2> left_cloud_msg_;

    std::mutex left_cloud_lock_;

    std::deque<sensor_msgs::PointCloud2> right_cloud_msg_;

    std::mutex right_cloud_lock_;

    std::deque<sensor_msgs::PointCloud2> rear_cloud_msg_;

    std::mutex rear_cloud_lock_;

#ifdef Ground
    ros::Publisher ground_pub_;
    ros::Publisher obstacle_pub_;

    void VisualizationGroundPoint(const pcl::PointCloud<PointXYZIRT>& cloud,
        const cvr_lse::multi_cloud_label& label_info);
#else
#define VisualizationGroundPoint(cloud, labelInfo);
#endif
};
}

#endif //CVR_LSE_ENTRANCE_GROUND_H
