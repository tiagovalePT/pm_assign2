#ifndef OBJECT_3D_ESTIMATION_H
#define OBJECT_3D_ESTIMATION_H

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <cmath>
#include "std_msgs/Bool.h"
#include "opencv2/opencv.hpp"
#include <image_transport/image_transport.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include "opencv2/core.hpp"
#include <opencv2/core/types.hpp>
#include <sensor_msgs/Image.h>
#include <cmath>

// Includes das mensagens do darknet_ros
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/ObjectCount.h>


#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include "tf2_ros/static_transform_broadcaster.h"
#include <tf/transform_listener.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/common/centroid.h>

cv::Mat img1, img_car, cropedImage, cropedDepthMap;
int ROI_xmin, ROI_xmax, ROI_ymin, ROI_ymax;
int ROI_xmin_closest, ROI_xmax_closest, ROI_ymin_closest, ROI_ymax_closest;
int width_camera, height_camera, closerX, closerY;
int count_BB = 0, sum_sizesBB = 0, width, height, size_bb, best_bb_id, best_bb_size;
float min_dp = 100000000, dp;
int biggest_width, biggest_height;

darknet_ros_msgs::BoundingBoxes BB_cars;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;

PointCloudXYZ::Ptr cloud_for_centroid;
PointCloudXYZ::Ptr cloud_of_centroid;

cv::Mat depthMap_global;

cv::Mat leftimg;

cv::Mat cropped_depthMap;

std::string frame_id_img, frame_id_pointCloud;

tf::TransformListener *tf_listener;

struct cam_param {
  double fx, fy, cx, cy;
  double height, width;
};

int best_BB [5];
int flag = 0;

cam_param left_cam;

ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher publisherCloudXYZ;

#endif // OBJECT_3D_ESTIMATION_H
