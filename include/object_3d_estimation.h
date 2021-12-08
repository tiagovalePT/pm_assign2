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
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include "tf2_ros/static_transform_broadcaster.h"
#include <tf/transform_listener.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

cv::Mat img1, img_car;
int count_bb, ROI_xmin, ROI_xmax, ROI_ymin, ROI_ymax;
int size_bb, biggest_width, biggest_height, width, height, id_bb;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
PointCloudXYZ::Ptr msg_cloudXYZ;

std::string frame_id_img, frame_id_pointCloud;

tf::TransformListener *tf_listener;

struct cam_param {
  double fx, fy, cx, cy;
  double height, width;
};

cam_param left_cam;

#endif // OBJECT_3D_ESTIMATION_H
