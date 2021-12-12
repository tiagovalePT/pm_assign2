#ifndef OBJECT_3D_ESTIMATION_H
#define OBJECT_3D_ESTIMATION_H

#include <ros/ros.h>
#include <cmath>
#include "opencv2/opencv.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// Includes das mensagens do darknet_ros
#include <darknet_ros_msgs/BoundingBoxes.h>
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include "tf2_ros/static_transform_broadcaster.h"
#include <tf/transform_listener.h>

#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#define THRESHOLD_CENTROID 10

// Imagens usadas
cv::Mat img1, img_car, cropedImage, cropedDepthMap;

// Coordenadas da region of interest
int ROI_xmin, ROI_xmax, ROI_ymin, ROI_ymax;
int ROI_xmin_closest, ROI_xmax_closest, ROI_ymin_closest, ROI_ymax_closest;

// Tamanho em pixels da imagem da camera
int width_camera, height_camera, closerX, closerY;

//Variaveis Auxiliares e contadores
int count_BB = 0, sum_sizesBB = 0, width, height, size_bb, best_bb_id, best_bb_size;
float min_dp = 100000000, dp;
int biggest_width, biggest_height;
int centerX, centerY;
float fov_x, fov_y;
int centroidX = 0, centroidY = 0;

int width_img = 0, height_img = 0;

// Msg BoundingBoxes que guarda todas as bounding boxes que têm como classe "car"
darknet_ros_msgs::BoundingBoxes BB_cars;

// Point Clouds XYZ e XYZRGB
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;

PointCloudXYZRGB::Ptr cloudRGB_global (new PointCloudXYZRGB);

PointCloudXYZ::Ptr cloud_for_centroid;
PointCloudXYZ::Ptr cloud_of_centroid;
PointCloudXYZRGB::Ptr msg_cloud_xyz_rgb;

// Depth map
cv::Mat depthMap_global;

// Imagem da left camera
cv::Mat leftimg;

// Depth map cortada
cv::Mat cropped_depthMap;

std::string frame_id_img, frame_id_pointCloud;

// Listener da transformada
tf::TransformListener *tf_listener;

// Struct dos parâmetros da camera
struct cam_param {
  double fx, fy, cx, cy;
  double height, width;
};

int best_BB [5];
int flag = 0;

cam_param left_cam;

// Publishers
ros::Publisher pubPose;
ros::Publisher pubNearestCar;
ros::Publisher publisherCloudXYZ, publisherCloudXYZRGB;
image_transport::Publisher pubDepthmap;
ros::Publisher pubSizeCar;

#endif // OBJECT_3D_ESTIMATION_H
