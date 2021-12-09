#include <object_3d_estimation.h>
//Node [/dark2/darknet_ros]
//Publications:
// * /dark2/darknet_ros/check_for_objects/feedback [darknet_ros_msgs/CheckForObjectsActionFeedback]
// * /dark2/darknet_ros/check_for_objects/result [darknet_ros_msgs/CheckForObjectsActionResult]
// * /dark2/darknet_ros/check_for_objects/status [actionlib_msgs/GoalStatusArray]
// * /objects/left/bounding_boxes [darknet_ros_msgs/BoundingBoxes]
// * /objects/left/detection_image [sensor_msgs/Image]
// * /objects/left/found_object [darknet_ros_msgs/ObjectCount]
// * /rosout [rosgraph_msgs/Log]

//Subscriptions:
// * /clock [rosgraph_msgs/Clock]
// * /dark2/darknet_ros/check_for_objects/cancel [unknown type]
// * /dark2/darknet_ros/check_for_objects/goal [unknown type]
// * /stereo/left/image_rect_color [sensor_msgs/Image]

//Services:
// * /dark2/darknet_ros/get_loggers
// * /dark2/darknet_ros/set_logger_level

void calculate_depthmap (const PointCloudXYZ::Ptr& PclXYZ)
{
    //Meu v2

//    msg_cloudXYZRGB->header.frame_id = "vision_frame";
//    pcl_conversions::toPCL(ros::Time::now(), msg_cloudXYZRGB->header.stamp);
//    msg_cloudXYZRGB->height = 1;

    PointCloudXYZ::Ptr cloud_toPublish (new PointCloudXYZ);
    cloud_toPublish->header.frame_id = "vision_frame";
    pcl_conversions::toPCL(ros::Time::now(), cloud_toPublish->header.stamp);
    cloud_toPublish->height = 1;


    cv::Mat depthMap = cv::Mat::zeros(left_cam.height, left_cam.width, CV_8UC1);

    cv::Mat pxI_toPublish = cv::Mat::zeros(left_cam.height, left_cam.width, CV_32FC1);

    pcl::PointXYZ minpt, maxpt;
    pcl::getMinMax3D(*PclXYZ,minpt,maxpt);

//    ROS_ERROR("Min x: %f", minpt.x);
//    ROS_ERROR("Min y: %f", minpt.y);
//    ROS_ERROR("Min z: %f", minpt.z);
//    ROS_ERROR("Max x: %f", maxpt.x);
//    ROS_ERROR("Max y: %f", maxpt.y);
//    ROS_ERROR("Max z: %f", maxpt.z);




    int u, v, posFinal_px, posFinal_py, pxValue;

    double x, y, z;

    for (int i = 0; i < PclXYZ->points.size(); i++) {
        if(PclXYZ->points[i].z == PclXYZ->points[i].z) {
            x = PclXYZ->points[i].x;
            y = PclXYZ->points[i].y;
            z = PclXYZ->points[i].z;

            u = (left_cam.fx * x + left_cam.cx * z) / z;
            v = (left_cam.fy * y + left_cam.cy * z) / z;


            pxValue = 0;
            pxValue = 255 - (z-minpt.z)/(maxpt.z-minpt.z)*200; //mudar isto para as contas q eu tinha se tiver bem

            if(u >= 0 && u < left_cam.width && v >= 0 && v < left_cam.height && z >= 0) {
                depthMap.at<uint8_t>(v, u) = pxValue;
                pxI_toPublish.at<float>(v, u) = z;

                pcl::PointXYZ pointPCL;
                pointPCL.x = x;
                pointPCL.y = y;
                pointPCL.z = z;
                cloud_toPublish->points.push_back(pointPCL);

            }
        }
    }

    cloud_toPublish->width = cloud_toPublish->points.size();
    publisherCloudXYZ.publish(cloud_toPublish);


    cv::imshow("Depth Map", depthMap);
    cv::waitKey(1);

//    cv::imshow("To publish", pxI_toPublish);
//    cv::waitKey(1);
}

void callback_img_pcl (const sensor_msgs::ImageConstPtr& msg_img, const sensor_msgs::PointCloud2ConstPtr& msg_pcl)
{

    cv::Mat leftimg = cv_bridge::toCvShare(msg_img, "bgr8")->image;

    cv::imshow("img_orig", leftimg);
    cv::waitKey(1);


    //Interessa a ordem q se faz isto? aka primeiro transform e dps fromrosmsg ou ao contrario?
    sensor_msgs::PointCloud2::Ptr result (new sensor_msgs::PointCloud2);
    pcl_ros::transformPointCloud(frame_id_img, *msg_pcl, *result, *tf_listener);

    PointCloudXYZ::Ptr msg_cloudXYZ (new PointCloudXYZ);
    pcl::fromROSMsg(*result, *msg_cloudXYZ);


    calculate_depthmap(msg_cloudXYZ);

}


//Header header
//int8 count
void callback_CloserCar(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg_BB,
                        const sensor_msgs::PointCloud2ConstPtr& msg_pcl,
                        const sensor_msgs::ImageConstPtr& msg_DI,
                        const darknet_ros_msgs::ObjectCount::ConstPtr msg_CO
                        ){


  //Interessa a ordem q se faz isto? aka primeiro transform e dps fromrosmsg ou ao contrario?
  sensor_msgs::PointCloud2 result;
  pcl_ros::transformPointCloud(frame_id_img, *msg_pcl, result, *tf_listener);

  msg_cloudXYZ.reset(new PointCloudXYZ);
  pcl::fromROSMsg(result, *msg_cloudXYZ);

  cv::Mat depthMap;
  calculate_depthmap(msg_cloudXYZ, depthMap);


  count_bb = msg_CO->count;
  int count = 0;
  int greatest_so_far = 0;

  // Bounding Boxes
  if(msg_BB!=NULL){
//    int biggest_bb = 0;

//    width = (int) msg_BB->bounding_boxes[0].xmax - msg_BB->bounding_boxes[0].xmin;
//    height = (int) msg_BB->bounding_boxes[0].ymax - msg_BB->bounding_boxes[0].ymin;
//    best_BB[0] = width * height;

//    for (int i = 0; i < 5; i++)
//    {
//      width = (int) msg_BB->bounding_boxes[i].xmax - msg_BB->bounding_boxes[i].xmin;
//      height = (int) msg_BB->bounding_boxes[i].ymax - msg_BB->bounding_boxes[i].ymin;
//      size_bb = width * height;
//      if (size_bb > greatest_so_far){
//        greatest_so_far = size_bb;

//      }
//     }



    for(unsigned long i = 0; i < count_bb-1; i++)
    {
      if(msg_BB->bounding_boxes[i].Class == "car")
      {
        width = (int) msg_BB->bounding_boxes[i].xmax - msg_BB->bounding_boxes[i].xmin;
        height = (int) msg_BB->bounding_boxes[i].ymax - msg_BB->bounding_boxes[i].ymin;
        size_bb = width * height;

        if( size_bb > biggest_bb){
          biggest_bb = size_bb;
          biggest_width = width;
          biggest_height = height;

//          best_BB[count] = msg_BB->bounding_boxes[i].id;
//          count++;

//          if(biggest_bb < lowest_BB){
//            lowest_BB = biggest_bb;
//          }

          ROI_xmin = msg_BB->bounding_boxes[i].xmin;
          ROI_xmax = msg_BB->bounding_boxes[i].xmax;
          ROI_ymin = msg_BB->bounding_boxes[i].ymin;
          ROI_ymax = msg_BB->bounding_boxes[i].ymax;
        }
      }
    }
  }
  //size_bb = msg_BB->bounding_boxes[1].xmin;
  ROS_ERROR("BIGGEST BB IS: %d || SIZE IS: %d %d %d", id_bb, biggest_bb, ROI_xmax-ROI_xmin, ROI_ymax-ROI_ymin);
  //ROS_ERROR("BIGGEST BB IS: %d %d %d %d %d", best_BB[0], best_BB[1], best_BB[2], best_BB[3], best_BB[4]);



  // Image
  cv_bridge::CvImagePtr cv_ptr;
  img1 = cv_bridge::toCvCopy(msg_DI, sensor_msgs::image_encodings::BGR8)->image;

    //cv::imshow("image", img1);
    //cv::waitKey(1);

  if(ROI_xmax-ROI_xmin > 0 &&  ROI_ymax-ROI_ymin > 0){
    cropedImage = img1(cv::Rect(ROI_xmin,ROI_ymin,biggest_width,biggest_height));

    cropedDepthMap = depthMap(cv::Rect(ROI_xmin,ROI_ymin,biggest_width,biggest_height));

    ROS_ERROR("ALMOST PRINTING %d %d", cropedImage.size().width, cropedImage.size().height);
    //    cv::imshow("Closest Car", cropedImage);
    //    cv::waitKey(1);
    //    cv::destroyWindow("Closest Car");
  }
  else {
    ROS_ERROR("NO IMAGE DETECTED YET");
  }


  // Calculate distance with deph map and ROI
  // depthMap
  width_camera = img1.size().width;
  height_camera = img1.size().height;

  min_dp = 0;

  for(int h = ROI_ymin; h < ROI_ymax; h++){
    for(int w = ROI_xmin; w < ROI_xmax; w++){
      dp = depthMap.at<float>(h, w);
      if(dp < min_dp){
        min_dp = dp;
        closerX = w;
        closerY = h;
      }
    }
  }
  ROS_ERROR("XYZ Closest Point: %d %d", closerX, closerY);

  // Publish
  sensor_msgs::ImagePtr ROI_RGB = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cropedImage).toImageMsg();
  sensor_msgs::ImagePtr ROI_DM = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cropedDepthMap).toImageMsg();
  //depthMap cropedDepthMap

  pub1.publish(ROI_RGB);
  pub2.publish(ROI_DM);



}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_3d_estimation");
  ros::NodeHandle nh;
  ros::NodeHandle n_public;

  tf_listener = new tf::TransformListener();

  image_transport::ImageTransport it(n_public);

//  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> sub_BoundingBox(n_public, "/objects/left/bounding_boxes", 1);
//  message_filters::Subscriber<darknet_ros_msgs::ObjectCount> sub_foundObject(n_public, "/objects/left/found_object", 1);
//  //message_filters::Subscriber<sensor_msgs::Image> sub_DetectionImage(n_public, "/objects/left/detection_image", 100);
//  message_filters::Subscriber<sensor_msgs::Image> sub_DetectionImage(n_public, "/stereo/left/image_rect_color", 1);
//  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_pclrgb(n_public, "/velodyne_points", 1);


  //  ros::Subscriber sub1 = n_public.subscribe("/objects/left/bounding_boxes", 1, cbBoundingBoxes);
  //  ros::Subscriber sub2 = n_public.subscribe("/objects/left/found_object", 1, cbFoundObject);
  //  image_transport::Subscriber sub = it.subscribe("/objects/left/detection_image", 1, msgCallback);


  // -------------------------- // A 2 // --------------------------

  //message_filters::Subscriber<sensor_msgs::Image> sub_imgleft(n_public, "/stereo/left/image_rect_color", 1);

  //ros::Subscriber sub_pclrgb = n_public.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, pclCB);


  //image_transport::Subscriber sub_imgleft = it.subscribe("/stereo/left/image_rect_color", 1, left_imgCB);
  message_filters::Subscriber<sensor_msgs::Image> sub_imgleft(n_public, "/stereo/left/image_rect_color", 1);

  //ros::Subscriber sub_pclrgb = n_public.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, pclCB);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_pclrgb(n_public, "/velodyne_points", 1);

  publisherCloudXYZ = n_public.advertise<PointCloudXYZ>("PCLTESTE",1);


  n_public.getParam("/object_3d_estimation/left_img_frameId", frame_id_img);
  n_public.getParam("/object_3d_estimation/pointCloud_frameId", frame_id_pointCloud);


  boost::shared_ptr<sensor_msgs::CameraInfo const> cam_info;
  sensor_msgs::CameraInfo cam_info_msg;

  while (cam_info == NULL) {

      cam_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/stereo/left/camera_info", ros::Duration(5));

      if(cam_info != NULL){
          cam_info_msg = *cam_info;
          left_cam.fx = cam_info_msg.K[0];
          left_cam.fy = cam_info_msg.K[4];
          left_cam.cx = cam_info_msg.K[2];
          left_cam.cy = cam_info_msg.K[5];
          left_cam.height = cam_info_msg.height;
          left_cam.width = cam_info_msg.width;
          ROS_ERROR("Left cam -> fx=%.2f fy=%.2f cx=%.2f cy=%.2f", left_cam.fx, left_cam.fy, left_cam.cx, left_cam.cy);
      }
      else {
          ROS_ERROR("Error getting left camera intrinsic matrix!");
          ros::Duration(1.0).sleep();
      }
  }
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> ApproximatePolicy;
  message_filters::Synchronizer<ApproximatePolicy> sync(ApproximatePolicy(10), sub_imgleft, sub_pclrgb);
  sync.registerCallback(boost::bind(&callback_img_pcl, _1, _2));

//  typedef message_filters::sync_policies::ApproximateTime<darknet_ros_msgs::BoundingBoxes, sensor_msgs::PointCloud2, sensor_msgs::Image, darknet_ros_msgs::ObjectCount> ApproximatePolicy;
//  message_filters::Synchronizer<ApproximatePolicy> sync(ApproximatePolicy(10), sub_BoundingBox, sub_pclrgb, sub_DetectionImage, sub_foundObject);
//  sync.registerCallback(boost::bind(&callback_CloserCar, _1, _2, _3, _4));

  // Publish
  // Define a publisher object.
  pub1 = n_public.advertise<sensor_msgs::Image>("closest_car/RGB", 1);
  pub2 = n_public.advertise<sensor_msgs::Image>("closest_car/DM", 1);


  ros::spin();
}
