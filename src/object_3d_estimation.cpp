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



void msgCallback(const sensor_msgs::ImageConstPtr& msg)
{
  //ROS_ERROR("IN IMAGE CALLBACK");

  cv_bridge::CvImagePtr cv_ptr;
  img1 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;

//  cv::imshow("image", img1);
//  cv::waitKey(1);

  if(ROI_xmax-ROI_xmin > 0 &&  ROI_ymax-ROI_ymin >0){
    cv::Mat cropedImage = img1(cv::Rect(ROI_xmin,ROI_ymin,biggest_width,biggest_height));

    ROS_ERROR("ALMOST PRINTING %d %d", cropedImage.size().width, cropedImage.size().height);
    //    cv::imshow("Closest Car", cropedImage);
    //    cv::waitKey(1);
  }
  else {
    ROS_ERROR("NO IMAGE DETECTED YET");
  }


}


//Header header
//Header image_header
//BoundingBox[] bounding_boxes

//float64 probability
//int64 xmin
//int64 ymin
//int64 xmax
//int64 ymax
//int16 id
//string Class
void cbBoundingBoxes(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
  int biggest_bb = 0;
  for(unsigned long i = 0; i < count_bb-1; i++)
  {
    if(msg->bounding_boxes[i].Class == "car")
    {
      width = (int) msg->bounding_boxes[i].xmax - msg->bounding_boxes[i].xmin;
      height = (int) msg->bounding_boxes[i].ymax - msg->bounding_boxes[i].ymin;
      size_bb = width * height;

      if( size_bb > biggest_bb){
        biggest_bb = size_bb;
        biggest_width = width;
        biggest_height = height;

        ROI_xmin = msg->bounding_boxes[i].xmin;
        ROI_xmax = msg->bounding_boxes[i].xmax;
        ROI_ymin = msg->bounding_boxes[i].ymin;
        ROI_ymax = msg->bounding_boxes[i].ymax;
      }
    }
  }
  ROS_ERROR("BIGGEST BB IS: %d || SIZE IS: %d %d %d", id_bb, biggest_bb, ROI_xmax-ROI_xmin, ROI_ymax-ROI_ymin);




  //ROS_ERROR("IN BOUNDING BOX CALLBACK %d", msg->bounding_boxes[0].xmin);

  //  std::cout<<"Bouding Boxes (header):" << msg->header <<std::endl;
  //  std::cout<<"Bouding Boxes (image_header):" << msg->image_header <<std::endl;
  //  std::cout<<"Bouding Boxes (Class):" << msg->bounding_boxes[0].Class <<std::endl;
  //  std::cout<<"Bouding Boxes (xmin):" << msg->bounding_boxes[0].xmin <<std::endl;
  //  std::cout<<"Bouding Boxes (xmax):" << msg->bounding_boxes[0].xmax <<std::endl;
  //  std::cout<<"Bouding Boxes (ymin):" << msg->bounding_boxes[0].ymin <<std::endl;
  //  std::cout<<"Bouding Boxes (ymax):" << msg->bounding_boxes[0].ymax <<std::endl;
  //  std::cout << "\033[2J\033[1;1H";     // clear terminal


}

void calculate_depthmap (const PointCloudXYZ::Ptr& PclXYZ, cv::Mat& depthMap)
{
    cv::Scalar max_value_float = std::numeric_limits<float>::max();
    depthMap = cv::Mat(left_cam.height, left_cam.width, CV_32FC1, max_value_float);
    //depthMap = cv::Mat(left_cam.height, left_cam.width, CV_16UC1, max_value_float);

    float u, v, z;
    int posFinal_px, posFinal_py;

    /* CAROL
    for (int i = 0; i < PclXYZ->points.size(); i++) {
        if(PclXYZ->points[i].z == PclXYZ->points[i].z) {


            z = PclXYZ->points[i].z * 1000;
            u = (PclXYZ->points[i].x * 1000 * left_cam.fx) / z;
            v = (PclXYZ->points[i].y * 1000 * left_cam.fy) / z;
            posFinal_px = (int)(u + left_cam.cx);
            posFinal_py = (int)(v + left_cam.cy);

            if(posFinal_px > (left_cam.width -1))
                posFinal_px = left_cam.width -1;

            if(posFinal_py > (left_cam.height -1))
                posFinal_py = left_cam.height -1;

            ROS_ERROR("z = %f", z);

           //depthMap.at<float>(posFinal_py, posFinal_px) = z;
        }
    }*/

/*
    //Jorge
    for (int i = 0; i < PclXYZ->points.size(); i++) {
        if(PclXYZ->points[i].z == PclXYZ->points[i].z) {


            z = PclXYZ->points[i].z;
            u = (PclXYZ->points[i].x * left_cam.fx) / z;
            v = (PclXYZ->points[i].y * left_cam.fy) / z;
            posFinal_px = (int)(u + left_cam.cx);
            posFinal_py = (int)(v + left_cam.cy);

            if(posFinal_px >= 0 && posFinal_px < left_cam.width && posFinal_py >= 0 && posFinal_py < left_cam.height && z > 0) {

                float dist = sqrt(pow(PclXYZ->points[i].x,2) + pow(PclXYZ->points[i].y,2) + pow(PclXYZ->points[i].z,2));

                depthMap.at<uint16_t>(posFinal_py, posFinal_px) = (uint16_t)(65535 - std::min(dist*655.35*1.5, 55535.0));
            }
        }
    }*/


    //MEU
    for (int i = 0; i < PclXYZ->points.size(); i++) {
        if(PclXYZ->points[i].z == PclXYZ->points[i].z) {

            z = PclXYZ->points[i].z * 1000;

            u = (PclXYZ->points[i].x * 1000 * left_cam.fx) / z;
            v = (PclXYZ->points[i].y * 1000 * left_cam.fy) / z;
            posFinal_px = (int)(u + left_cam.cx);
            posFinal_py = (int)(v + left_cam.cy);

            if(posFinal_px >= 0 && posFinal_px < left_cam.width && posFinal_py >= 0 && posFinal_py < left_cam.height && z > 0) {
                ROS_ERROR("z = %f", z/1000);


                depthMap.at<float>(posFinal_py, posFinal_px) = z;
            }
        }
    }



    depthMap.convertTo(depthMap, CV_8UC1);
    cv::imshow("Depth Map", depthMap);
    cv::waitKey(1);


}

void callback_img_pcl (const sensor_msgs::ImageConstPtr& msg_img, const sensor_msgs::PointCloud2ConstPtr& msg_pcl)
{

    cv::Mat leftimg = cv_bridge::toCvShare(msg_img, "bgr8")->image;

    cv::imshow("img_orig", leftimg);
    cv::waitKey(1);


    //Interessa a ordem q se faz isto? aka primeiro transform e dps fromrosmsg ou ao contrario?
    sensor_msgs::PointCloud2 result;
    pcl_ros::transformPointCloud(frame_id_img, *msg_pcl, result, *tf_listener);

    msg_cloudXYZ.reset(new PointCloudXYZ);
    pcl::fromROSMsg(result, *msg_cloudXYZ);

    cv::Mat depthMap;
    calculate_depthmap(msg_cloudXYZ, depthMap);

}



//Header header
//int8 count
void callback_CloserCar(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg_BB,
                        const sensor_msgs::PointCloud2ConstPtr& msg_pcl,
                        const sensor_msgs::ImageConstPtr& msg_DI){


  //Interessa a ordem q se faz isto? aka primeiro transform e dps fromrosmsg ou ao contrario?
  sensor_msgs::PointCloud2 result;
  pcl_ros::transformPointCloud(frame_id_img, *msg_pcl, result, *tf_listener);

  msg_cloudXYZ.reset(new PointCloudXYZ);
  pcl::fromROSMsg(result, *msg_cloudXYZ);

  cv::Mat depthMap;
  calculate_depthmap(msg_cloudXYZ, depthMap);



  // Bounding Boxes
  int biggest_bb = 0;
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

        ROI_xmin = msg_BB->bounding_boxes[i].xmin;
        ROI_xmax = msg_BB->bounding_boxes[i].xmax;
        ROI_ymin = msg_BB->bounding_boxes[i].ymin;
        ROI_ymax = msg_BB->bounding_boxes[i].ymax;
      }
    }
  }
  //ROS_ERROR("BIGGEST BB IS: %d || SIZE IS: %d %d %d", id_bb, biggest_bb, ROI_xmax-ROI_xmin, ROI_ymax-ROI_ymin);
  ROS_ERROR("BIGGEST BB IS: %d", size_bb);



  // Image
  cv_bridge::CvImagePtr cv_ptr;
  img1 = cv_bridge::toCvCopy(msg_DI, sensor_msgs::image_encodings::BGR8)->image;

    cv::imshow("image", img1);
    cv::waitKey(1);

  if(ROI_xmax-ROI_xmin > 0 &&  ROI_ymax-ROI_ymin >0){
    cv::Mat cropedImage = img1(cv::Rect(ROI_xmin,ROI_ymin,biggest_width,biggest_height));

    ROS_ERROR("ALMOST PRINTING %d %d", cropedImage.size().width, cropedImage.size().height);
//    cv::imshow("Closest Car", cropedImage);
//    cv::waitKey(1);
//    cv::destroyWindow("Closest Car");
  }
  else {
    ROS_ERROR("NO IMAGE DETECTED YET");
  }

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_3d_estimation");
  ros::NodeHandle nh;
  ros::NodeHandle n_public;
  image_transport::ImageTransport it(n_public);

  tf_listener = new tf::TransformListener();

  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> sub_BoundingBox(n_public, "/objects/left/bounding_boxes", 100);
  //message_filters::Subscriber<darknet_ros_msgs::ObjectCount> sub_foundObject(n_public, "/objects/left/found_object", 100);
  //message_filters::Subscriber<sensor_msgs::Image> sub_DetectionImage(n_public, "/objects/left/detection_image", 100);
  message_filters::Subscriber<sensor_msgs::Image> sub_DetectionImage(n_public, "/stereo/left/image_rect_color", 100);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_pclrgb(n_public, "/velodyne_points", 100);


  //  ros::Subscriber sub1 = n_public.subscribe("/objects/left/bounding_boxes", 1, cbBoundingBoxes);
  //  ros::Subscriber sub2 = n_public.subscribe("/objects/left/found_object", 1, cbFoundObject);
  //  image_transport::Subscriber sub = it.subscribe("/objects/left/detection_image", 1, msgCallback);


  // -------------------------- // A 2 // --------------------------

  //message_filters::Subscriber<sensor_msgs::Image> sub_imgleft(n_public, "/stereo/left/image_rect_color", 1);

  //ros::Subscriber sub_pclrgb = n_public.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, pclCB);



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
  typedef message_filters::sync_policies::ApproximateTime<darknet_ros_msgs::BoundingBoxes, sensor_msgs::PointCloud2, sensor_msgs::Image> ApproximatePolicy;
  message_filters::Synchronizer<ApproximatePolicy> sync(ApproximatePolicy(10), sub_BoundingBox, sub_pclrgb, sub_DetectionImage);
  sync.registerCallback(boost::bind(&callback_CloserCar, _1, _2, _3));


  ros::Rate loop_rate(10);
  while (ros::ok())
  {

    ros::spinOnce();

    //ROS_ERROR("WHILE");

    loop_rate.sleep();
  }
}
