 #include "object_3d_estimation.h"


void calculate_depthmap (const PointCloudXYZ::Ptr& PclXYZ, cv::Mat& output)
{
    //Meu v2

//    msg_cloudXYZRGB->header.frame_id = "vision_frame";
//    pcl_conversions::toPCL(ros::Time::now(), msg_cloudXYZRGB->header.stamp);
//    msg_cloudXYZRGB->height = 1;

    PointCloudXYZ::Ptr cloud_toPublish (new PointCloudXYZ);

    cloud_toPublish->header.frame_id = "vision_frame";
    pcl_conversions::toPCL(ros::Time::now(), cloud_toPublish->header.stamp);
    cloud_toPublish->height = 1;

    // Calculate the FOV of the camera (in x)
//    fov_x = 2 * atan2( left_cam.width, (2*left_cam.fx) );
//    fov_y = 2 * atan2( left_cam.height, (2*left_cam.fy) );

//    ROS_ERROR("FOV CAMERA: %f %f", fov_x, fov_y);

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
    cloud_for_centroid = cloud_toPublish;

    cloud_toPublish->width = cloud_toPublish->points.size();
    publisherCloudXYZ.publish(cloud_toPublish);


//    cv::imshow("Depth Map", depthMap);
//    cv::waitKey(1);

    output = pxI_toPublish;

//    cv::imshow("To publish", pxI_toPublish);
//    cv::waitKey(1);
}







void callback_img_pcl (const sensor_msgs::ImageConstPtr& msg_img, const sensor_msgs::PointCloud2ConstPtr& msg_pcl)
{

    leftimg = cv_bridge::toCvShare(msg_img, "bgr8")->image;

    width_img = leftimg.size().width;
    height_img = leftimg.size().height;


//    cv::imshow("img_orig", leftimg);
//    cv::waitKey(1);
    //ROS_ERROR("BOAS");

    //Interessa a ordem q se faz isto? aka primeiro transform e dps fromrosmsg ou ao contrario?
    sensor_msgs::PointCloud2::Ptr result (new sensor_msgs::PointCloud2);
    pcl_ros::transformPointCloud(frame_id_img, *msg_pcl, *result, *tf_listener);

    PointCloudXYZ::Ptr msg_cloudXYZ (new PointCloudXYZ);
    pcl::fromROSMsg(*result, *msg_cloudXYZ);


    calculate_depthmap(msg_cloudXYZ, depthMap_global);


    if(!leftimg.empty()){
      if(ROI_xmin_closest > 0 && ROI_xmax_closest > 0 &&  ROI_ymin_closest > 0 && ROI_ymax_closest > 0){
        // Show bounding box
        rectangle( leftimg, cv::Point(ROI_xmin_closest, ROI_ymin_closest), cv::Point(ROI_xmax_closest, ROI_ymax_closest), cv::Scalar(0,215,255) );

        for(int i=0; i< BB_cars.bounding_boxes.size(); i++){
          if(BB_cars.bounding_boxes[i].id != best_bb_id){
            rectangle( leftimg, cv::Point(BB_cars.bounding_boxes[i].xmin, BB_cars.bounding_boxes[i].ymin), cv::Point(BB_cars.bounding_boxes[i].xmax, BB_cars.bounding_boxes[i].ymax), cv::Scalar(110,198,120) );
          }

        }

        // Show center


        rectangle( leftimg, cv::Point(centroidX-THRESHOLD_CENTROID, centroidY-THRESHOLD_CENTROID), cv::Point(centroidX+THRESHOLD_CENTROID, centroidY+THRESHOLD_CENTROID), cv::Scalar(184,53,255) );
      }
      cv::imshow("img_orig", leftimg);
      cv::waitKey(1);
    }



   //Publish car PCL |A3|


   PointCloudXYZRGB::Ptr cloudRGB_toPublish (new PointCloudXYZRGB);

   cloudRGB_toPublish->header.frame_id = "vision_frame";
   pcl_conversions::toPCL(ros::Time::now(), cloudRGB_toPublish->header.stamp);
   cloudRGB_toPublish->height = 1;
   std::vector<cv::Point2f> centers;

   //pointPCLRGB.z = depthMap_input.at<float>(i,j);
   //cv::kmeans(InputArray data, 2, InputOutputArray bestLabels, TermCriteria criteria, int flags, OutputArray centers)
   //cv::kmeans(data, K ,labels,cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 10, 1.),MAX_ITERATIONS,cv::KMEANS_PP_CENTERS,colors);

   std::vector<int> kmeans_output;
   cv::kmeans(depthMap_global, 2, kmeans_output, cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 10, 1.), 5, cv::KMEANS_PP_CENTERS);

   //kmeans(points, clusterCount, labels,TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 10, 1.0), 3, KMEANS_PP_CENTERS, centers);

   for (int i = 0; i < depthMap_global.size().height; i++) {
       for (int j = 0; j < depthMap_global.size().width; j++) {

           if(j > ROI_xmin_closest && j < ROI_xmax_closest && i > ROI_ymin_closest && i < ROI_ymax_closest) {
               pcl::PointXYZRGB pointPCLRGB;


               if(depthMap_global.at<float>(i,j) > 0 && depthMap_global.at<float>(i,j) < 1000) {

                   pointPCLRGB.z = depthMap_global.at<float>(i,j);

               //ROS_ERROR("z= %f", pointPCLRGB.z);
               }

               else {
                   continue;
               }

               pointPCLRGB.x = (pointPCLRGB.z * j - pointPCLRGB.z * left_cam.cx) / left_cam.fx;
               pointPCLRGB.y = (pointPCLRGB.z * i - pointPCLRGB.z * left_cam.cy) / left_cam.fy;

                if(!leftimg.empty()){
                    pointPCLRGB.r = leftimg.at<cv::Vec3b>(i,j)[2];
                    pointPCLRGB.g = leftimg.at<cv::Vec3b>(i,j)[1];
                    pointPCLRGB.b = leftimg.at<cv::Vec3b>(i,j)[0];
                }


//                if(pointPCLRGB.z > 2000)
//                ROS_ERROR("i = %d j = %d", i, j);
               //ROS_ERROR("%f %f", left_cam.width, left_cam.height);
               //ROS_ERROR("%d %d %d %d", ROI_xmin_closest, ROI_xmax_closest, ROI_ymin_closest, ROI_ymax_closest);
               //ROS_ERROR("x = %f  y = %f  z = %f  r= %d g = %d b = %d", pointPCLRGB.x, pointPCLRGB.y, pointPCLRGB.z, pointPCLRGB.r, pointPCLRGB.g, pointPCLRGB.b);


               cloudRGB_toPublish->points.push_back(pointPCLRGB);
           }

       }

   }

   cloudRGB_toPublish->width = cloudRGB_toPublish->points.size();
   publisherCloudXYZRGB.publish(cloudRGB_toPublish);

   cloudRGB_global = cloudRGB_toPublish;

}



void cb_BoundingBoxes(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg_BB){

  if(msg_BB != NULL){
    int mean_BB = 0;
    darknet_ros_msgs::BoundingBox BoundingBox_cars;

    // Guardar todas as bounding boxes de carros
    BB_cars.bounding_boxes.clear();
    count_BB = 0;
    sum_sizesBB = 0;
    for (int i = 0; i < msg_BB->bounding_boxes.size(); i++){
      if(msg_BB->bounding_boxes[i].Class == "car")
      {
        int xmin = (int) msg_BB->bounding_boxes[i].xmin;
        int xmax = (int) msg_BB->bounding_boxes[i].xmax;
        int ymin = (int) msg_BB->bounding_boxes[i].ymin;
        int ymax = (int) msg_BB->bounding_boxes[i].ymax;
        width = xmax - xmin;
        height = ymax - ymin;

        if(xmax < width_img &&
           ymax < height_img &&
           xmin >= 0 &&
           ymin >= 0 &&
           xmax > 0 &&
           ymax > 0){
          size_bb = width * height;

          BoundingBox_cars.probability = msg_BB->bounding_boxes[i].probability;
          BoundingBox_cars.xmin = xmin;
          BoundingBox_cars.xmax = xmax;
          BoundingBox_cars.ymin = ymin;
          BoundingBox_cars.ymax = ymax;
          BoundingBox_cars.id = count_BB;


          BB_cars.bounding_boxes.push_back(BoundingBox_cars);


          sum_sizesBB = sum_sizesBB + size_bb;

          count_BB++;
        }

      }
    }

    // Calcular média dos tamanhos das bounding boxes
    if(count_BB > 0)
      mean_BB = sum_sizesBB / count_BB;
    else
      ROS_ERROR("No bounding boxes");

    //ROS_ERROR("Median of bb size: %d", mean_BB);
    min_dp = 100000000;
    // Calcular a menor distância a partir do depth map de cada bounding box (unicamente das que têm um tamanho maior do que a média)
    for(int j = 0; j < BB_cars.bounding_boxes.size(); j++){
      width = (int) BB_cars.bounding_boxes[j].xmax - BB_cars.bounding_boxes[j].xmin;
      height = (int) BB_cars.bounding_boxes[j].ymax - BB_cars.bounding_boxes[j].ymin;
      size_bb = width * height;

      ROI_xmin = BB_cars.bounding_boxes[j].xmin;
      ROI_xmax = BB_cars.bounding_boxes[j].xmax;
      ROI_ymin = BB_cars.bounding_boxes[j].ymin;
      ROI_ymax = BB_cars.bounding_boxes[j].ymax;
      float min_dp_frame = 100000000;


      if( size_bb > mean_BB){
        for(int y = ROI_ymin; y < ROI_ymax; y++){
          for(int x = ROI_xmin; x < ROI_xmax; x++){
            if(y < height_img-1 &&  x < width_img-1)
              dp = depthMap_global.at<float>(y, x);

            //ROS_ERROR("DIST: %d", dp);

            // Calcular distância mínima dentro de cada bounding box
            if(dp < min_dp_frame && dp > 0){
              min_dp_frame = dp;
            }
          }
        }
        // Ver qual a bounding box que tem o valor mais pequeno para a distância
        if(min_dp_frame < min_dp){
          min_dp = min_dp_frame;

          best_bb_id = BB_cars.bounding_boxes[j].id;
          best_bb_size = size_bb;
          ROI_xmin_closest = BB_cars.bounding_boxes[j].xmin;
          ROI_xmax_closest = BB_cars.bounding_boxes[j].xmax;
          ROI_ymin_closest = BB_cars.bounding_boxes[j].ymin;
          ROI_ymax_closest = BB_cars.bounding_boxes[j].ymax;
        }
      }
    }

    ROS_ERROR("Best BoundingBox: %d | d: %f | %d %d %d %d", best_bb_id, min_dp, ROI_xmin_closest, ROI_xmax_closest, ROI_ymin_closest, ROI_ymax_closest);



    biggest_width = ROI_xmax_closest - ROI_xmin_closest;
    biggest_height = ROI_ymax_closest - ROI_ymin_closest;

//    centerX = ROI_xmin_closest + biggest_width/2;
//    centerY = ROI_ymin_closest + biggest_height/2;

//    // Create a smaller bounding box around the center of the object
//    float min_dp1 = 100000000;
//    for(int x = centerX-THRESHOLD_CENTROID; x < centerX+THRESHOLD_CENTROID; x++){
//      for(int y = centerY-THRESHOLD_CENTROID; y < centerY+THRESHOLD_CENTROID; y++){
//        if(centerX-THRESHOLD_CENTROID >= 0 &&
//           centerY-THRESHOLD_CENTROID >= 0 &&
//           centerX+THRESHOLD_CENTROID < width_img &&
//           centerY+THRESHOLD_CENTROID < height_img
//           ){
//          dp = depthMap_global.at<float>(y, x);
//          //ROS_ERROR("DP = %f", dp);
//          if(dp < min_dp1 && dp > 0){
//           min_dp1 = dp;
//          }
//        }
//      }
//    }

//    ROS_ERROR("Closest Distance: %f", min_dp1);

    // Calculate centroid of ROI with cloudRGB_toPublish
    pcl::CentroidPoint<pcl::PointXYZ> centroid;

    for(int i=0; i < cloudRGB_global->points.size(); i++){
      centroid.add (pcl::PointXYZ(cloudRGB_global->points[i].x,
                                   cloudRGB_global->points[i].y,
                                   cloudRGB_global->points[i].z));
    }

    pcl::PointXYZ c1;
    centroid.get (c1);

    float pitch = atan2(c1.x, (c1.z + 1E-5));
    float roll = atan2(-c1.y, (c1.z + 1E-5));


    ROS_ERROR("Centroid: %f %f %f %f %f", c1.x, c1.y, c1.z, pitch, roll);

    tf2::Quaternion myQuaternion;

    // Valorização do A3: publish a ‘pose’ message from the centroid
    geometry_msgs::PoseStamped poseVF;
    poseVF.header.frame_id = "vision_frame";
    poseVF.pose.position.x = c1.x;
    poseVF.pose.position.y = c1.y;
    poseVF.pose.position.z = c1.z;
    myQuaternion.setRPY(roll, pitch, 0);
    myQuaternion = myQuaternion.normalize();
    poseVF.pose.orientation.x = myQuaternion[0];
    poseVF.pose.orientation.y = myQuaternion[1];
    poseVF.pose.orientation.z = myQuaternion[2];
    poseVF.pose.orientation.w = myQuaternion[3];

    pubPose.publish(poseVF);

    // Publish the bounding box from the nearest car -> This will be used in A4 (object_visualization)
    darknet_ros_msgs::BoundingBox nearest_car;
    nearest_car.id = best_bb_id;
    nearest_car.xmax = ROI_xmax_closest;
    nearest_car.xmin = ROI_xmin_closest;
    nearest_car.ymin = ROI_ymin_closest;
    nearest_car.ymax = ROI_ymax_closest;

    pubNearestCar.publish(nearest_car);



   }
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_3d_estimation");

    ros::NodeHandle n_public;
    ros::NodeHandle n_private("~");

    tf_listener = new tf::TransformListener();

    //std::string teste;
    //n_public.getParam("/object_3d_estimation/teste", teste);
    //ROS_ERROR("%s", teste.c_str());

    // First sincronization
    image_transport::ImageTransport it(n_public);
    //image_transport::Subscriber sub_imgleft = it.subscribe("/stereo/left/image_rect_color", 1, left_imgCB);
    message_filters::Subscriber<sensor_msgs::Image> sub_imgleft(n_public, "/stereo/left/image_rect_color", 1);
    //ros::Subscriber sub_pclrgb = n_public.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, pclCB);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_pclrgb(n_public, "/velodyne_points", 1);

    publisherCloudXYZ = n_public.advertise<PointCloudXYZ>("PCLTESTE",1);
    publisherCloudXYZRGB = n_public.advertise<PointCloudXYZRGB>("CARRO",1);

    // Second Callback
    ros::Subscriber sub_boundingBoxes = n_public.subscribe<darknet_ros_msgs::BoundingBoxes>("/objects/left/bounding_boxes", 1, cb_BoundingBoxes);


    n_public.getParam("/object_3d_estimation/left_img_frameId", frame_id_img);
    n_public.getParam("/object_3d_estimation/pointCloud_frameId", frame_id_pointCloud);
    pubPose = n_public.advertise<geometry_msgs::PoseStamped>("/nearest_Pose", 1);
    pubNearestCar = n_public.advertise<darknet_ros_msgs::BoundingBox>("/nearest_car", 1);


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



    //Source https://answers.ros.org/question/256238/solved-c-approximatetime-with-more-than-two-topics/
    //Source 7.3 http://wiki.ros.org/message_filters?distro=hydro#Time_Synchronizer
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> ApproximatePolicy;
    message_filters::Synchronizer<ApproximatePolicy> sync(ApproximatePolicy(10), sub_imgleft, sub_pclrgb);
    sync.registerCallback(boost::bind(&callback_img_pcl, _1, _2));

    /*
    // rate em Hertz
    ros::Rate rate(10);

    while(ros::ok())
    {
        ros::spinOnce(); // for listening to subscriptions


        rate.sleep();
    }*/

    ros::spin();

    return 0;
}
