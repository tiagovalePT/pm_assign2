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

    cv::imshow("img_orig", leftimg);
    cv::waitKey(1);


    //Interessa a ordem q se faz isto? aka primeiro transform e dps fromrosmsg ou ao contrario?
    sensor_msgs::PointCloud2::Ptr result (new sensor_msgs::PointCloud2);
    pcl_ros::transformPointCloud(frame_id_img, *msg_pcl, *result, *tf_listener);

    PointCloudXYZ::Ptr msg_cloudXYZ (new PointCloudXYZ);
    pcl::fromROSMsg(*result, *msg_cloudXYZ);


    calculate_depthmap(msg_cloudXYZ, depthMap_global);

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
        width = (int) msg_BB->bounding_boxes[i].xmax - msg_BB->bounding_boxes[i].xmin;
        height = (int) msg_BB->bounding_boxes[i].ymax - msg_BB->bounding_boxes[i].ymin;
        size_bb = width * height;

        BoundingBox_cars.probability = msg_BB->bounding_boxes[i].probability;
        BoundingBox_cars.xmin = msg_BB->bounding_boxes[i].xmin;
        BoundingBox_cars.xmax = msg_BB->bounding_boxes[i].xmax;
        BoundingBox_cars.ymin = msg_BB->bounding_boxes[i].ymin;
        BoundingBox_cars.ymax = msg_BB->bounding_boxes[i].ymax;
        BoundingBox_cars.id = count_BB;


        BB_cars.bounding_boxes.push_back(BoundingBox_cars);


        sum_sizesBB = sum_sizesBB + size_bb;

        count_BB++;
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
    rectangle( depthMap_global, cv::Point(ROI_xmin_closest, ROI_ymin_closest), cv::Point(ROI_xmax_closest, ROI_ymax_closest), cv::Scalar(255) );



    biggest_width = ROI_xmax_closest - ROI_xmin_closest;
    biggest_height = ROI_ymax_closest - ROI_ymin_closest;

    centerX = ROI_xmin_closest + biggest_width/2;
    centerY = ROI_ymin_closest + biggest_height/2;

    // Show center in depthMap
    rectangle( depthMap_global, cv::Point(centerX-THRESHOLD_CENTROID, centerY-THRESHOLD_CENTROID), cv::Point(centerX+THRESHOLD_CENTROID, centerY+THRESHOLD_CENTROID), cv::Scalar(255) );

    cv::imshow("Image with Bounding Box", depthMap_global);
    cv::waitKey(1);

    float min_dp1 = 100000000;


    for(int x = centerX-THRESHOLD_CENTROID; x < centerX+THRESHOLD_CENTROID; x++){
      for(int y = centerY-THRESHOLD_CENTROID; y < centerY+THRESHOLD_CENTROID; y++){
        dp = depthMap_global.at<float>(y, x);
        //ROS_ERROR("DP = %f", dp);
        if(dp < min_dp1 && dp > 0){
         min_dp1 = dp;
        }
      }
    }

    //float segment_center = depthMap_global.at<float>(centerY, centerX);
    ROS_ERROR("Depth of center: %f", min_dp1);

    //cropped_depthMap = depthMap_global(cv::Rect(ROI_xmin_closest, ROI_ymin_closest, biggest_width, biggest_height));
    //pcl::computeCentroid(pointCloud, outCentroid);

    // Calculate centroid of ROI in the point cloud
    int u, v, posFinal_px, posFinal_py, pxValue;

    double x, y, z;


    for (int i = 0; i < cloud_for_centroid->points.size(); i++)
    {
        if(cloud_for_centroid->points[i].x > ROI_xmin_closest && ROI_xmin_closest < ROI_xmax_closest &&
           cloud_for_centroid->points[i].y > ROI_ymin_closest && ROI_ymin_closest < ROI_ymax_closest)
        {
          pcl::PointXYZ pointPCL;
          pointPCL.x = cloud_for_centroid->points[i].x;
          pointPCL.y = cloud_for_centroid->points[i].y;
          pointPCL.z = cloud_for_centroid->points[i].z;
          cloud_of_centroid->points.push_back(pointPCL);
        }
    }

    pcl::PointXYZ centroid;
    //pcl::ComputeCentroid(cloud_of_centroid, centroid);




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

    // Second Callback
    ros::Subscriber sub_boundingBoxes = n_public.subscribe<darknet_ros_msgs::BoundingBoxes>("/objects/left/bounding_boxes", 1, cb_BoundingBoxes);


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
