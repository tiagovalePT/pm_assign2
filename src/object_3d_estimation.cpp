#include "object_3d_estimation.h"


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

void cb_BoundingBoxes(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg_BB){

  if(msg_BB != NULL){
    int i = 0, z = 0;
    size_bb = 0;
    while(/*msg_BB->bounding_boxes[i].id*/ z < 3){
      if(msg_BB->bounding_boxes[i].Class == "car")
      {
        width = (int) msg_BB->bounding_boxes[i].xmax - msg_BB->bounding_boxes[i].xmin;
        height = (int) msg_BB->bounding_boxes[i].ymax - msg_BB->bounding_boxes[i].ymin;
        size_bb = width * height;

        if( size_bb > biggest_bb){
          biggest_bb = size_bb;
          biggest_width = width;
          biggest_height = height;
          id_bb = msg_BB->bounding_boxes[i].id;
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
      i++;
      z++;
    }
    ROS_ERROR("BIGGEST BB IS: %d || SIZE IS: %d %d %d", id_bb, biggest_bb, ROI_xmax-ROI_xmin, ROI_ymax-ROI_ymin);


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
