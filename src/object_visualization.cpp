#include "object_visualization.h"



void left_imgCB (const sensor_msgs::ImageConstPtr& msg_img)
{
    leftimg = cv_bridge::toCvShare(msg_img, "bgr8")->image;

    for (int i = 0; i < BBs.bounding_boxes.size(); i++) {

        if(BBs.bounding_boxes[i].xmin > 0.95 * nearestBB.xmin && BBs.bounding_boxes[i].xmin <= 1.05 * nearestBB.xmin &&
           BBs.bounding_boxes[i].ymin > 0.95 * nearestBB.ymin && BBs.bounding_boxes[i].ymin <= 1.05 * nearestBB.ymin &&
           BBs.bounding_boxes[i].xmax > 0.95 * nearestBB.xmax && BBs.bounding_boxes[i].xmax <= 1.05 * nearestBB.xmax &&
           BBs.bounding_boxes[i].ymax > 0.95 * nearestBB.ymax && BBs.bounding_boxes[i].ymax <= 1.05 * nearestBB.ymax &&
           poseXYZ.pose.position.x > 0 && poseXYZ.pose.position.x < 10 && abs(poseXYZ.pose.position.y) < 5) {


            std::string coord = "(X,Y;Z) = (" + std::to_string(poseXYZ.pose.position.x) + "; " +
                                                std::to_string(poseXYZ.pose.position.y) + "; " +
                                                std::to_string(poseXYZ.pose.position.z) + ")";

            std::string sizeCar = "Width = " + std::to_string(nearest_car_width) + "; " +
                                  "Height = " + std::to_string(nearest_car_height);

            putText(leftimg, coord, cv::Point(BBs.bounding_boxes[i].xmin, BBs.bounding_boxes[i].ymin - 20), cv::FONT_HERSHEY_COMPLEX, 0.7, cvScalar(255, 255, 255), 1.8, cv::LINE_AA);
            putText(leftimg, sizeCar, cv::Point(BBs.bounding_boxes[i].xmin, BBs.bounding_boxes[i].ymax + 20), cv::FONT_HERSHEY_COMPLEX, 0.7, cvScalar(255, 255, 255), 1.8, cv::LINE_AA);

            rectangle(leftimg, cv::Point(BBs.bounding_boxes[i].xmin, BBs.bounding_boxes[i].ymin), cv::Point(BBs.bounding_boxes[i].xmax, BBs.bounding_boxes[i].ymax), cv::Scalar(0,0,255));


        }
        else
            rectangle(leftimg, cv::Point(BBs.bounding_boxes[i].xmin, BBs.bounding_boxes[i].ymin), cv::Point(BBs.bounding_boxes[i].xmax, BBs.bounding_boxes[i].ymax), cv::Scalar(110,198,120));

    }

    cv::imshow("img_orig", leftimg);
    cv::waitKey(1);
}

void cb_BoundingBoxes (const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg_BB) {

    BBs.bounding_boxes.clear();


    for(int i=0; i< msg_BB->bounding_boxes.size(); i++){

        if(msg_BB->bounding_boxes[i].Class == "car")
        {
            darknet_ros_msgs::BoundingBox bb_self;

            bb_self.xmin = msg_BB->bounding_boxes[i].xmin;
            bb_self.xmax = msg_BB->bounding_boxes[i].xmax;
            bb_self.ymin = msg_BB->bounding_boxes[i].ymin;
            bb_self.ymax = msg_BB->bounding_boxes[i].ymax;

            BBs.bounding_boxes.push_back(bb_self);
        }

    }
}

void cb_nearestBB (const darknet_ros_msgs::BoundingBox::ConstPtr& msg_BB)
{
    nearestBB.xmax = msg_BB->xmax;
    nearestBB.xmin = msg_BB->xmin;
    nearestBB.ymax = msg_BB->ymax;
    nearestBB.ymin = msg_BB->ymin;
    nearestBB.id = msg_BB->id;

    //ROS_ERROR("Nearest BoundingBox: %d %d %d %d", nearestBB.xmax, nearestBB.xmin, nearestBB.ymax, nearestBB.ymin);


}


void cb_poseXYZ(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

    tf_listener->transformPose("/base_link", *msg, poseXYZ);

    //ROS_ERROR("x = %f  y = %f  z= %f", poseXYZ.pose.position.x, poseXYZ.pose.position.y, poseXYZ.pose.position.z);

}


void cb_carSize(const geometry_msgs::Point::ConstPtr& msg_car_size){

  nearest_car_width = msg_car_size->x;
  nearest_car_height = msg_car_size->y;

}


void cb_pubDepthmap(const sensor_msgs::ImageConstPtr& msg) {

    cv::Mat depthMap = cv_bridge::toCvShare(msg, "bgr8")->image;

    cv::Mat depthROI = cv::Mat(depthMap, cv::Rect(cv::Point(nearestBB.xmin, nearestBB.ymin), cv::Point(nearestBB.xmax, nearestBB.ymax)));


    cv::Mat output;
    cv::applyColorMap(depthROI,output, cv::COLORMAP_HOT);


        cv::imshow("depth", output);
        cv::waitKey(1);

//    cv::imshow("depth", depthMap2);
//    cv::waitKey(1);

}


int main(int argc, char **argv)
{
   ros::init(argc, argv, "object_visualization");

   ros::NodeHandle n_public;
   ros::NodeHandle n_private("~");

   //tf_listener = new tf::TransformListener();


   // First sincronization
   image_transport::ImageTransport it(n_public);
   image_transport::Subscriber sub_imgleft = it.subscribe("/stereo/left/image_rect_color", 1, left_imgCB);
   //message_filters::Subscriber<sensor_msgs::Image> sub_imgleft(n_public, "/stereo/left/image_rect_color", 1);
   //ros::Subscriber sub_pclrgb = n_public.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, pclCB);
   //message_filters::Subscriber<sensor_msgs::PointCloud2> sub_pclrgb(n_public, "/velodyne_points", 1);



   ros::Subscriber sub_boundingBoxes = n_public.subscribe<darknet_ros_msgs::BoundingBoxes>("/objects/left/bounding_boxes", 1, cb_BoundingBoxes);
   //message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> sub_boundingBoxes(n_public, "/objects/left/bounding_boxes", 1);

   ros::Subscriber sub_nearestBB = n_public.subscribe<darknet_ros_msgs::BoundingBox>("/nearest_car", 1, cb_nearestBB);
   ros::Subscriber sub_poseXYZ = n_public.subscribe<geometry_msgs::PoseStamped>("/nearest_Pose", 1, cb_poseXYZ);
   ros::Subscriber sub_carSize = n_public.subscribe<geometry_msgs::Point>("/nearest_car_size", 1, cb_carSize);
   image_transport::Subscriber sub_depthMap = it.subscribe("/depthMap_pub", 1, cb_pubDepthmap);


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

   tf_listener = new tf::TransformListener();

   try {
       tf_listener->waitForTransform(frame_id_img, "/base_link", ros::Time::now(), ros::Duration(3.0));
     } catch (tf::TransformException &ex) {
       ROS_ERROR("%s", ex.what());
       ros::Duration(1.0).sleep();
     }


/*
   //Source https://answers.ros.org/question/256238/solved-c-approximatetime-with-more-than-two-topics/
   //Source 7.3 http://wiki.ros.org/message_filters?distro=hydro#Time_Synchronizer
   typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes> ApproximatePolicy;
   message_filters::Synchronizer<ApproximatePolicy> sync(ApproximatePolicy(10), sub_imgleft, sub_boundingBoxes);
   sync.registerCallback(boost::bind(&mainCBack, _1, _2));*/

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
