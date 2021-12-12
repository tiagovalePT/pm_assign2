#include "object_visualization.h"

//Callback image_left
void left_imgCB (const sensor_msgs::ImageConstPtr& msg_img)
{
    //obter imagem camera left
    leftimg = cv_bridge::toCvShare(msg_img, "bgr8")->image;

    cv::Mat Heatblend = cv::Mat::zeros(1,1,CV_8UC1);

    //Percorre todas as bouding boxes retornadas pelo YOLO
    for (int i = 0; i < BBs.bounding_boxes.size(); i++) {

        //Caso seja a bounding box do carro mais perto (com um erro de 5%) e a sinalização de perigo estiver de acordo com as
        //condicoes ( x < 10 e abs(y) < 5) --> sinalizar perigo
        if(BBs.bounding_boxes[i].xmin > 0.95 * nearestBB.xmin && BBs.bounding_boxes[i].xmin <= 1.05 * nearestBB.xmin &&
           BBs.bounding_boxes[i].ymin > 0.95 * nearestBB.ymin && BBs.bounding_boxes[i].ymin <= 1.05 * nearestBB.ymin &&
           BBs.bounding_boxes[i].xmax > 0.95 * nearestBB.xmax && BBs.bounding_boxes[i].xmax <= 1.05 * nearestBB.xmax &&
           BBs.bounding_boxes[i].ymax > 0.95 * nearestBB.ymax && BBs.bounding_boxes[i].ymax <= 1.05 * nearestBB.ymax &&
           poseXYZ.pose.position.x > 0 && poseXYZ.pose.position.x < 10 && abs(poseXYZ.pose.position.y) < 5) {

            //Conteudo a escrever na imagem -> pose do centroide do carro em questão
            std::string coord = "(X,Y;Z) = (" + std::to_string(poseXYZ.pose.position.x) + "; " +
                                                std::to_string(poseXYZ.pose.position.y) + "; " +
                                                std::to_string(poseXYZ.pose.position.z) + ")";

            //Conteudo a escrever na imagem -> dimensões calculadas do carro
            std::string sizeCar = "Width = " + std::to_string(nearest_car_width) + "; " +
                                  "Height = " + std::to_string(nearest_car_height);

            //Adicionar heatmap na Region of Interest
            addWeighted(leftimg, 0.2, depthMapHeat, 0.8, 0.0, Heatblend);
            cv::Mat heatblend2 = cv::Mat(Heatblend, cv::Rect(cv::Point(nearestBB.xmin, nearestBB.ymin), cv::Point(nearestBB.xmax, nearestBB.ymax)));
            heatblend2.copyTo(leftimg(cv::Rect(BBs.bounding_boxes[i].xmin, BBs.bounding_boxes[i].ymin, heatblend2.cols, heatblend2.rows)));

            //Escrever na imagem os conteudos preparados anteriormente
            putText(leftimg, coord, cv::Point(BBs.bounding_boxes[i].xmin, BBs.bounding_boxes[i].ymin - 20), cv::FONT_HERSHEY_COMPLEX, 0.7, cvScalar(255, 255, 255), 1.8, cv::LINE_AA);
            putText(leftimg, sizeCar, cv::Point(BBs.bounding_boxes[i].xmin, BBs.bounding_boxes[i].ymax + 20), cv::FONT_HERSHEY_COMPLEX, 0.7, cvScalar(255, 255, 255), 1.8, cv::LINE_AA);

            putText(leftimg, "!!! HAZARD !!! !!! HAZARD !!! !!! HAZARD !!!", cv::Point(BBs.bounding_boxes[i].xmin, BBs.bounding_boxes[i].ymin - 45), cv::FONT_HERSHEY_COMPLEX, 0.7, cvScalar(0, 0, 255), 1.8, cv::LINE_AA);
                        putText(leftimg, "!!! HAZARD !!! !!! HAZARD !!! !!! HAZARD !!!", cv::Point(BBs.bounding_boxes[i].xmin, BBs.bounding_boxes[i].ymax + 45), cv::FONT_HERSHEY_COMPLEX, 0.7, cvScalar(0, 0, 255), 1.8, cv::LINE_AA);
                        rectangle(leftimg, cv::Point(BBs.bounding_boxes[i].xmin, BBs.bounding_boxes[i].ymin), cv::Point(BBs.bounding_boxes[i].xmax, BBs.bounding_boxes[i].ymax), cv::Scalar(0,0,255), 2);

            //Sinalizar bounding box a vermelho
            rectangle(leftimg, cv::Point(BBs.bounding_boxes[i].xmin, BBs.bounding_boxes[i].ymin), cv::Point(BBs.bounding_boxes[i].xmax, BBs.bounding_boxes[i].ymax), cv::Scalar(0,0,255),2);


        }
        else
            //Caso não seja necessário sinalizar perigo --> mostrar bounding box a verde
            rectangle(leftimg, cv::Point(BBs.bounding_boxes[i].xmin, BBs.bounding_boxes[i].ymin), cv::Point(BBs.bounding_boxes[i].xmax, BBs.bounding_boxes[i].ymax), cv::Scalar(110,198,120), 2);

    }

    //Mostrar resultado do algoritmo de deteção de perigo
    cv::imshow("img_orig", leftimg);
    cv::waitKey(1);

}

//Call back que analisa as bounding boxes recebidas para depois serem analisadas e representadas
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

//Callback que guarda as dimensões da bounding box do carro mais perto (foi publicada no nó object_3d_estimation)
void cb_nearestBB (const darknet_ros_msgs::BoundingBox::ConstPtr& msg_BB)
{
    nearestBB.xmax = msg_BB->xmax;
    nearestBB.xmin = msg_BB->xmin;
    nearestBB.ymax = msg_BB->ymax;
    nearestBB.ymin = msg_BB->ymin;
    nearestBB.id = msg_BB->id;
    //ROS_ERROR("Nearest BoundingBox: %d %d %d %d", nearestBB.xmax, nearestBB.xmin, nearestBB.ymax, nearestBB.ymin);
}

//Callback para transformar as coordenadas da pose do centroide do carro mais perto
//Transformada de vision_frame para base_link
void cb_poseXYZ(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    tf_listener->transformPose("/base_link", *msg, poseXYZ);
    //ROS_ERROR("x = %f  y = %f  z= %f", poseXYZ.pose.position.x, poseXYZ.pose.position.y, poseXYZ.pose.position.z);
}


//Callback para receber o valor das dimensões do carro calculadas
//Dimensões foram publicadas pelo nó object_3d_estimation)
void cb_carSize(const geometry_msgs::Point::ConstPtr& msg_car_size){

  nearest_car_width = msg_car_size->x;
  nearest_car_height = msg_car_size->y;
}

//Callback que recebe o depthmap desenvolvido no nó object_3d_estimation
//No nó object_3d_estimation é publicado o depthmap
void cb_pubDepthmap(const sensor_msgs::ImageConstPtr& msg) {

    cv::Mat depthMap = cv_bridge::toCvShare(msg, "mono8")->image;
    cv::applyColorMap(depthMap, depthMapHeat, cv::COLORMAP_HOT);
}


int main(int argc, char **argv)
{
   ros::init(argc, argv, "object_visualization");

   ros::NodeHandle n_public;
   ros::NodeHandle n_private("~");


   image_transport::ImageTransport it(n_public);

   //Subscribe image_left
   image_transport::Subscriber sub_imgleft = it.subscribe("/stereo/left/image_rect_color", 1, left_imgCB);
   //Subscribe bounding boxes detetadas
   ros::Subscriber sub_boundingBoxes = n_public.subscribe<darknet_ros_msgs::BoundingBoxes>("/objects/left/bounding_boxes", 1, cb_BoundingBoxes);
   //Subscribe das dimensoes da bounding box do carro mais perto
   ros::Subscriber sub_nearestBB = n_public.subscribe<darknet_ros_msgs::BoundingBox>("/nearest_car", 1, cb_nearestBB);
   //Subscribe da pose do centroide do carro mais perto
   ros::Subscriber sub_poseXYZ = n_public.subscribe<geometry_msgs::PoseStamped>("/nearest_Pose", 1, cb_poseXYZ);
   //Subscribe das dimensoes do carro calculadas
   ros::Subscriber sub_carSize = n_public.subscribe<geometry_msgs::Point>("/nearest_car_size", 1, cb_carSize);
   //Subscribe do depthmap implementado no nó object_3d_detection
   image_transport::Subscriber sub_depthMap = it.subscribe("/depthMap_pub", 1, cb_pubDepthmap);

   //Receber parametros que contem o nome das frames ( neste caso -> vision_frame e velodyne
   n_public.getParam("/object_3d_estimation/left_img_frameId", frame_id_img);
   n_public.getParam("/object_3d_estimation/pointCloud_frameId", frame_id_pointCloud);

   //Algoritmo para obter informação da camera (fx,fy,cx,cy,height e width)
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


   ros::spin();

   return 0;
}
