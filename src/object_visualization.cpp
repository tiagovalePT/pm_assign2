#include <object_visualization.h>

void cbLeftImg(){

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_visualization");
  ros::NodeHandle nh;
  ros::NodeHandle n_public;


  image_transport::ImageTransport it(n_public);

  image_transport::Subscriber sub = it.subscribe( "/stereo/left/image_rect_color", 1, cbLeftImg);



  ROS_INFO("Hello world!");
  ros::spin();



}
