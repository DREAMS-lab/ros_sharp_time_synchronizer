#include "ros/ros.h"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "image_transport/image_transport.h"
#include <sensor_msgs/CameraInfo.h>
#include <boost/assign/list_of.hpp>

image_transport::Publisher image_pub;
ros::Publisher info_pub;
sensor_msgs::CameraInfo cam_info_msg = sensor_msgs::CameraInfo();

void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
{
  try
  {
    cv::Mat image = cv::imdecode(cv::Mat(msg->data),1);
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
    ros::Time time = ros::Time::now();
    cv_ptr->encoding = "bgr8";
    cv_ptr->header.stamp = time;
    cv_ptr->header.frame_id = "/traj_output";
    cv_ptr->image = image;
    image_pub.publish(cv_ptr->toImageMsg());      
    cam_info_msg.header.stamp = time;
    info_pub.publish(cam_info_msg);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert to image!");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_pub = it.advertise("/udrone/camera/up/image_raw/raw", 1);
  info_pub = nh.advertise<sensor_msgs::CameraInfo>("/udrone/camera/up/image_raw/camera_info", 1);
  ros::Rate loop_rate(50);  
  cam_info_msg.header.frame_id = "udrone_up";
  cam_info_msg.width = 640;
  cam_info_msg.height = 480;
  cam_info_msg.K = boost::array<double, 9>{{585.7560709479847, 0.0, 320.5, 0.0, 585.7560709479847, 240.5, 0.0, 0.0, 1.0}};
  cam_info_msg.distortion_model = "plumb_bob";
  cam_info_msg.D = boost::assign::list_of(0.0)(0.0)(0.0)(0.0)(0.0).convert_to_container<std::vector<double> >(); 
  cam_info_msg.R = boost::array<double, 9>{{ 1, 0, 0, 0, 1, 0, 0, 0, 1 }};
  cam_info_msg.P = boost::array<double, 12>{{585.7560709479847, 0.0, 320.5, -0.0, 0.0, 585.7560709479847, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0}};


  ros::Subscriber sub = nh.subscribe("/udrone/camera/up/rgb/compressed", 1, imageCallback);

  while(ros::ok()){
    ros::spin();
    loop_rate.sleep();
  }
}
