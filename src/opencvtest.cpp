#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <ahb_ros_opencv_dynamic_reconfigure_example/filterConfig.h>

ros::Subscriber sub;
ros::Publisher pub;
ahb_ros_opencv_dynamic_reconfigure_example::filterConfig config;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	//ROS_INFO("imageCallback: image received with size %dx%d\n", msg->width, msg->height);

	cv_bridge::CvImagePtr cvimg = cv_bridge::toCvCopy(msg, "bgr8");

	cv::Mat img_gray;
	cv::cvtColor(cvimg->image, img_gray, CV_BGR2GRAY);
	cv::Canny(img_gray, img_gray, config.thresholdLow, config.thresholdHigh);

	cv::Mat img_edges_color(cvimg->image.size(), cvimg->image.type(), cv::Scalar(config.edgeColorBlue, config.edgeColorGreen, config.edgeColorRed));
	img_edges_color.copyTo(cvimg->image, img_gray); // use img_gray as mask

	pub.publish(cvimg->toImageMsg());
}

void dynamic_reconf_callback(ahb_ros_opencv_dynamic_reconfigure_example::filterConfig &p_config, uint32_t level) {
  //ROS_INFO("Reconfigure Request: %d %d", p_config.thresholdLow, p_config.thresholdHigh);

  config = p_config;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ahb_ros_opencv_dynamic_reconfigure_example");
  ros::NodeHandle n;

  sub = n.subscribe("image_raw", 1, imageCallback);
  pub = n.advertise<sensor_msgs::Image>("image_edges", 1);

  dynamic_reconfigure::Server<ahb_ros_opencv_dynamic_reconfigure_example::filterConfig> server;
  dynamic_reconfigure::Server<ahb_ros_opencv_dynamic_reconfigure_example::filterConfig>::CallbackType f;

  f = boost::bind(&dynamic_reconf_callback, _1, _2);
  server.setCallback(f);

  ros::spin();

  return 0;
}
