#include <ros/ros.h>
#include <ros/node_handle.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv2/legacy/legacy.hpp>
#include "bspline.h"
#include <opencv/highgui.h>
#include <string.h>

class ImageConverter {

public:
  ImageConverter(ros::NodeHandle &n, char **argv) :
    n_(n), it_(n_)
    {
      image_sub_ = it_.subscribe(argv[1], 1, &ImageConverter::imageCallback, this);

    }

  ~ImageConverter()
    {
      cvDestroyWindow("Image window");
    }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
    {

      IplImage *cv_image = NULL;
      try
      {
        cv_image = bridge_.imgMsgToCv(msg_ptr, "bgr8");
      }
      catch (sensor_msgs::CvBridgeException error)
      {
        ROS_ERROR("error");
      }
    }
protected:
  ros::NodeHandle n_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  sensor_msgs::CvBridge bridge_;
};

int main(int argc, char** argv)
{
  if(argc < 1)
  {
    exit(-1);
    std::cout << "usage: ros_to_openCv topic" << std::endl;
  }

  ros::init(argc, argv, "ros_to_openCv");
  ros::NodeHandle n("~");
  ImageConverter ic(n, argv);
  ros::spin();
  return 0;
}
