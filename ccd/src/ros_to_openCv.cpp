#include <ros/ros.h>
#include <ros/node_handle.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <string.h>
class ImageConverter {

public:
  ImageConverter(ros::NodeHandle &n) :
    n_(n), it_(n_)
  {
    cvNamedWindow("Image window");
    image_sub_ = it_.subscribe(
                               "/stereo/left/image_rect", 1, &ImageConverter::imageCallback, this);
    n_.param ("save_image", save_image_, std::string(""));
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

    if (cv_image->width > 60 && cv_image->height > 60)
      cvCircle(cv_image, cvPoint(50,50), 10, cvScalar(255));

    cvShowImage("Image window", cv_image);
    n_.getParam("save_image", save_image_);
    if(save_image_ != "")
      {
        ROS_INFO("Saving image to %s", save_image_.c_str());
        cvSaveImage(save_image_.c_str(), cv_image);
      }
    cvWaitKey(3);
  }

protected:

  ros::NodeHandle n_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  sensor_msgs::CvBridge bridge_;
  std::string save_image_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_to_openCv");
  ros::NodeHandle n("~");
  ImageConverter ic(n);
  ros::spin();
  return 0;
}
