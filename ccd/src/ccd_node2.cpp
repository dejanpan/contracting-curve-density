#include <ros/ros.h>
#include <ros/node_handle.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv2/legacy/legacy.hpp>
#include <opencv/highgui.h>
#include <string.h>
#include <iostream>
#include <ccd/bspline.h>
#include <ccd/ccd.h>

//class ImageConverter {

class CCDNode
{
 public:
  ros::NodeHandle n_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  sensor_msgs::CvBridge bridge_;
  std::string image_topic_;
  CCD ccd;
  int count_;
  int init_method_;
  CCDNode(ros::NodeHandle &n) :
  n_(n), it_(n_)
  {
    // n_.param("image_topic", image_topic_, std::string("/narrow_stereo/left/image_rect"));
    n_.param("image_topic", image_topic_, std::string("/wide_stereo/left/image_rect"));
    n_.param("init_method", init_method_, 1);
    image_sub_ = it_.subscribe(image_topic_, 1, &CCDNode::imageCallback, this);
    ROS_INFO("CCDNode Ready, listening on topic %s", image_topic_.c_str());
    ccd.read_params(std::string("ccd_params.xml"));
    ccd.init_mat();
    count_ = 0;
  }

  ~CCDNode(){}



  void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
  {
    count_++;
    //IplImage *cv_image = NULL;
    cv::Mat cv_image;
    try
    {
      cv_image = bridge_.imgMsgToCv(msg_ptr);
    }
    catch (sensor_msgs::CvBridgeException error)
    {
      ROS_ERROR("error");
    }
    //ccd.pts.clear();
    //canvas = imread("../data/ball.png", 1);
    cv_image.copyTo(ccd.canvas);
    cv_image.copyTo(ccd.image);
    // Mat_<Vec3b>& img = (Mat_<Vec3b>&)ccd.image;
    // std::cerr << "sample data: " << (int)img(356, 180)[0] << " " << (int)img(356, 180)[1] << " " << (int)img(356, 180)[2] << " "  << std::endl;


    std::cerr << "I got a new image" << std::endl;

    if (count_ == 1)
    {
      ccd.tpl = cv::imread("../data/book.png", 1);
      ccd.init_pts(init_method_);
        
      BSpline bs(ccd.degree(), ccd.resolution(), ccd.pts);    

      // for (int i = 0; i < ccd.get_resolution(); ++i)
      //   std::cerr << "pts bspline: " << bs[i].x << " " << bs[i].y << std::endl;
        
      ccd.init_cov(bs, (int)ccd.degree());
    }      
    ccd.run_ccd();
    // cv::imshow("Original", ccd.canvas);
    // cv::waitKey(1);
    std::stringstream name;
    name << count_;
    cv::imwrite(name.str() + ".png", ccd.canvas);
    // sleep(1);
  }
  //protected:
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
  CCDNode ccd_node(n);
  //  ccd_node.canvas = imread(argv[1], 1);
  //  ccd_node.img = imread(argv[1], 1);
  ros::spin();
  return 0;
}
