#include <ros/ros.h>
#include <ros/node_handle.h>
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv2/legacy/legacy.hpp>
#include <opencv/highgui.h>
#include <string.h>
#include <iostream>
#include "ccd/sift_init.h"
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
  std::vector<cv::Point3d> pts_;
  CCD ccd;
  int count_;
  int init_method_;
  CCDNode(ros::NodeHandle &n) :
  n_(n), it_(n_)
  {
    // n_.param("image_topic", image_topic_, std::string("/narrow_stereo/left/image_rect"));
    // n_.param("image_topic", image_topic_, std::string("/wide_stereo/left/image_rect_color"));
    n_.param("image_topic", image_topic_, std::string("/prosilica/image_raw"));
    n_.param("init_method", init_method_, 1);
    image_sub_ = it_.subscribe(image_topic_, 1, &CCDNode::imageCallback, this);
    ROS_INFO("CCDNode Ready, listening on topic %s", image_topic_.c_str());
    ccd.read_params(std::string("ccd_params.xml"));
    ROS_INFO("HELLOOOOOO %d", init_method_);
    ccd.init_mat();
    count_ = 0;
  }

  ~CCDNode(){}



  cv::Mat readImage(const sensor_msgs::ImageConstPtr& msg_ptr)
  {

        cv::Mat image = cv::Mat(1024, 768, CV_8UC3);
    if (msg_ptr->encoding.find("bayer") != std::string::npos)
      {
        const std::string& raw_encoding = msg_ptr->encoding;
        cv::Mat camera_image;
        int raw_type = CV_8UC1;
        if (raw_encoding == sensor_msgs::image_encodings::BGR8 || raw_encoding == sensor_msgs::image_encodings::RGB8)
          raw_type = CV_8UC3;
        const cv::Mat raw(msg_ptr->height, msg_ptr->width, raw_type,
                          const_cast<uint8_t*>(&msg_ptr->data[0]), msg_ptr->step);

        // Convert to color BGR
        int code = 0;
        if (msg_ptr->encoding == sensor_msgs::image_encodings::BAYER_RGGB8)
          code = CV_BayerBG2BGR;
        else if (msg_ptr->encoding == sensor_msgs::image_encodings::BAYER_BGGR8)
          code = CV_BayerRG2BGR;
        else if (msg_ptr->encoding == sensor_msgs::image_encodings::BAYER_GBRG8)
          code = CV_BayerGR2BGR;
        else if (msg_ptr->encoding == sensor_msgs::image_encodings::BAYER_GRBG8)
          code = CV_BayerGB2BGR;
        else
        {
          ROS_ERROR("[image_proc] Unsupported encoding '%s'", msg_ptr->encoding.c_str());
        }
        cv::cvtColor(raw, camera_image, code);

        // cv::Mat tmp2;
        // cv::cvtColor(tmpImage, tmp2, CV_BGR2GRAY);
        cv::resize(camera_image, image, cv::Size(1024,768),0,0,cv::INTER_LINEAR);
      }
      else
      {
        image = bridge_.imgMsgToCv(msg_ptr, "bgr8");
      }
      return image;
  }

  void contourSift()
{
  int row;
  IplImage sift_tpl = ccd.tpl;
  IplImage sift_tpl_img = ccd.canvas;
  IplImage *tpl_ptr = &sift_tpl;
  IplImage *tpl_img_ptr = &sift_tpl_img;
  // CvMat points_mat = sift_init(tpl_ptr, tpl_img_ptr, 30);
  CvMat points_mat = sift_init(tpl_ptr, tpl_img_ptr, 30);
  CvMat *points_mat_ptr = &points_mat;
  double *ptr = points_mat_ptr->data.db;
  int step = points_mat.step/sizeof(double);
  for (row = 0; row < points_mat_ptr->rows; ++row)
  {
    pts_.push_back(cv::Point3d((ptr+step*row)[0]/(ptr+step*row)[2], (ptr+step*row)[1]/(ptr+step*row)[2], 1));
    // cv::circle(my_ccd.canvas, cvPoint((ptr+step*row)[0]/(ptr+step*row)[2], (ptr+step*row)[1]/(ptr+step*row)[2]), 2, CV_RGB(0,255,0), 2, 8, 0);
  }
  //clean..........
}

  void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
  {
    count_++;
    cv::Mat cv_image = readImage(msg_ptr);
    cv_image.copyTo(ccd.canvas);
    cv_image.copyTo(ccd.image);
    if (count_ == 1)
    {

      ccd.tpl = cv::imread("data/book.png", 1);
      contourSift();
      
      if((int)pts_.size() > ccd.degree())
      {
        for (int i = 0; i < ccd.degree(); ++i)
          pts_.push_back(pts_[i]);
      }
      
      ccd.pts = pts_;
      BSpline bs(ccd.degree(), ccd.resolution(), ccd.pts);    
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
