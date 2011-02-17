#include <ros/ros.h>
#include <ros/node_handle.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv2/legacy/legacy.hpp>
#include <opencv/highgui.h>
#include <string.h>
#include <ccd/ccd.h>

//class ImageConverter {

class CCDNode : public CCD
{
public:
  double *params;
  std::vector<CvPoint2D64f> pts1;
  std::string image_topic_;
  CCDNode(ros::NodeHandle &n, char **argv) :
    n_(n), it_(n_)
    {
      n_.param("image_topic", image_topic_, std::string("/narrow_stereo/left/image_rect"));
      image_sub_ = it_.subscribe(image_topic_, 1, &CCDNode::imageCallback, this);
      params = new double[9];
      cv::namedWindow("Original", 1);
    }

  ~CCDNode()
    {
      delete(params);
    }


/** 
 * draw control points manually
 * 
 * @param event 
 * @param x 
 * @param y 
 * @param flags 
 * @param param 
 */
 static void on_mouse( int event, int x, int y, int flags, void* param )
    {
      //  MaskParams* params = (MaskParams*)_params;
      CCDNode *ccd = (CCDNode*)param;
      
      if( ccd->canvas.empty())
        return;
      
      //caution: check
      // if( img1.at<double>() )
      //   y = img1->height - y;
      
      switch( event )
      {
      case CV_EVENT_LBUTTONDOWN:
        // std::cout << "Event = " << event << std::endl;
        break;
      case CV_EVENT_LBUTTONUP:
        // std::cout << "Event = " << event << std::endl;
        cv::circle(ccd->canvas,cv::Point(x,y),2,cv::Scalar(0,0,255),2);
        ccd->pts1.push_back(cvPoint2D64f(x,y));
        // cvShowImage("Original",img1);
        cv::imshow("Original", ccd->canvas);
        break;
      }
}

  void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
    {

      //IplImage *cv_image = NULL;
      cv::Mat cv_image;
      try
      {
        cv_image = bridge_.imgMsgToCv(msg_ptr, "bgr8");
      }
      catch (sensor_msgs::CvBridgeException error)
      {
        ROS_ERROR("error");
      }
      //canvas = imread("../data/ball.png", 1);
      canvas.copyTo(cv_image);
      img.copyTo(cv_image);

      cv::GaussianBlur(cv_image, img , cv::Size(9,9) ,0);
      // cv::imshow("Origianl", img);
      char key ;

      params[0] = 0.5;
      params[1] = 4;
      params[2] = 4;
      params[3] = 3;
      params[4] = 0.5;
      params[5] = 0.25;
      params[6] = 40;
      params[7] = 1;
      params[8] = 80;
      set_params(params);

      cvSetMouseCallback( "Original", on_mouse,  (void*)this);
      // cvShowImage("Original",img1);
      cv::imshow("Original", canvas);
      while (1)
      {
        key = cvWaitKey(10);
        if (key == 27) break;
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
  CCDNode ccd_node(n, argv);
  ros::spin();
  return 0;
}
