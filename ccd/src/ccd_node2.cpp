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

class CCDNode
{
public:
  ros::NodeHandle n_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  sensor_msgs::CvBridge bridge_;
  double *params;
  std::vector<CvPoint2D64f> pts1;
  std::string image_topic_;
  CCD ccd;
  int count_;
  CCDNode(ros::NodeHandle &n) :
    n_(n), it_(n_)
    {
      // n_.param("image_topic", image_topic_, std::string("/narrow_stereo/left/image_rect"));
      n_.param("image_topic", image_topic_, std::string("/wide_stereo/left/image_rect"));
      image_sub_ = it_.subscribe(image_topic_, 1, &CCDNode::imageCallback, this);
      params = new double[9];
      cv::namedWindow("Original", 1);
      ROS_INFO("CCDNode Ready, listening on topic %s", image_topic_.c_str());
      params[0] = 0.5;
      params[1] = 4;
      params[2] = 4;
      params[3] = 3;
      params[4] = 0.5;
      params[5] = 0.25;
      params[6] = 40;
      params[7] = 1;
      params[8] = 50;
      params[9] = 4;
      ccd.set_params(params);
      count_ = 0;
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
      CCD *ccd = (CCD*)param;
      
      if( ccd->canvas.empty())
        return;
      
      //caution: check
      // if( image1.at<double>() )
      //   y = image1->height - y;
      
      switch( event )
      {
      case CV_EVENT_LBUTTONDOWN:
        // std::cout << "Event = " << event << std::endl;
        break;
      case CV_EVENT_LBUTTONUP:
        // std::cout << "Event = " << event << std::endl;
        cv::circle(ccd->canvas,cv::Point(x,y),2,cv::Scalar(0,0,255),2);
        ccd->pts.push_back(cvPoint2D64f(x,y));
        // cvShowImage("Original",image1);
        cv::imshow("Original", ccd->canvas);
        break;
      }
}

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
      Mat_<Vec3b>& img = (Mat_<Vec3b>&)ccd.image;
      std::cerr << "sample data: " << (int)img(356, 180)[0] << " " << (int)img(356, 180)[1] << " " << (int)img(356, 180)[2] << " "  << std::endl;


      std::cerr << "I got a new image" << std::endl;

      //cv::GaussianBlur(cv_image, img, cv::Size(9,9), 0);
      // cv::imshow("Origianl", img);
      if (count_ == 1)
      {
        char key;
      
        cvSetMouseCallback( "Original", on_mouse,  (void*)&this->ccd);
        // cvShowImage("Original",img1);
        cv::imshow("Original", ccd.canvas);
        while (1)
        {
          key = cvWaitKey(10);
          if (key == 27) 
            break;
        }
        // for closed curves, we have to append 3 more points
        // to the end, these 3 new points are the three one
        // located in the head of the array
        if(ccd.pts.size() > params[9])
          for (int i = 0; i < params[9]; ++i)
            ccd.pts.push_back(ccd.pts[i]);
        
        // for (int i = 0; i < ccd.pts.size(); i++)
        //   std::cerr << "pts initialized: " << ccd.pts[i].x << " " << ccd.pts[i].y << std::endl;
        
        BSpline bs(params[9], ccd.get_resolution(), ccd.pts);

    

        // for (int i = 0; i < ccd.get_resolution(); ++i)
        //   std::cerr << "pts bspline: " << bs[i].x << " " << bs[i].y << std::endl;

        ccd.init_cov(bs, params[9]);
        // bs.release();
        // bs.~BSpline();
      }
      //ccd.init_pts(pts1);
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
