#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <iostream>
#include <vector>
#include "sift_init.h"
#include "ccd/bspline.h"
#include "ccd/ccd.h"
using namespace cv;
// control points initialized mannually
std::vector<cv::Point2d> pts;

/** 
 * draw control points manually
 * 
 * @param event 
 * @param x 
 * @param y 
 * @param flags 
 * @param param 
 */
void on_mouse( int event, int x, int y, int flags, void* param )
{
  //  MaskParams* params = (MaskParams*)_params;
  cv::Mat *image = (cv::Mat*)param;
  if( image->empty())
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
      cv::circle(*image,cv::Point(x,y),2,cv::Scalar(0,0,255),2);
      pts.push_back(cv::Point2d(x,y));
      // cvShowImage("Original",image1);
      cv::imshow("Original", *image);
      break;
  }
}

int main (int argc, char * argv[]) 
{
  // if (argc < 2)
  // {
  //   printf("Usage %s image.png \n", argv[0]);
  //   exit(0);
  // }
  // 
  // the count of points on curve, equidistant distributed
  // const int resolution = 50;
  
  // the degree of B-Spline curve
  // int t = 3;
  
  // load image from a specified file
  //image1= cvLoadImage(argv[1], 1);
  // image1 = imread(argv[1], 1);
  // cv::Mat image = imread(argv[1], 1);  
  // cv::Mat input_image = imread("../data/ball.png", 1);

  // cv::Mat image;
  // cv::GaussianBlur(input_image, image , cv::Size(9,9) ,0);
  // cv::imshow("Origianl", image);
  char key;
  // while (1)
  // {
  //   key = cvWaitKey(10);
  //   if (key == 27) break;
  // }
  //image - working copy
  //image1 - for visualization
  CCD my_ccd;
  my_ccd.canvas = imread("../data/ball.png", 1);
  my_ccd.image = imread("../data/ball.png", 1);
  
  double *params = new double[10];
  params[0] = 0.5;
  params[1] = 4;
  params[2] = 4;
  params[3] = 3;
  params[4] = 0.5;
  params[5] = 0.25;
  params[6] = 40;
  params[7] = 1;
  params[8] = 80;
  params[9] = 4;

  my_ccd.set_params(params);
  
  // convert the image into Mat fortmat
  //cv::Mat image(image1);

  // store the control points trasformed in the shape space
  // CvPoint2D64f pts_tmp;

  ////////////////////////////////////////////////////////////////
  // mannaully initialize the control points
  ///////////////////////////////////////////////////////////////
  cv::namedWindow("Original", 1);
  cv::setMouseCallback( "Original", on_mouse,  (void*)&my_ccd.canvas);
  // cvShowImage("Original",image1);
  cv::imshow("Original", my_ccd.canvas);
  while (1)
  {
    key = cvWaitKey(10);
    if (key == 27) break;
  }
  ////////////////////////////////////////////////////////////////

  
  // for closed curves, we have to append 3 more points
  // to the end, these 3 new points are the three one
  // located in the head of the array
  if(pts.size() > params[9])
  {
    for (int i = 0; i < params[9]; ++i)
      pts.push_back(pts[i]);
  }

  my_ccd.init_pts(pts);
  my_ccd.run_ccd();

  double interval = (pts.size() - params[9])/params[8];
  std::cout << "resolution" << params[8] << " pts-n " << (pts.size() - params[9]) << " increment: " <<  interval  << " interval " << params[8]/(pts.size() - params[9]) << std::endl;
  delete(params);
  return 0;
}
