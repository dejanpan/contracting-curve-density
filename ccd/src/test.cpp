#include "ccd/ccd.h"

using namespace cv;
// control points initialized mannually
std::vector<CvPoint2D64f> pts;

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
  cv::Mat *img = (cv::Mat*)param;
  if( img->empty())
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
      cv::circle(*img,cv::Point(x,y),2,cv::Scalar(0,0,255),2);
      pts.push_back(cvPoint2D64f(x,y));
      // cvShowImage("Original",img1);
      cv::imshow("Original", *img);
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
  //img1= cvLoadImage(argv[1], 1);
  // img1 = imread(argv[1], 1);
  // cv::Mat img = imread(argv[1], 1);  
  // cv::Mat input_img = imread("../data/ball.png", 1);

  // cv::Mat img;
  // cv::GaussianBlur(input_img, img , cv::Size(9,9) ,0);
  // cv::imshow("Origianl", img);
  char key;
  // while (1)
  // {
  //   key = cvWaitKey(10);
  //   if (key == 27) break;
  // }
  //img - working copy
  //img1 - for visualization
  CCD my_ccd;
  my_ccd.canvas = imread("../data/ball.png", 1);
  my_ccd.img = imread("../data/ball.png", 1);
  
  double *params = new double[9];
  params[0] = 0.5;
  params[1] = 4;
  params[2] = 4;
  params[3] = 3;
  params[4] = 0.5;
  params[5] = 0.25;
  params[6] = 40;
  params[7] = 1;
  params[8] = 80;

  my_ccd.set_params(params);
  
  // convert the image into Mat fortmat
  //cv::Mat img(img1);

  // store the control points trasformed in the shape space
  // CvPoint2D64f pts_tmp;

  ////////////////////////////////////////////////////////////////
  // mannaully initialize the control points
  ///////////////////////////////////////////////////////////////
  cv::namedWindow("Original", 1);
  cvSetMouseCallback( "Original", on_mouse,  (void*)&my_ccd.canvas);
  // cvShowImage("Original",img1);
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
  if(pts.size() > 3)
  {
    pts.push_back(pts[0]);
    pts.push_back(pts[1]);
    pts.push_back(pts[2]);
  }


  my_ccd.init_pts(pts);
  my_ccd.run_ccd();

  delete(params);
  return 0;
}
