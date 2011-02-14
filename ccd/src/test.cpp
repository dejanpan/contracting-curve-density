#include "ccd.h"

using namespace cv;
cv::Mat img1;
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
  if( img1.empty())
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
      cv::circle(img1,cv::Point(x,y),2,cv::Scalar(0,0,255),2);
      pts.push_back(cvPoint2D64f(x,y));
      // cvShowImage("Original",img1);
      cv::imshow("Original",img1);
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

  img1 = imread("../data/ball.png", 1);
  cv::Mat img = imread("../data/ball.png", 1);
  // convert the image into Mat fortmat
  //cv::Mat img(img1);

  // store the control points trasformed in the shape space
  // CvPoint2D64f pts_tmp;

  ////////////////////////////////////////////////////////////////
  // mannaully initialize the control points
  ///////////////////////////////////////////////////////////////
  cv::namedWindow("Original", 1);
  cvSetMouseCallback( "Original", on_mouse, 0 );
  // cvShowImage("Original",img1);
  cv::imshow("Original", img1);
  char key ;
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

  CCD my_ccd(img);
  my_ccd.init_pts(pts);
  my_ccd.run_ccd();
  return 0;
}
