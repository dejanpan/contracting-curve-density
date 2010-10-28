#include "ccd_panin.h"
using namespace cv;
IplImage *img1;
std::vector<CvPoint2D64f> pts;
void on_mouse( int event, int x, int y, int flags, void* param )
{
    if( !img1 )
        return;

    if( img1->origin )
        y = img1->height - y;

    switch( event )
    {
		case CV_EVENT_LBUTTONDOWN:
			// std::cout << "Event = " << event << std::endl;
			break;
		case CV_EVENT_LBUTTONUP:
			// std::cout << "Event = " << event << std::endl;
			cvCircle(img1,cvPoint(x,y),2,cvScalar(0,0,255),2);
			pts.push_back(cvPoint2D64f(x,y));
			cvShowImage("Original",img1);
			break;
    }
}

int main (int argc, char * argv[]) 
{
  const int resolution = 100;
  int t = 3;
  IplImage dst_img;
  img1= cvLoadImage(argv[1], 1);
  cv::Mat img(img1);
  BSpline bs;
  int tmp[3];
  char filename[30];
  int iter = 0 ;
  double tol = 1.0;
  CvSeqReader reader;
  CvMemStorage* storage;
  CvPoint2D64f pts_tmp;
  cvNamedWindow("Original", 1);
  cvSetMouseCallback( "Original", on_mouse, 0 );
  cvShowImage("Original",img1);
  char key ;
  while (1)
  {
    key = cvWaitKey(10);
    if (key == 27) break;
  }

  if(pts.size() > 3)
  {
    pts.push_back(pts[0]);
    pts.push_back(pts[1]);
    pts.push_back(pts[2]);
  }
  for (size_t i = 0; i < pts.size(); ++i){
    std::cout<< pts[i].x << " " << pts[i].y << std::endl;
  }
  
  // bs = BSpline(pts, t , resolution);
  cv::Mat jacob;
  cv::Mat img_map;
  cv::Mat mean_in, mean_out;
  cv::Mat covar_in ,covar_out;
  cv::Mat arr_in, arr_out;
  cv::Mat pixel_diff;
  // dx is the result of weigted least square, corresponding the change of
  // model parameters
  // dx = (J'JW)^{-1}J'W(c-m)
  // X is the model paramter, inital values is [0 0 0 0 0 0]
  // after each iteration, we change X to X-dx
  cv::Mat dx = Mat::zeros(6,1, CV_64F);
  cv::Mat dx_old = Mat::zeros(6,1, CV_64F);
  cv::Mat X = Mat::zeros(6,1, CV_64F);
  // X.at<double>(2,0) = -0.5;
  // X.at<double>(3,0) = -0.5;
  std::vector<CvPoint> curve;

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////          /** \brief start iteration loop, set tolerance and iteration steps if norm(dx, dx_old) < tol and iter < 100, exit the loop
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////    
  while(tol > 0.01 && iter < 100)
  {
    for (int i = 0; i < 6; ++i)
      X.at<double>(i,0) = X.at<double>(i,0) - dx.at<double>(i,0);
    // recompute the sample points in the curve through new dx, in the fist loop, X always [0 0 0 0 0 0]
    // therefore, points have no change
    for (size_t i = 0; i < pts.size(); ++i)
    {
      pts_tmp.x = X.at<double>(0,0) + (1+X.at<double>(2,0))*pts[i].x + X.at<double>(5,0)*pts[i].y;
      pts_tmp.y = X.at<double>(1,0) + (1+X.at<double>(3,0))*pts[i].y + X.at<double>(4,0)*pts[i].x;
      pts[i].x = round(pts_tmp.x);
      pts[i].y = round(pts_tmp.y);
      for (int j = 0; j < 6; ++j){
        std::cout << X.at<double>(j,0) << " ";
      }
      std::cout << std::endl;
      std::cout << " i : "  << pts[i].x << " " <<pts[i].y << std::endl;
    }
    std::cout << "before " << std::endl;
    BSpline bs(t , resolution, pts);
    // for (int i = 0; i < resolution; ++i){
    //   std::cout << bs[i].x  << " " << bs[i].y << std::endl;
    // }
    // if(iter == 0) exit(-1);
    std::cout << "after " << std::endl;

    img_map = cv::Mat::zeros(cvGetSize(img1), CV_8U);
    // determine the location of pixels in the image ,inside , outside or on the curve
    imageMap(img1, img_map, bs, resolution);
    
    mean_in = Mat::zeros(1, 3, CV_64F);
    mean_out = Mat::zeros(1,3, CV_64F);

    storage = cvCreateMemStorage(0);
    CvSeq *seq_in = cvCreateSeq( CV_32SC3 , sizeof(CvSeq), sizeof(int)*3 , storage);
    CvSeq *seq_out = cvCreateSeq( CV_32SC3 , sizeof(CvSeq), sizeof(int)*3, storage);
    covar_in =  Mat::zeros(3, 3, CV_64F);
    covar_out =  Mat::zeros(3, 3, CV_64F);

    curve.clear();
    computeMean(img_map, img,  mean_in, mean_out, curve, seq_in, seq_out);

    std::cout << mean_in.at<double>(0,0) << " " << mean_in.at<double>(0,1) << " " << mean_in.at<double>(0,2)    << " " << seq_in->total << " "<< curve.size() <<std::endl;
std::cout << mean_out.at<double>(0,0) << " " << mean_out.at<double>(0,1) << " " << mean_out.at<double>(0,2) <<" " << seq_out->total << " "<< curve.size() << std::endl;

// for (int i = 0; i < (int)curve.size(); ++i){
//   std::cout << curve[i].x << " " <<curve[i].y << std::endl;
// }

// arr_in repsents all pixels inside the curve
// arr_out repsents all pixels outside the curve
    arr_in = Mat::zeros(seq_in->total, 3, CV_64F);
    arr_out = Mat::zeros(seq_out->total, 3, CV_64F);
    
    cvStartReadSeq(seq_in, &reader);
    for (int i = 0 ; i < seq_in->total; ++i)
        {
      CV_READ_SEQ_ELEM(tmp,reader);
      arr_in.at<double>(i,0) = tmp[0];
      arr_in.at<double>(i,1) = tmp[1];
      arr_in.at<double>(i,2) = tmp[2];
    }
  
    cvStartReadSeq(seq_out, &reader);
    for (int i= 0; i < seq_out->total; ++i){
      CV_READ_SEQ_ELEM(tmp,reader);
      arr_out.at<double>(i,0) = tmp[0];
      arr_out.at<double>(i,1) = tmp[1];
      arr_out.at<double>(i,2) = tmp[2];
    }

    // for (int i = 10000; i < 10086; ++i){
    //   std::cout << arr_in.at<double>(i, 0 )  << " " << arr_in.at<double>(i, 1 )  << " " << arr_in.at<double>(i, 2 )  << " " << std::endl;
    // }
    // calculate the covariance matrices of all pixels outside and inside the curve respectively
    cv::calcCovarMatrix(arr_in,  covar_in, mean_in, CV_COVAR_NORMAL|CV_COVAR_ROWS|CV_COVAR_USE_AVG, CV_64F);
    cv::calcCovarMatrix(arr_out, covar_out, mean_out, CV_COVAR_NORMAL|CV_COVAR_ROWS|CV_COVAR_USE_AVG, CV_64F);
    cvReleaseMemStorage(&storage);
    for (int i = 0; i < 3; ++i){
      for (int j = 0; j < 3; ++j){
        covar_in.at<double>(i, j) /=  seq_in->total - 1;
        covar_out.at<double>(i, j) /=  seq_out->total - 1;
      }
    }

  //   for (int i = 0; i < 3; ++i){
  //   for (int j = 0 ; j < 3; ++j){
  //     std::cout << covar_in.at<double>(i,j) << " ";
  //   }
  //   std::cout <<  std::endl;
  // }


    // pixel_diff represents the the difference between each pixel and mean value of corresponding region.
    // pixel_diff = c - m, c is on pixel, m is the mean value of its region
    pixel_diff = Mat::zeros(6*curve.size(), 1, CV_64F);
    jacob  = Mat::zeros((curve.size())*6, 6, CV_64F);
    // calculate the jacob matrix
    computeJacob(img, img_map, pixel_diff, jacob, mean_in, mean_out, curve);

    // call weigted least square function to calculate dx
    dx = wls(jacob, pixel_diff, covar_in, covar_out, curve.size());

    for (int i = 0; i < 6; ++i) std::cout << dx.at<double>(i,0) << " ";
    std::cout << std::endl;
    std::cout << "x_old:" << std::endl;
    for (int i = 0; i < 6; ++i) std::cout << dx_old.at<double>(i,0) << " ";
    std::cout << std::endl;

    // this block of code is for the purpose of monitoring the change of curve
    dst_img = img_map;
    snprintf(filename, 250, "../data/bspline%d.png", iter);
    std::cout << filename <<" hello" << std::endl;
    cvSaveImage(filename, &dst_img);

    // caculate the cost function
    // E = sum_i{(c_i-m_in)'(C_in)^{-1}(c_i - m_i)}
    //   + sum_i{(c_i-m_out)'(C_out)^{-1}(c_i - m_out)}
    // theoretically, E should decrease in the process of iteration
    cv::Mat pm_in, pm_out, dis;
    pm_in.create(1,3,CV_64F);
    pm_out.create(1,3,CV_64F);
    for (int i = 0; i < (int)curve.size(); ++i){
      pm_in.zeros(1, 3 , CV_64F);
      pm_out.zeros(1, 3 , CV_64F);
      pm_in.at<double>(0,0) = img.at<Vec3b>(curve[i].x, curve[i].y)[0] - mean_in.at<double>(0, 0);
      pm_in.at<double>(0,1) = img.at<Vec3b>(curve[i].x, curve[i].y)[1] - mean_in.at<double>(0, 1);
      pm_in.at<double>(0,2) = img.at<Vec3b>(curve[i].x, curve[i].y)[2] - mean_in.at<double>(0, 2);
      pm_out.at<double>(0,0) = img.at<Vec3b>(curve[i].x, curve[i].y)[0] - mean_out.at<double>(0, 0);
      pm_out.at<double>(0,1) = img.at<Vec3b>(curve[i].x, curve[i].y)[1] - mean_out.at<double>(0, 1);
      pm_out.at<double>(0,2) = img.at<Vec3b>(curve[i].x, curve[i].y)[2] - mean_out.at<double>(0, 2);
      dis += pm_in*covar_in.inv(DECOMP_SVD)*pm_in.t();
      dis += pm_out*covar_out.inv(DECOMP_SVD)*pm_out.t();
    }
    
    pm_in.release();
    pm_out.release();
    tol = norm(dx, dx_old);
    std::cout << "iteration " << iter << " : "  << (double)tol << " " << cvSqrt(dis.at<double>(0,0)) << std::endl;
    dx_old = dx;
    iter += 1;

    for (int i = 0; i < resolution; ++i){
        int j = (i+1)%resolution;
        cvLine( img1, bs[i],bs[j],CV_RGB( 0, 0, 255 ),1,8,0 ); 
    }
    cvShowImage("Original",img1);
    
    arr_in.release();
    arr_out.release();
    mean_in.release();
    mean_out.release();
    covar_in.release();
    covar_out.release();
    pixel_diff.release();
    jacob.release();
    img_map.release();
    bs.release();
  }
  // delete [] points;
  cvReleaseImage(&img1);
  X.release();
  dx.release();
  dx_old.release();
  return 0;
}
