#include "ccd.h"
using namespace cv;
int main (int argc, char * argv[]) 
{
  const int resolution = 200;
  int n = 12;
  int t = 3;
  CvPoint2D64f *pts = new CvPoint2D64f[n+1];  
  pts[0].x=313;  pts[0].y=271;
  pts[1].x=308;  pts[1].y=304;
  pts[2].x=316;  pts[2].y=333;
  pts[3].x=346;  pts[3].y=353;
  pts[4].x=403;  pts[4].y=351;
  pts[5].x=430;  pts[5].y=325;
  pts[6].x=437;  pts[6].y=291;
  pts[7].x=416;  pts[7].y=243;
  pts[8].x=355;  pts[8].y=230;
  pts[9].x=316;  pts[9].y=260;
  pts[10].x=313;  pts[10].y=271;
  pts[11].x=308;  pts[11].y=304;
  pts[12].x=316;  pts[12].y=333;

  IplImage dst_img, *img1= cvLoadImage(argv[1], 1);
  BSpline bs;
  int tmp[3];
  char filename[30];
  int iter = 0 ;
  double tol = 1.0;
  CvSeqReader reader;
  CvMemStorage* storage;
  CvPoint2D64f pts_tmp;
  

  cv::Mat jacob;
  cv::Mat img_map;
  cv::Mat mean_in, mean_out;
  cv::Mat covar_in ,covar_out;
  cv::Mat arr_in, arr_out;
  cv::Mat pixel_diff;
  cv::Mat img(img1);
  cv::Mat x = Mat::zeros(6,1, CV_64F);
  cv::Mat x_old = x ;
  cv::Mat X = x;
  std::vector<CvPoint> curve;

  while(tol > 0.01 && iter < 100)
  {
    for (int i = 0; i < 6; ++i){
      X.at<double>(i,0) = X.at<double>(i,0) - x.at<double>(i,0);
    }
    for (int i = 0; i <= n; ++i){
      pts_tmp.x = X.at<double>(0,0) + (1+X.at<double>(2,0))*pts[i].x + X.at<double>(5,0)*pts[i].y;
      pts_tmp.y = X.at<double>(1,0) + (1+X.at<double>(3,0))*pts[i].y + X.at<double>(4,0)*pts[i].x;
      pts[i].x = round(pts_tmp.x);
      pts[i].y = round(pts_tmp.y);
      // std::cout << " i : "  << pts[i].x << " " <<pts[i].y << std::endl;
    }

    bs = BSpline(n, t , resolution, pts);
    img_map = cv::Mat::zeros(cvGetSize(img1), CV_8U);
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

    arr_in = Mat::zeros(seq_in->total, 3, CV_64F);
    arr_out = Mat::zeros(seq_out->total, 3, CV_64F);
    
    cvStartReadSeq(seq_in, &reader);
    for (int i = 0 ; i < seq_in->total; ++i){
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

    
    pixel_diff = Mat::zeros(6*curve.size(), 1, CV_64F);
    jacob  = Mat::zeros((curve.size())*6, 6, CV_64F);
    // if(iter == 1) exit(-1);
    computeJacob(img, img_map, pixel_diff, jacob, mean_in, mean_out, curve);

    // for (int i = 0; i < 1662 ; ++i){
    //   for (int j = 0; j < 6; ++j){
    //     std::cout << jacob.at<double>(i,j) << " ";  
    //   }
    //   std::cout << std::endl;
    // }

    x = wls(jacob, pixel_diff, covar_in, covar_out, curve.size());
    // if(iter == 1) exit(-1);

    for (int i = 0; i < 6; ++i) std::cout << x.at<double>(i,0) << " ";
    std::cout << std::endl;
    std::cout << "x_old:" << std::endl;
    for (int i = 0; i < 6; ++i) std::cout << x_old.at<double>(i,0) << " ";
    std::cout << std::endl;

    dst_img = img_map;
    snprintf(filename, 30, "bspline%d.png", iter);
    std::cout << filename << std::endl;
    cvSaveImage(filename, &dst_img);
    
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
    tol = norm(x, x_old);
    std::cout << "iteration " << iter << " : "  << (double)tol << " " << cvSqrt(dis.at<double>(0,0)) << std::endl;
    x_old = x;
    iter += 1;
    

    arr_in.release();
    arr_out.release();
    mean_in.release();
    mean_out.release();
    covar_in.release();
    covar_out.release();
    pixel_diff.release();
    jacob.release();
    img_map.release();
  }
  delete [] pts;
  cvReleaseImage(&img1);
  return 0;
}
