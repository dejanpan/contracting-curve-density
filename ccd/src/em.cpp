#include "opencv/cv.h"
#include "opencv/ml.h"
#include "opencv/highgui.h"
using namespace std;
int main( int argc, char** argv )
{
  const int MAX_CLUSTERS = 10;
  cv::Vec3b colorTab[] =
      {
        cv::Vec3b(0, 0, 255),
        cv::Vec3b(0,255,0),
        cv::Vec3b(255,100,100),
        cv::Vec3b(255,0,255),
        cv::Vec3b(0,255,255),
        cv::Vec3b(1,55,25),
        cv::Vec3b(0,255,100),
        cv::Vec3b(5,9,199),
        cv::Vec3b(55,255,55),
        cv::Vec3b(4,25,55)
      };
  
  cv::Mat im = cv::imread(argv[1]);
  cv::Mat_<cv::Vec3b>& imP = (cv::Mat_<cv::Vec3b>&)im;
  
  cv::Mat imageShow= cv::Mat::zeros(im.rows, im.cols, CV_8UC3);
  int sampleCount = im.rows*im.cols;
  CvMat *image = cvCreateMat(sampleCount, 3, CV_32FC1);
  // im.convertTo(image, CV_32FC3, 1, 0);
  float *imagePtr = image->data.fl;
  int step = image->step/sizeof(float);
  for (int i = 0; i < im.rows; ++i)
  {
    for (int j = 0; j < im.cols; ++j)
    {
      (imagePtr + (i*im.cols+j)*step)[0] = imP(i,j)[0];
      (imagePtr + (i*im.cols+j)*step)[1] = imP(i,j)[1];
      (imagePtr + (i*im.cols+j)*step)[2] = imP(i,j)[2];
    }
  }
  // imshow("em", image);
  // cv::waitKey(-1);
  int clusterCount = MIN(atoi(argv[2]), MAX_CLUSTERS);
  
  // cv::Mat labels = cv::Mat::zeros( sampleCount, 1, CV_32SC1);
  CvMat* labels = cvCreateMat( sampleCount, 1, CV_32SC1 );
  CvEM em_model;
  CvEMParams params;


  // initialize model's parameters
  params.covs      = NULL;
  params.means     = NULL;
  params.weights   = NULL;
  params.probs     = NULL;
  params.nclusters = clusterCount;
  params.cov_mat_type       = CvEM::COV_MAT_SPHERICAL;
  params.start_step         = CvEM::START_AUTO_STEP;
  params.term_crit.max_iter = 10;
  params.term_crit.epsilon  = 0.1;
  params.term_crit.type     = CV_TERMCRIT_ITER|CV_TERMCRIT_EPS;

  // cluster the data
  em_model.train( image, 0, params, labels);

#if 0
  // the piece of code shows how to repeatedly optimize the model
  // with less-constrained parameters
  //(COV_MAT_DIAGONAL instead of COV_MAT_SPHERICAL)
  // when the output of the first stage is used as input for the second.
  CvEM em_model2;
  params.cov_mat_type = CvEM::COV_MAT_DIAGONAL;
  params.start_step = CvEM::START_E_STEP;
  params.means = em_model.get_means();
  params.covs = (const CvMat**)em_model.get_covs();
  params.weights = em_model.get_weights();

  em_model2.train( image, 0, params, labels);
  // to use em_model2, replace em_model.predict()
  // with em_model2.predict() below
#endif
  // classify every image pixel

  for (int i = 0 ; i < im.rows; ++i)
  {
    for (int j = 0; j < im.cols; ++j)
    {
      int imgRow = i*im.cols+j;
      imageShow.at<cv::Vec3b>(i,j) = colorTab[labels->data.i[imgRow]];
    }
  }
  cv::namedWindow("cluster",1);
  cv::imshow("clusters", imageShow);
  cv::waitKey(0);
  
  cvReleaseMat(&image);
  cvReleaseMat(&labels);
  return 0;
}
