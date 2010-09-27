#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "bspline.h"
#include <iostream>
#include <string>
#include <cstdio>

#define INNER_PIXEL 60
#define OUTER_PIXEL 10
#define CURVE_PIXEL 250
#define VIN_PIXEL 150
#define VOUT_PIXEL 200
using namespace cv;

void imageMap(IplImage *image,
              cv::Mat &img_map,
              BSpline &curve_points,
              int n_points)
{
  CvMemStorage* storage = cvCreateMemStorage(0);
  CvSeq* contour = cvCreateSeq(CV_SEQ_ELTYPE_POINT,
                               sizeof(CvSeq),
                               sizeof(CvPoint2D32f),
                               storage );
  
  for (int i = 0; i < n_points; ++i)
    cvSeqPush(contour, &curve_points[i]);
  
  CvPoint2D32f p;

  for (int i = 0; i < img_map.rows; ++i)
  {
    for (int j = 0; j < img_map.cols; ++j)
    {
      p.x = (float)j, p.y = (float)i;
      // if(i == 390 && j == 370)
      //   std::cout << i << " " << j << ": "  <<  cvPointPolygonTest(contour, p, 0) << std::endl;
      if(cvPointPolygonTest(contour, p, 0) == -100)
      {
        if(img_map.at<uchar>(i, j) == 0)
        {
          img_map.at<uchar>(i,j) = OUTER_PIXEL;
        }
      }
      else if(cvPointPolygonTest(contour, p, 0) == 100)
      {
        if(img_map.at<uchar>(i, j) == 0)
          img_map.at<uchar>(i,j) = INNER_PIXEL;
      }
      else
      {
        img_map.at<uchar>(i,j) = CURVE_PIXEL;
      }
    }
  }
  cvReleaseMemStorage(&storage);
}


void computeMean(cv::Mat &image_map,
                 cv::Mat &img,
                 cv::Mat &mean_in,
                 cv::Mat &mean_out,
                 std::vector<CvPoint> &curve,
                 CvSeq* arr_in,
                 CvSeq* arr_out)
{
  CvPoint tmp;
  tmp.x = 0, tmp.y = 0;
  for (int i = 0 ; i < img.rows; ++i){
    for (int j = 0; j < img.cols; ++j)
      if(image_map.at<uchar>(i,j) == INNER_PIXEL){
        // std::cout << i << " " << j << " " <<  cvmGet(image_map, i,j) << " "<< std::endl;
        mean_in.at<double>(0,0) += img.at<Vec3b>(i,j)[0];
        mean_in.at<double>(0,1) += img.at<Vec3b>(i,j)[1];
        mean_in.at<double>(0,2) += img.at<Vec3b>(i,j)[2];
        int data[3] = {img.at<Vec3b>(i,j)[0], img.at<Vec3b>(i,j)[1], img.at<Vec3b>(i,j)[2]};
        cvSeqPush(arr_in, data);
      }
      else if(image_map.at<uchar>(i,j) == OUTER_PIXEL){
        mean_out.at<double>(0,0) += img.at<Vec3b>(i,j)[0];
        mean_out.at<double>(0,1) += img.at<Vec3b>(i,j)[1];
        mean_out.at<double>(0,2) += img.at<Vec3b>(i,j)[2];
        int data[3] = {img.at<Vec3b>(i,j)[0], img.at<Vec3b>(i,j)[1], img.at<Vec3b>(i,j)[2]};
        cvSeqPush(arr_out, data);
      }
      else if(image_map.at<uchar>(i,j) == CURVE_PIXEL){
        tmp.x = i, tmp.y = j;
        curve.push_back(tmp);
        // std::cout <<  "curve x = "<< i << " " << j << std::endl;
      }
  }
  mean_in.at<double>(0,0) /= arr_in->total;
  mean_in.at<double>(0,1) /= arr_in->total;
  mean_in.at<double>(0,2) /= arr_in->total;
  mean_out.at<double>(0,0) /= arr_out->total;
  mean_out.at<double>(0,1) /= arr_out->total;
  mean_out.at<double>(0,2) /= arr_out->total;
}



void computeJacob(
    cv::Mat &img,
    cv::Mat &image_map,
    cv::Mat &pixel_diff,
    cv::Mat &jacob,
    cv::Mat &mean_in,
    cv::Mat &mean_out,
    std::vector<CvPoint> &curve
)
{
  double *grd = new double[6];
  for (int i = 0; i < (int)curve.size(); ++i)
  {
    pixel_diff.at<double>(3*i, 0) = img.at<Vec3b>(curve[i].x, curve[i].y)[0] - mean_in.at<double>(0,0);
    pixel_diff.at<double>(3*i+1, 0) = img.at<Vec3b>(curve[i].x, curve[i].y)[1] - mean_in.at<double>(0,1);
    pixel_diff.at<double>(3*i+2, 0) = img.at<Vec3b>(curve[i].x, curve[i].y)[2] - mean_in.at<double>(0,2);
    // TODO: if the curve touch the border.
    grd[0] = img.at<Vec3b>(curve[i].x, curve[i].y)[0] - img.at<Vec3b>(curve[i].x - 1, curve[i].y)[0];
    grd[1] = img.at<Vec3b>(curve[i].x, curve[i].y)[0] - img.at<Vec3b>(curve[i].x, curve[i].y - 1)[0];
    grd[2] = img.at<Vec3b>(curve[i].x, curve[i].y)[1] - img.at<Vec3b>(curve[i].x - 1, curve[i].y)[1];
    grd[3] = img.at<Vec3b>(curve[i].x, curve[i].y)[1] - img.at<Vec3b>(curve[i].x, curve[i].y - 1)[1];
    grd[4] = img.at<Vec3b>(curve[i].x, curve[i].y)[2] - img.at<Vec3b>(curve[i].x - 1, curve[i].y)[2];
    grd[5] = img.at<Vec3b>(curve[i].x, curve[i].y)[2] - img.at<Vec3b>(curve[i].x, curve[i].y - 1)[2];

    // if(i == 1){
    //   std::cout <<"grad:"<< grd[0] << " " << grd[1] << " " << grd[2] << " "  << grd[3] << " "  << grd[4] << " "  << grd[5] << " " << std::endl;
    // }

    for (int j = i*3; j < (i+1)*3; ++j)
    {
      double *jacob_ptr = jacob.ptr<double>(j);
      int ind = j%3;
      jacob_ptr[0] = grd[ind*2];
      jacob_ptr[1] = grd[ind*2+1];
      jacob_ptr[2] = curve[i].x*grd[ind*2];
      jacob_ptr[3] = curve[i].y*grd[ind*2+1];
      jacob_ptr[4] = curve[i].x*grd[ind*2+1];
      jacob_ptr[5] = curve[i].y*grd[ind*2];
    }
  }
  for (int i = (int)curve.size(); i < (int)curve.size()*2; ++i)
  {
    int tmp_ind = i-curve.size();
    pixel_diff.at<double>(3*i, 0) = img.at<Vec3b>(curve[tmp_ind].x, curve[tmp_ind].y)[0] - mean_out.at<double>(0,0);
    pixel_diff.at<double>(3*i+1, 0) = img.at<Vec3b>(curve[tmp_ind].x, curve[tmp_ind].y)[1] - mean_out.at<double>(0,1);
    pixel_diff.at<double>(3*i+2, 0) = img.at<Vec3b>(curve[tmp_ind].x, curve[tmp_ind].y)[2] - mean_out.at<double>(0,2);
    // if(i == 553){
    //   std::cout << (int)img.at<Vec3b>(curve[tmp_ind].x, curve[tmp_ind].y)[2] << " " << mean_out.at<double>(0,2) <<" " << pixel_diff.at<double>(3*i+2, 0)<< std::endl;
    // }

    for (int j = 0; j < 6; ++j)
    {
      jacob.at<double>(3*i,j) = jacob.at<double>(3*tmp_ind, j);
      jacob.at<double>(3*i+1,j) = jacob.at<double>(3*tmp_ind+1, j);
      jacob.at<double>(3*i+2,j) = jacob.at<double>(3*tmp_ind+2, j);
      // if(i >= 275*2)
      // {
      //   std::cout<< tmp_ind<< ": "<< jacob.at<double>(tmp_ind,j) << "  ";
      // }
    }
    //    std::cout <<  std::endl;
  }
}


void chol3x3(cv::Mat &mat, int mat_type){
  if(mat_type == 1 )
  {
    mat.at<double>(0,0) = mat.at<double>(1,1) = mat.at<double>(2,2) = 1;
    mat.at<double>(0,1) = mat.at<double>(0,2) = mat.at<double>(1,0)
                        = mat.at<double>(1,2) = mat.at<double>(2,0)
                        = mat.at<double>(2,1) = 0.001;
  }
  else{  
    mat.at<double>(0,0) = cvSqrt(mat.at<double>(0,0));
    mat.at<double>(0,1) = mat.at<double>(1,0)/mat.at<double>(0,0);
    mat.at<double>(0,2) = mat.at<double>(2,0)/mat.at<double>(0,0);
    mat.at<double>(1,1) = cvSqrt(mat.at<double>(1,1) - mat.at<double>(0,1)*mat.at<double>(0,1));
    mat.at<double>(1,2) = (mat.at<double>(2,1) - mat.at<double>(0,2)*mat.at<double>(0,1))/mat.at<double>(1,1);
    mat.at<double>(2,2) = cvSqrt(mat.at<double>(2,2) - mat.at<double>(1,2)*mat.at<double>(1,2) - mat.at<double>(0,2)*mat.at<double>(0,2));
    mat.at<double>(1,0) = mat.at<double>(2,0) = mat.at<double>(2,1) = 0.0;
  }
  
}

cv::Mat wls(cv::Mat &jacob,
            cv::Mat &pixel_diff,
            cv::Mat &covar_in,
            cv::Mat &covar_out,
            int count
            )
{
  cv::Mat covar_in_inv = Mat::zeros(3,3 , CV_64F);
  cv::Mat covar_out_inv = Mat::zeros(3,3 , CV_64F);
  cv::Mat in_eig, out_eig;
  covar_in_inv = covar_in.inv(DECOMP_SVD);
  covar_out_inv = covar_out.inv(DECOMP_SVD);

  eigen(covar_in_inv, in_eig);
  eigen(covar_out_inv,out_eig);
  int in_mat_type =0, out_mat_type = 0;
  for (int i = 0; i < 3; ++i)
  {
    if(in_eig.at<double>(i,0) < 0)
      in_mat_type = 1;
    if(out_eig.at<double>(i,0) < 0)
      out_mat_type = 1;
  }
  // std::cout << " in_eig: "<< in_eig.at<double>(0,0) <<" " << in_eig.at<double>(1,0) <<" " << in_eig.at<double>(2,0) <<" " << std::endl;
  // if(in_mat_type == 1 )
  // {
  //   exit(-1);
  // }
  // std::cout << " out_eig: "<< out_eig.at<double>(0,0) <<" " << out_eig.at<double>(1,0) <<" " << out_eig.at<double>(2,0) <<" " << std::endl;
  // if(out_mat_type == 1 )
  // {
  //   exit(-1);
  // }


  chol3x3(covar_in_inv, in_mat_type);
  chol3x3(covar_out_inv, out_mat_type);
    std::cout <<  std::endl;
  for (int i = 0; i < 3; ++i){
    for (int j = 0 ; j < 3; ++j){
      std::cout << covar_in_inv.at<double>(i,j) << " ";
    }
    std::cout <<  std::endl;
  }
  std::cout <<  std::endl;

  //  covar_in_inv.at<double>(0,0) = cvSqrt(covar_in_inv.at<double>(0,0));
  cv::Mat tmp_mat = Mat::zeros(6, 6*count, CV_64F);
  for (int i = 0; i < 6; ++i)
  {
    for (int j = 0; j < count; ++j)
      for (int k = 0; k < 3; ++k)
        tmp_mat.at<double>(i,3*j+k) = jacob.at<double>(3*j,i)*(covar_in_inv.at<double>(0,k))
                                      + jacob.at<double>(3*j+1,i)*(covar_in_inv.at<double>(1,k))
                                      + jacob.at<double>(3*j+2,i)*(covar_in_inv.at<double>(2,k));
    for (int j = count; j < 2*count; ++j)
      for (int k = 0; k < 3; ++k)
        tmp_mat.at<double>(i,3*j+k) = jacob.at<double>(3*j,i)*(covar_out_inv.at<double>(0,k))
                                      + jacob.at<double>(3*j+1,i)*(covar_out_inv.at<double>(1,k))
                                      + jacob.at<double>(3*j+2,i)*(covar_out_inv.at<double>(2,k));
  }

  // for (int i = 0; i < 6; ++i){
  //   for (int j = 0; j < 6*count; ++j){
  //     std::cout << tmp_mat.at<double>(i,j) << " " ;
  //   }
  //   std::cout <<std::endl;
  // }
  std::cout <<" norm: "<<  norm(tmp_mat) << std::endl;
  
  cv::Mat X = (tmp_mat*jacob).inv(DECOMP_SVD)*tmp_mat*pixel_diff;
  tmp_mat.release();
  covar_out_inv.release();
  covar_in_inv.release();
  return X;
}


void computeCov(cv::Mat &arr_in,
                cv::Mat &arr_out,
                cv::Mat &mean_in,
                cv::Mat &mean_out,
                cv::Mat &covar_in,
                cv::Mat &covar_out,
                int count)
{
  calcCovarMatrix(arr_in,  covar_in, mean_in, CV_COVAR_NORMAL|CV_COVAR_ROWS, CV_64F);
  calcCovarMatrix(arr_out, covar_out, mean_out, CV_COVAR_NORMAL|CV_COVAR_ROWS, CV_64F);
  for (int i = 0; i < 3; ++i){
    for (int j = 0; j < 3; ++j){
      covar_in.at<double>(i, j) /=  count - 1;
      covar_out.at<double>(i, j) /=  count - 1;
    }
  }
}
