#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "bspline.h"
#include <iostream>
#include <string>
#include <cstdio>
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
  
  CvPoint2D32f p, tmp;

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
          img_map.at<uchar>(i,j) = 10;
        }
      }
      else if(cvPointPolygonTest(contour, p, 0) == 100)
      {
        if(img_map.at<uchar>(i, j) == 0)
        {
          img_map.at<uchar>(i,j) = 60;
        }
      }
      else
      {
        img_map.at<uchar>(i,j) = 250;
        if(((j > 1) && (j < (img_map.cols - 1)))
           && ((i > 1) && (i < (img_map.rows- 1))))
          for (int y = i-1; y <= i+1; ++y)
            for (int x = j -1; x <= j+1; ++x)
            {
              tmp.x = x; tmp.y = y;
              if((x != j) && (y != i))
              {
                double poly_test = cvPointPolygonTest(contour, tmp, 0);
                if(poly_test  == 100)
                  img_map.at<uchar>(y,x) = 150;
                else if( poly_test == -100)
                  img_map.at<uchar>(y,x) = 200;
              }
            }
      }
    }
  }
  cvReleaseMemStorage(&storage);
}


void computeMean(cv::Mat &image_map,
                   cv::Mat &img,
                   cv::Mat &mean_in,
                   cv::Mat &mean_out,
                 std::vector<CvPoint> &curve_in,
                 std::vector<CvPoint> &curve_out,
                 CvSeq* inputMatrix_in,
                 CvSeq* inputMatrix_out)
{
  CvPoint tmp;
  tmp.x = 0, tmp.y = 0;
  for (int i = 0 ; i < img.rows; ++i){
    for (int j = 0; j < img.cols; ++j)
      if(image_map.at<uchar>(i,j) == 60){
        // std::cout << i << " " << j << " " <<  cvmGet(image_map, i,j) << " "<< std::endl;
        mean_in.at<double>(0,0) += img.at<Vec3b>(i,j)[0];
        mean_in.at<double>(1,0) += img.at<Vec3b>(i,j)[1];
        mean_in.at<double>(2,0) += img.at<Vec3b>(i,j)[2];
/* 
 *         if(i == 299 && j == 401){
 *           std::cout << (int)img.at<Vec3b>(i,j)[0] << " " << (int)img.at<Vec3b>(i,j)[1] << " " << (int)img.at<Vec3b>(i,j)[2] << std::endl;
 * std::cout << mean_in.at<double>(0,0) << " " << mean_in.at<double>(1,0) << mean_in.at<double>(2,0)<< std::endl;
 * }
 */
        int data[3] = {img.at<Vec3b>(i,j)[0], img.at<Vec3b>(i,j)[1], img.at<Vec3b>(i,j)[2]};
        cvSeqPush(inputMatrix_in, data);
      }
      else if(image_map.at<uchar>(i,j) == 10){
        mean_out.at<double>(0,0) += img.at<Vec3b>(i,j)[0];
        mean_out.at<double>(1,0) += img.at<Vec3b>(i,j)[1];
        mean_out.at<double>(2,0) += img.at<Vec3b>(i,j)[2];
        int data[3] = {img.at<Vec3b>(i,j)[0], img.at<Vec3b>(i,j)[1], img.at<Vec3b>(i,j)[2]};
        cvSeqPush(inputMatrix_out, data);
      }
      else if(image_map.at<uchar>(i,j) == 150){
        tmp.x = i, tmp.y = j;
        curve_in.push_back(tmp);
// std::cout <<  "curve_in x = "<< i << " " << j << std::endl;
      }
      else if(image_map.at<uchar>(i,j) == 200){
        tmp.x = i, tmp.y = j;
        curve_out.push_back(tmp);
      }

  }
  mean_in.at<double>(0,0) /= inputMatrix_in->total;
  mean_in.at<double>(1,0) /= inputMatrix_in->total;
  mean_in.at<double>(2,0) /= inputMatrix_in->total;
  mean_out.at<double>(0,0) /= inputMatrix_out->total;
  mean_out.at<double>(1,0) /= inputMatrix_out->total;
  mean_out.at<double>(2,0) /= inputMatrix_out->total;
}



void computeJacob(
    cv::Mat &img,
    cv::Mat &image_map,
    cv::Mat &pixel_diff,
    cv::Mat &jacob,
    cv::Mat &mean_in,
    cv::Mat &mean_out,
    std::vector<CvPoint> &curve_in,
    std::vector<CvPoint> &curve_out
                  )
{
  double *grd = new double[6];
  // for (int i = 0; i < 6; ++i) grd[i] = .0;
  //std::cout << "not seg" << std::endl;
  for (int i = 0; i < (int)curve_in.size(); ++i)
  {
    pixel_diff.at<double>(3*i, 0) = img.at<Vec3b>(curve_in[i].x, curve_in[i].y)[0] - mean_in.at<double>(0,0);
    pixel_diff.at<double>(3*i+1, 0) = img.at<Vec3b>(curve_in[i].x, curve_in[i].y)[1] - mean_in.at<double>(0,1);
    pixel_diff.at<double>(3*i+2, 0) = img.at<Vec3b>(curve_in[i].x, curve_in[i].y)[2] - mean_in.at<double>(0,2);
    // TODO: if the curve touch the border.
    grd[0] = img.at<Vec3b>(curve_in[i].x, curve_in[i].y)[0] - img.at<Vec3b>(curve_in[i].x - 1, curve_in[i].y)[0];
    grd[1] = img.at<Vec3b>(curve_in[i].x, curve_in[i].y)[0] - img.at<Vec3b>(curve_in[i].x, curve_in[i].y - 1)[0];
    grd[2] = img.at<Vec3b>(curve_in[i].x, curve_in[i].y)[1] - img.at<Vec3b>(curve_in[i].x - 1, curve_in[i].y)[1];
    grd[3] = img.at<Vec3b>(curve_in[i].x, curve_in[i].y)[1] - img.at<Vec3b>(curve_in[i].x, curve_in[i].y - 1)[1];
    grd[4] = img.at<Vec3b>(curve_in[i].x, curve_in[i].y)[2] - img.at<Vec3b>(curve_in[i].x - 1, curve_in[i].y)[2];
    grd[5] = img.at<Vec3b>(curve_in[i].x, curve_in[i].y)[2] - img.at<Vec3b>(curve_in[i].x, curve_in[i].y - 1)[2];
    for (int j = i*3; j < (int)(i+1)*3; ++j)
    {
      double *jacob_ptr = jacob.ptr<double>(j);
      int ind = j%3;
      jacob_ptr[0] = grd[ind*2];
      jacob_ptr[1] = grd[ind*2+1];
      jacob_ptr[2] = curve_in[i].x*grd[ind*2];
      jacob_ptr[3] = curve_in[i].y*grd[ind*2+1];
      jacob_ptr[4] = curve_in[i].x*grd[ind*2+1];
      jacob_ptr[5] = curve_in[i].y*grd[ind*2];
    }
  }
  for (int i = 0; i < (int)curve_out.size(); ++i)
  {
    pixel_diff.at<double>(3*(i+curve_in.size()), 0) = img.at<Vec3b>(curve_out[i].x, curve_out[i].y)[0] - mean_out.at<double>(0,0);
    pixel_diff.at<double>(3*(i+curve_in.size())+1, 0) = img.at<Vec3b>(curve_out[i].x, curve_out[i].y)[1] - mean_out.at<double>(1,0);
    pixel_diff.at<double>(3*(i+curve_in.size())+2, 0) = img.at<Vec3b>(curve_out[i].x, curve_out[i].y)[2] - mean_out.at<double>(2,0);
    // TODO: if the curve touch the border.
    grd[0] = img.at<Vec3b>(curve_out[i].x, curve_out[i].y)[0] - img.at<Vec3b>(curve_out[i].x - 1, curve_out[i].y)[0];
    grd[1] = img.at<Vec3b>(curve_out[i].x, curve_out[i].y)[0] - img.at<Vec3b>(curve_out[i].x, curve_out[i].y - 1)[0];
    grd[2] = img.at<Vec3b>(curve_out[i].x, curve_out[i].y)[1] - img.at<Vec3b>(curve_out[i].x - 1, curve_out[i].y)[1];
    grd[3] = img.at<Vec3b>(curve_out[i].x, curve_out[i].y)[1] - img.at<Vec3b>(curve_out[i].x, curve_out[i].y - 1)[1];
    grd[4] = img.at<Vec3b>(curve_out[i].x, curve_out[i].y)[2] - img.at<Vec3b>(curve_out[i].x - 1, curve_out[i].y)[2];
    grd[5] = img.at<Vec3b>(curve_out[i].x, curve_out[i].y)[2] - img.at<Vec3b>(curve_out[i].x, curve_out[i].y - 1)[2];
    for (int j = (i+curve_in.size())*3; j < (int)(i+curve_in.size()+1)*3; ++j)
    {
      double *jacob_ptr = jacob.ptr<double>(j);
      int ind = j%3;
      jacob_ptr[0] = grd[ind*2];
      jacob_ptr[1] = grd[ind*2+1];
      jacob_ptr[2] = curve_out[i].x*grd[ind*2];
      jacob_ptr[3] = curve_out[i].y*grd[ind*2+1];
      jacob_ptr[4] = curve_out[i].x*grd[ind*2+1];
      jacob_ptr[5] = curve_out[i].y*grd[ind*2];
    }
  }
  // delete [] grd;
  // if()
}

cv::Mat wls(cv::Mat &jacob,
            cv::Mat &pixel_diff,
            cv::Mat &covar_in,
            cv::Mat &covar_out,
            int in_count,
            int out_count
            )
{
  // cv::Mat weight = Mat::zeros(3*(in_count + out_count), 3*(in_count+out_count), CV_64F);
  cv::Mat covar_in_inv = Mat::zeros(3,3 , CV_64F);
  cv::Mat covar_out_inv = Mat::zeros(3,3 , CV_64F);
  std::cout<<"inverse: " << std::endl;
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
  if(in_mat_type == 1 )
  {
    covar_in_inv.at<double>(0,0) =     covar_in_inv.at<double>(1,1) =     covar_in_inv.at<double>(2,2) = 1;
    covar_in_inv.at<double>(0,1) =     covar_in_inv.at<double>(0,2) =     covar_in_inv.at<double>(1,0) = covar_in_inv.at<double>(1,2) =     covar_in_inv.at<double>(2,0) =     covar_in_inv.at<double>(2,1) = 0.001;
  }
  else{  
  covar_in_inv.at<double>(0,0) = cvSqrt(covar_in_inv.at<double>(0,0));
  covar_in_inv.at<double>(0,1) = covar_in_inv.at<double>(1,0)/covar_in_inv.at<double>(0,0);
  covar_in_inv.at<double>(0,2) = covar_in_inv.at<double>(2,0)/covar_in_inv.at<double>(0,0);
  covar_in_inv.at<double>(1,1) = cvSqrt(covar_in_inv.at<double>(1,1) - covar_in_inv.at<double>(0,1)*covar_in_inv.at<double>(0,1));
  covar_in_inv.at<double>(1,2) = (covar_in_inv.at<double>(2,1) - covar_in_inv.at<double>(0,2)*covar_in_inv.at<double>(0,1))/covar_in_inv.at<double>(1,1);
  covar_in_inv.at<double>(2,2) = cvSqrt(covar_in_inv.at<double>(2,2) - covar_in_inv.at<double>(1,2)*covar_in_inv.at<double>(1,2) - covar_in_inv.at<double>(0,2)*covar_in_inv.at<double>(0,2));
  covar_in_inv.at<double>(1,0) = covar_in_inv.at<double>(2,0) = covar_in_inv.at<double>(2,1) = 0.0;
}
  if(out_mat_type == 1 )
  {
    covar_out_inv.at<double>(0,0) =     covar_out_inv.at<double>(1,1) =     covar_out_inv.at<double>(2,2) = 1;
    covar_out_inv.at<double>(0,1) =     covar_out_inv.at<double>(0,2) =     covar_out_inv.at<double>(1,0) =   covar_out_inv.at<double>(1,2) =     covar_out_inv.at<double>(2,0) =     covar_out_inv.at<double>(2,1) = 0.001;
  }
  else
    {
    covar_out_inv.at<double>(0,0) = cvSqrt(covar_out_inv.at<double>(0,0));
  covar_out_inv.at<double>(0,1) = covar_out_inv.at<double>(1,0)/covar_out_inv.at<double>(0,0);
  covar_out_inv.at<double>(0,2) = covar_out_inv.at<double>(2,0)/covar_out_inv.at<double>(0,0);
  covar_out_inv.at<double>(1,1) = cvSqrt(covar_out_inv.at<double>(1,1) - covar_out_inv.at<double>(0,1)*covar_out_inv.at<double>(0,1));
  covar_out_inv.at<double>(1,2) = (covar_out_inv.at<double>(2,1) - covar_out_inv.at<double>(0,2)*covar_out_inv.at<double>(0,1))/covar_out_inv.at<double>(1,1);
  covar_out_inv.at<double>(2,2) = cvSqrt(covar_out_inv.at<double>(2,2) - covar_out_inv.at<double>(1,2)*covar_out_inv.at<double>(1,2) - covar_out_inv.at<double>(0,2)*covar_out_inv.at<double>(0,2));
  covar_out_inv.at<double>(1,0) = covar_out_inv.at<double>(2,0) = covar_out_inv.at<double>(2,1) = 0.0;
  }
  //  covar_in_inv.at<double>(0,0) = cvSqrt(covar_in_inv.at<double>(0,0));
  cv::Mat tmp_mat = Mat::zeros(6, 3*(in_count+ out_count), CV_64F);
  for (int i = 0; i < 6; ++i){
    for (int j = 0; j < in_count; ++j){
      for (int k = 0; k < 3; ++k){
        tmp_mat.at<double>(i,3*j+k) = jacob.at<double>(3*j,i)*(covar_in_inv.at<double>(0,k)) + jacob.at<double>(3*j+1,i)*(covar_in_inv.at<double>(1,k)) + jacob.at<double>(3*j+2,i)*(covar_in_inv.at<double>(2,k));
      }
    }
    for (int j = in_count; j < (in_count+out_count); ++j){
      for (int k = 0; k < 3; ++k){
        tmp_mat.at<double>(i,3*j+k) = jacob.at<double>(3*j,i)*(covar_out_inv.at<double>(0,k)) + jacob.at<double>(3*j+1,i)*(covar_out_inv.at<double>(1,k)) + jacob.at<double>(3*j+2,i)*(covar_out_inv.at<double>(2,k));
      }
    }
  }
  
  cv::Mat X = (tmp_mat*jacob).inv(DECOMP_SVD)*tmp_mat*pixel_diff;
  // for (int i = 0; i < 6; ++i){
  //   for (int j = 0 ; j < 10; ++j){
  //     std::cout <<  tmp_mat.at<double>(i,j) << " ";
  //   }
  //   std::cout  << std::endl;
  // }
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
                int in_count,
                int out_count)
{
  calcCovarMatrix(arr_in,  covar_in, mean_in, CV_COVAR_NORMAL|CV_COVAR_ROWS, CV_64F);
  calcCovarMatrix(arr_out, covar_out, mean_out, CV_COVAR_NORMAL|CV_COVAR_ROWS, CV_64F);
  for (int i = 0; i < 3; ++i){
    for (int j = 0; j < 3; ++j){
      covar_in.at<double>(i, j) /=  in_count - 1;
      covar_out.at<double>(i, j) /=  out_count - 1;
    }
  }
}
// please check the boundary
