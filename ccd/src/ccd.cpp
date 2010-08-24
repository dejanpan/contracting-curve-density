#include "ccd.h"
#include "opencv/cv.hpp"
#include "opencv/highgui.h"
#include "bspline.h"
#include <iostream>
using namespace cv;
CvMat imageMap(IplImage *image, BSpline &curve_points, int n_points){
    CvMemStorage* storage = cvCreateMemStorage(0);
    CvSeq* contour = cvCreateSeq( CV_SEQ_ELTYPE_POINT , sizeof(CvSeq), sizeof(CvPoint2D32f) , storage );
    for (int i = 0; i < n_points; ++i){
        cvSeqPush(contour, &curve_points[i]);
    }
    CvMat *image_points = cvCreateMat(image->height, image->width, CV_32FC1);
    cvZero(image_points);
    std::cout << image->width << " " << image->height << std::endl;
    CvPoint2D32f p;
    for (int i = 0; i < image_points->rows; ++i){
        for (int j = 0; j < image_points->cols; ++j){
            p.x = (float)j, p.y = (float)i;
            if(i == 60 && j == 282)
                std::cout << i << " " << j << ": "  <<  cvPointPolygonTest(contour, p, 0) << std::endl;
            if(cvPointPolygonTest(contour, p, 0) == -100)
                cvmSet(image_points,i,j, 1.0);
            else
                cvmSet(image_points,i,j, 0.0);
        }
    }
    cvReleaseMemStorage(&storage);
    return *image_points;
}

typedef struct{
    double r;
    double g;
    double b;
} RGB;

void computeMean(CvMat *image_map, IplImage *image, CvMat *mean_in, CvMat *mean_out, CvSeq* inputMatrix_in, CvSeq* inputMatrix_out)
{
    Mat img(image);
    for (int i = 0 ; i < image->height; ++i){
        for (int j = 0; j < image->width; ++j)
            if(cvmGet(image_map, i,j) == 1){
                mean_in->data.fl[0] += img.at<Vec3b>(i,j)[0];
                mean_in->data.fl[1] += img.at<Vec3b>(i,j)[1];
                mean_in->data.fl[2] += img.at<Vec3b>(i,j)[2];
                int data[3] = {img.at<Vec3b>(i,j)[0], img.at<Vec3b>(i,j)[1], img.at<Vec3b>(i,j)[2]};
                cvSeqPush(inputMatrix_in, data);
            }
            else{
                if(img.at<Vec3b>(i,j)[0] != 0)
                    std::cout << i << " " << j << " " << (int)img.at<Vec3b>(i,j)[0] << std::endl;
                mean_out->data.fl[0] += img.at<Vec3b>(i,j)[0];
                mean_out->data.fl[1] += img.at<Vec3b>(i,j)[1];
                mean_out->data.fl[2] += img.at<Vec3b>(i,j)[2];
                int data[3] = {img.at<Vec3b>(i,j)[0], img.at<Vec3b>(i,j)[1], img.at<Vec3b>(i,j)[2]};
                cvSeqPush(inputMatrix_out, data);
            }
        }
}

void computeCov(IplImage *image, CvArr **arr_in, CvArr **arr_out, CvMat *mean_in, CvMat *mean_out, CvMat *covar_in, CvMat *covar_out)
{
    cvCalcCovarMatrix((const void **)arr_in,2,covar_in,mean_in,CV_COVAR_NORMAL);
    cvCalcCovarMatrix((const void **)arr_out,2,covar_out,mean_out,CV_COVAR_NORMAL);
}

int main (int argc, char * argv[]) 
{
    int n = 23;
    int t =3;
    BSPoint *pts = new BSPoint[n+1];
    pts[0].x=282;  pts[0].y=60;
    pts[1].x=278;   pts[1].y=75;  
    pts[2].x=276;  pts[2].y= 91;
    pts[3].x=272;  pts[3].y=109;
    pts[4].x=271;  pts[4].y=129;
    pts[5].x=275;  pts[5].y=166;
    pts[6].x=282;  pts[6].y=181;
    pts[7].x=293;  pts[7].y=198;
    pts[8].x=312;  pts[8].y=205;
    pts[9].x=334;  pts[9].y=200;
    pts[10].x=353;  pts[10].y=190;
    pts[11].x=362;  pts[11].y=170;
    pts[12].x=367;  pts[12].y=151;
    pts[13].x=367;  pts[13].y=134;
    pts[14].x=369;  pts[14].y=117;
    pts[15].x=369;  pts[15].y=101;
    pts[16].x=366;  pts[16].y=83;
    pts[17].x=361;  pts[17].y=64;
    pts[18].x=352;  pts[18].y=50;
    pts[19].x=335;  pts[19].y=39;
    pts[20].x=317;  pts[20].y=38;
    pts[21].x=303;  pts[21].y=37;
    pts[22].x=289;  pts[22].y=44;
    pts[23].x=282;  pts[23].y=60;
    const int resolution = 1500;
	BSpline bs = BSpline(n, t , resolution, pts);
    IplImage *img1= cvLoadImage(argv[1], 1);
    CvMat imap= imageMap(img1, bs, resolution);
    // IplImage stub, *dst_img;   
    // dst_img = cvGetImage(&imap, &stub);
    // std::cout << dst_img->width << " " << dst_img->height<<  std::endl;    
    // cvNamedWindow("image map");
    // cvShowImage("image map", dst_img);
    // for (; ; ){
    //     cvWaitKey(0);        
    // }
    CvMat *mean_in = cvCreateMat(1, 3, CV_32FC1), *mean_out = cvCreateMat(1, 3, CV_32FC1);
    cvZero(mean_in);
    cvZero(mean_out);
    CvMemStorage* storage = cvCreateMemStorage(0);
    CvSeq *seq_in = cvCreateSeq( CV_32SC3 , sizeof(CvSeq), sizeof(int)*3 , storage);
    CvSeq *seq_out = cvCreateSeq( CV_32SC3 , sizeof(CvSeq), sizeof(int)*3, storage);
    CvMat *covar_in =  cvCreateMat(3, 3, CV_32FC1);
    CvMat *covar_out =  cvCreateMat(3, 3, CV_32FC1);
    computeMean(&imap, img1, mean_in, mean_out, seq_in, seq_out);
    CvMat **arr_in = new CvMat*[3], **arr_out = new CvMat*[3];

    arr_in[0] = cvCreateMat(1,seq_in->total, CV_8UC1);
    arr_in[1] = cvCreateMat(1,seq_in->total, CV_8UC1);
    arr_in[2] = cvCreateMat(1,seq_in->total, CV_8UC1);
    arr_out[0] = cvCreateMat(1,seq_in->total, CV_8UC1);
    arr_out[1] = cvCreateMat(1,seq_in->total, CV_8UC1);
    arr_out[2] = cvCreateMat(1,seq_in->total, CV_8UC1);
    
    // std::cout << inputMatrix_in->total << " "  << inputMatrix_out->total << std::endl;
    // computeCov(inputMatrix_in, inputMatrix_out, mean_in, mean_out, covar_in, covar_out);
   
    std::cout << mean_in->data.fl[0] << " " << mean_in->data.fl[1] << " " << mean_in->data.fl[2] << std::endl;
    std::cout << mean_out->data.fl[0] << " " << mean_out->data.fl[1] << " " << mean_out->data.fl[2] << std::endl;
    return 0;
}









