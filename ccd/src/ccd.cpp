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
    // std::cout << image->width << " " << image->height << std::endl;
    CvPoint2D32f p;
    for (int i = 0; i < image_points->rows; ++i){
        for (int j = 0; j < image_points->cols; ++j){
            p.x = (float)j, p.y = (float)i;
            // if(i == 60 && j == 282)
            std::cout << i << " " << j << ": "  <<  cvPointPolygonTest(contour, p, 0) << std::endl;
            if(cvPointPolygonTest(contour, p, 0) == -100)
                cvmSet(image_points,i,j, 60.0);
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
            if(cvmGet(image_map, i,j) == 60){
                std::cout << i << " " << j << " " << std::endl;
                mean_in->data.db[0] += img.at<Vec3b>(i,j)[0];
                mean_in->data.db[1] += img.at<Vec3b>(i,j)[1];
                mean_in->data.db[2] += img.at<Vec3b>(i,j)[2];
                int data[3] = {img.at<Vec3b>(i,j)[0], img.at<Vec3b>(i,j)[1], img.at<Vec3b>(i,j)[2]};
                cvSeqPush(inputMatrix_in, data);
            }
            else{
                mean_out->data.db[0] += img.at<Vec3b>(i,j)[0];
                mean_out->data.db[1] += img.at<Vec3b>(i,j)[1];
                mean_out->data.db[2] += img.at<Vec3b>(i,j)[2];
                int data[3] = {img.at<Vec3b>(i,j)[0], img.at<Vec3b>(i,j)[1], img.at<Vec3b>(i,j)[2]};
                cvSeqPush(inputMatrix_out, data);
            }
        }
}

void computeCov(CvMat **arr_in, CvMat **arr_out, CvMat *mean_in, CvMat *mean_out, CvMat *covar_in, CvMat *covar_out, int in_count, int out_count)
{
    cvCalcCovarMatrix((const void **)arr_in,in_count,covar_in,mean_in,CV_COVAR_NORMAL);
    cvCalcCovarMatrix((const void **)arr_out,out_count,covar_out,mean_out,CV_COVAR_NORMAL);
}

int main (int argc, char * argv[]) 
{
    // int n = 23;
    // int t =3;
    // BSPoint *pts = new BSPoint[n+1];
    // pts[0].x=282;  pts[0].y=60;
    // pts[1].x=278;   pts[1].y=75;  
    // pts[2].x=276;  pts[2].y= 91;
    // pts[3].x=272;  pts[3].y=109;
    // pts[4].x=271;  pts[4].y=129;
    // pts[5].x=275;  pts[5].y=166;
    // pts[6].x=282;  pts[6].y=181;
    // pts[7].x=293;  pts[7].y=198;
    // pts[8].x=312;  pts[8].y=205;
    // pts[9].x=334;  pts[9].y=200;
    // pts[10].x=353;  pts[10].y=190;
    // pts[11].x=362;  pts[11].y=170;
    // pts[12].x=367;  pts[12].y=151;
    // pts[13].x=367;  pts[13].y=134;
    // pts[14].x=369;  pts[14].y=117;
    // pts[15].x=369;  pts[15].y=101;
    // pts[16].x=366;  pts[16].y=83;
    // pts[17].x=361;  pts[17].y=64;
    // pts[18].x=352;  pts[18].y=50;
    // pts[19].x=335;  pts[19].y=39;
    // pts[20].x=317;  pts[20].y=38;
    // pts[21].x=303;  pts[21].y=37;
    // pts[22].x=289;  pts[22].y=44;
    // pts[23].x=282;  pts[23].y=60;
    // const int resolution = 1500;
    int n = 6;
    int t = 3;
    BSPoint *pts = new BSPoint[n+1];
    pts[0].x=1;  pts[0].y=1;
    pts[1].x=2;   pts[1].y=1;  
    pts[2].x=3;  pts[2].y= 1;
    pts[3].x=3;  pts[3].y=2;
    pts[4].x=2;  pts[4].y=2;
    pts[5].x=1;  pts[5].y=2;
    pts[6].x=1;  pts[6].y=1;
    const int resolution = 7;
	BSpline bs = BSpline(n, t , resolution, pts);
    IplImage *img1= cvLoadImage(argv[1], 1);
    CvMat imap= imageMap(img1, bs, resolution);
    std::cout << "bs points " << std::endl;
    for (int i = 0; i < resolution; ++i){
        std::cout << bs[i].x << " " << bs[i].y << std::endl;
    }
    std::cout << "bs points " << std::endl;
    IplImage stub, *dst_img;   
    dst_img = cvGetImage(&imap, &stub);
    std::cout << dst_img->width << " " << dst_img->height<<  std::endl;
    cvSaveImage("bspline.png", dst_img);
    // cvNamedWindow("image map");
    // cvShowImage("image map", dst_img);
    // for (; ; ){
    //     cvWaitKey(0);        
    // }
    CvMat *mean_in = cvCreateMat(3, 1, CV_64FC1), *mean_out = cvCreateMat(3,1, CV_64FC1);
    cvZero(mean_in);
    cvZero(mean_out);
    CvMemStorage* storage = cvCreateMemStorage(0);
    CvSeq *seq_in = cvCreateSeq( CV_32SC3 , sizeof(CvSeq), sizeof(int)*3 , storage);
    CvSeq *seq_out = cvCreateSeq( CV_32SC3 , sizeof(CvSeq), sizeof(int)*3, storage);
    CvMat *covar_in =  cvCreateMat(3, 3, CV_64FC1);
    CvMat *covar_out =  cvCreateMat(3, 3, CV_64FC1);
    computeMean(&imap, img1, mean_in, mean_out, seq_in, seq_out);
    CvMat **arr_in = new CvMat*[seq_in->total], **arr_out = new CvMat*[seq_out->total];
    CvSeqReader reader;
    cvStartReadSeq(seq_in, &reader);
    int tmp[3];
    for (int i = 0 ; i < seq_in->total; ++i){
           arr_in[i] = cvCreateMat(1,3, CV_64FC1);
           CV_READ_SEQ_ELEM(tmp,reader);
           cvmSet(arr_in[i], 0, 0, tmp[0]);
           cvmSet(arr_in[i], 0, 1, tmp[1]);
           cvmSet(arr_in[i], 0, 2, tmp[2]);
    }
    cvStartReadSeq(seq_out, &reader);
    for (int i= 0; i < seq_out->total; ++i){
        arr_out[i] = cvCreateMat(1,3, CV_64FC1);
        CV_READ_SEQ_ELEM(tmp,reader);
        cvmSet(arr_out[i], 0, 0, (double)tmp[0]);
        cvmSet(arr_out[i], 0, 1, (double)tmp[1]);
        cvmSet(arr_out[i], 0, 2, (double)tmp[2]);
    }
    // // std::cout << inputMatrix_in->total << " "  << inputMatrix_out->total << std::endl;
    computeCov(arr_in, arr_out, mean_in, mean_out, covar_in, covar_out, seq_in->total, seq_out->total);
    for (int i = 0; i < 3; ++i){
        for (int j = 0; j < 3; ++j){
            std::cout << cvmGet(covar_in, i,j) << " ";
        }
        std::cout << std::endl;
    }
    std::cout << mean_in->data.db[0] << " " << mean_in->data.db[1] << " " << mean_in->data.db[2] << std::endl;
    std::cout << mean_out->data.db[0] << " " << mean_out->data.db[1] << " " << mean_out->data.db[2] << std::endl;
    cvReleaseImage(&img1);
    return 0;
}
