#include "ccd.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "bspline.h"
#include <iostream>
using namespace cv;
CvMat imageMap(IplImage *image,
               BSpline &curve_points,
               int n_points)
{
    CvMemStorage* storage = cvCreateMemStorage(0);
    CvSeq* contour = cvCreateSeq( CV_SEQ_ELTYPE_POINT , sizeof(CvSeq), sizeof(CvPoint2D32f) , storage );
    for (int i = 0; i < n_points; ++i)
        cvSeqPush(contour, &curve_points[i]);
    CvMat *image_points = cvCreateMat(image->height, image->width, CV_32FC1);
    cvZero(image_points);
    // std::cout << image->width << " " << image->height << std::endl;
    CvPoint2D32f p, tmp;
    for (int i = 0; i < image_points->rows; ++i)
        {
            for (int j = 0; j < image_points->cols; ++j)
                {
                    p.x = (float)j, p.y = (float)i;
                    // if(i == 60 && j == 282)
                    // std::cout << i << " " << j << ": "  <<  cvPointPolygonTest(contour, p, 0) << std::endl;
                    if(cvPointPolygonTest(contour, p, 0) == -100){
                        if(cvmGet(image_points, i, j ) == 0)
                            cvmSet(image_points,i,j, 10);
                    }
                    else if(cvPointPolygonTest(contour, p, 0) == 100){
                        if(cvmGet(image_points, i, j) == 0)
                            cvmSet(image_points,i,j, 60);
                    }
                    else{
                        cvmSet(image_points,i,j, 250);
                        if(((j > 1) && (j < (image_points->cols - 1))) && ((i > 1) && (i < (image_points->rows- 1)))){
                            for (int y = i-1; y <= i+1; ++y){
                                for (int x = j -1; x <= j+1; ++x){
                                    tmp.x = x; tmp.y = y;
                                    if((x != j) && (y != i)){
                                        double poly_test = cvPointPolygonTest(contour, tmp, 0);
                                        if(poly_test  == 100)
                                            cvmSet(image_points,y,x, 150);
                                        else if( poly_test == -100)
                                            cvmSet(image_points,y,x, 200);
                                    }
                                }
                            }
                        }
                    }
                }
        }
    cvReleaseMemStorage(&storage);
    return *image_points;
}


void computeMean(CvMat *image_map,
                 IplImage *image,
                 CvMat *mean_in,
                 CvMat *mean_out,
                 CvSeq* inputMatrix_in,
                 CvSeq* inputMatrix_out)
{
    Mat img(image);
    for (int i = 0 ; i < image->height; ++i){
        for (int j = 0; j < image->width; ++j)
            if(cvmGet(image_map, i,j) == 60){
                // std::cout << i << " " << j << " " <<  cvmGet(image_map, i,j) << " "<< std::endl;
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
    mean_in->data.db[0] /= inputMatrix_in->total;
    mean_in->data.db[1] /= inputMatrix_in->total;
    mean_in->data.db[2] /= inputMatrix_in->total;
    mean_out->data.db[0] /= inputMatrix_out->total;
    mean_out->data.db[1] /= inputMatrix_out->total;
    mean_out->data.db[2] /= inputMatrix_out->total;
}

void computeCov(CvMat **arr_in,
                CvMat **arr_out,
                CvMat *mean_in,
                CvMat *mean_out,
                CvMat *covar_in,
                CvMat *covar_out,
                int in_count,
                int out_count)
{
    cvCalcCovarMatrix((const void **)arr_in,in_count,covar_in,mean_in,CV_COVAR_NORMAL);
    cvCalcCovarMatrix((const void **)arr_out,out_count,covar_out,mean_out,CV_COVAR_NORMAL);
    for (int i = 0; i < 3; ++i){
        for (int j = 0; j < 3; ++j){
            covar_in->data.db[i*3+j] /=  in_count - 1;
            covar_out->data.db[i*3+j] /=  out_count - 1;
        }
    }
}
// please check the boundary

double Mahalanobis( const Mat& v1, const Mat& v2, const Mat& icovar )
{
    int type = v1.type(), depth = v1.depth();
    Size sz = v1.size();
    int i, j, len = sz.width*sz.height*v1.channels();
    AutoBuffer<uchar> buf(len*v1.elemSize());
    double result = 0;

    CV_Assert( type == v2.type() && type == icovar.type() &&
        sz == v2.size() && len == icovar.rows && len == icovar.cols );

    if( v1.isContinuous() && v2.isContinuous() )
    {
        sz.width *= sz.height;
        sz.height = 1;
    }

    if( depth == CV_32F )
    {
        const float* src1 = (const float*)v1.data;
        const float* src2 = (const float*)v2.data;
        size_t step1 = v1.step/sizeof(src1[0]);
        size_t step2 = v2.step/sizeof(src2[0]);
        float* diff = (float*)(uchar*)buf;
        const float* mat = (const float*)icovar.data;
        size_t matstep = icovar.step/sizeof(mat[0]);

        for( ; sz.height--; src1 += step1, src2 += step2, diff += sz.width )
        {
            for( i = 0; i < sz.width; i++ )
                diff[i] = src1[i] - src2[i];
        }

        diff = (float*)(uchar*)buf;
        for( i = 0; i < len; i++, mat += matstep )
        {
            double row_sum = 0;
            for( j = 0; j <= len - 4; j += 4 )
                row_sum += diff[j]*mat[j] + diff[j+1]*mat[j+1] +
                           diff[j+2]*mat[j+2] + diff[j+3]*mat[j+3];
            for( ; j < len; j++ )
                row_sum += diff[j]*mat[j];
            result += row_sum * diff[i];
        }
    }
    else if( depth == CV_64F )
    {
        const double* src1 = (const double*)v1.data;
        const double* src2 = (const double*)v2.data;
        size_t step1 = v1.step/sizeof(src1[0]);
        size_t step2 = v2.step/sizeof(src2[0]);
        double* diff = (double*)(uchar*)buf;
        const double* mat = (const double*)icovar.data;
        size_t matstep = icovar.step/sizeof(mat[0]);

        for( ; sz.height--; src1 += step1, src2 += step2, diff += sz.width )
        {
            for( i = 0; i < sz.width; i++ )
                diff[i] = src1[i] - src2[i];
        }

        diff = (double*)(uchar*)buf;
        for( i = 0; i < len; i++, mat += matstep )
        {
            double row_sum = 0;
            for( j = 0; j <= len - 4; j += 4 )
                row_sum += diff[j]*mat[j] + diff[j+1]*mat[j+1] +
                           diff[j+2]*mat[j+2] + diff[j+3]*mat[j+3];
            for( ; j < len; j++ )
                row_sum += diff[j]*mat[j];
            result += row_sum * diff[i];
        }
    }
    else
        CV_Error( CV_StsUnsupportedFormat, "" );

    return result;
}

void localStati(CvMat *image_map,
                IplImage *image,
                BSpline c,
                int length,
                CvMat *mean_in,
                CvMat *mean_out,
                CvMat *covar_in,
                CvMat *covar_out
    ){
    Mat img(image);
    int vout_count = 0, vin_count = 0;
    CvMat *covar_in_inv = cvCreateMat(3,3, CV_64FC1);
    CvMat *covar_out_inv = cvCreateMat(3,3, CV_64FC1);
    cvZero(covar_in_inv);
    cvZero(covar_out_inv);
    cvInvert(covar_in, covar_in_inv, CV_SVD);
    cvInvert(covar_out, covar_out_inv, CV_SVD);
    double vin_assign_t = .0, vout_assign_t = .0;
    double din_maha = .0, dout_maha = .0;
    CvMat *pixel = cvCreateMat(3,1, CV_64FC1);
    for (int i = 0; i < length; ++i){
        for (int x = c[i].x-1; x <= c[i].x + 1; ++x){
            for (int y = c[i].y-1; y <= c[i].y + 1; ++y){
                if(cvmGet(image_map, x, y) == 150){
                    vout_count++;
                    pixel->data.db[0] = img.at<Vec3b>(x,y)[0]; 
                    pixel->data.db[1] = img.at<Vec3b>(x,y)[1]; 
                    pixel->data.db[2] = img.at<Vec3b>(x,y)[2]; 
                    din_maha += cvMahalanobis(pixel, mean_in, covar_in_inv);
                }
                else{
                    vin_count++;
                    dout_maha += cvMahalanobis(pixel, mean_in, covar_in_inv);
                }
            }
        }
    }
    vout_assign_t = vout_count/(vout_count + vin_count);
    vin_assign_t = 1 - vout_assign_t;
}

void ccd(){
    bool converged = false;
    while (!converged){
        
    }
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
    // int n = 6;
    // int t = 3;
    // BSPoint *pts = new BSPoint[n+1];
    // pts[0].x=1;  pts[0].y=1;
    // pts[1].x=1;   pts[1].y=2;  
    // pts[2].x=1;  pts[2].y= 3;
    // pts[3].x=2;  pts[3].y=3;
    // pts[4].x=2;  pts[4].y=2;
    // pts[5].x=2;  pts[5].y=1;
    // pts[6].x=1;  pts[6].y=1;
    // const int resolution = 7;
	BSpline bs = BSpline(n, t , resolution, pts);
    IplImage *img1= cvLoadImage(argv[1], 1);
    CvMat imap= imageMap(img1, bs, resolution);
    // std::cout << "bs points " << std::endl;
    // for (int i = 0; i < resolution; ++i)
    //     std::cout << bs[i].x << " " << bs[i].y << std::endl;
    // std::cout << "bs points " << std::endl;
    IplImage stub, *dst_img;   
    dst_img = cvGetImage(&imap, &stub);
    // std::cout << dst_img->width << " " << dst_img->height<<  std::endl;
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
    std::cout << mean_in->data.db[0] << " " << mean_in->data.db[1] << " " << mean_in->data.db[2] << std::endl;
    std::cout << mean_out->data.db[0] << " " << mean_out->data.db[1] << " " << mean_out->data.db[2] << std::endl;
    CvMat **arr_in = new CvMat*[seq_in->total], **arr_out = new CvMat*[seq_out->total];
    CvSeqReader reader;
    cvStartReadSeq(seq_in, &reader);
    int tmp[3];
    for (int i = 0 ; i < seq_in->total; ++i){
        arr_in[i] = cvCreateMat(1,3, CV_64FC1);
        CV_READ_SEQ_ELEM(tmp,reader);
        // std::cout << tmp[0] << " " << tmp[1] << " " << tmp[2] << std::endl;           
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
    cvReleaseImage(&img1);
    return 0;
}
