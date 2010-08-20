#include "ccd.h"
#include <cv.h>
#include <highgui.h>
#include "bspline.h"
#include <iostream>

CvMat imageMap(IplImage *image, BSpline &curve_points, int n_points){
    CvMemStorage* storage = cvCreateMemStorage(0);
    CvSeq* contour = cvCreateSeq( CV_SEQ_ELTYPE_POINT , sizeof(CvSeq), sizeof(CvPoint2D32f) , storage );
    for (int i = 0; i < n_points; ++i){
        cvSeqPush(contour, &curve_points[i]);        
    }
    CvMat image_points = cvMat(image->width, image->height, CV_8UC1);
    // CvSeqReader reader;
    // CvPoint pt= cvPoint(0,0);;
    // cvStartReadSeq(contour, &reader);
    // for (int i = 0; i < n_points; i++)
    // {
    //     CV_READ_SEQ_ELEM(pt, reader);
    //     std::cout << pt.x << " " << pt.y << std::endl;
    // }

    CvPoint2D32f p;
    for (int i = 0; i < image->width; ++i){
        for (int j = 0; j < image->height; ++j){
            p.x = (float)i, p.y = (float)j;
            if(i == 0 && j == 0)
            std::cout << i << " " << j << ": "  <<  cvPointPolygonTest(contour, p, false) << std::endl;
            // cvmSet(&image_points,i,j,cvPointPolygonTest(contour, p, 0));
        }
    }
    cvReleaseMemStorage(&storage);
    return image_points;
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
    IplImage *img1= cvLoadImage(argv[1], 0);
    CvMat imap= imageMap(img1, bs, resolution);
    IplImage stub, *dst_img;
    dst_img = cvGetImage(&imap, &stub);
    cvNamedWindow("image map");
    cvShowImage("image map", dst_img);
    return 0;
}
