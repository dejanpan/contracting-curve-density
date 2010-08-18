#include <iostream>
#include <opencv/cv.h>
#include <string>
#include <opencv/highgui.h>
#include <opencv2/legacy/legacy.hpp>
#include "bspline.h"

//#include <opencv/cvexts.h>

using namespace std;
void onChange(int);
IplImage *image = 0 ;
IplImage *image2 = 0 ;
int Thresholdness = 141;
int ialpha = 0;
int ibeta=15; 
int igamma=2;
static string input_file = "grey1.bmp";
int main(int argc, char* argv[])
{
    if(argc >= 2)
        input_file = argv[1];
	cvNamedWindow("snake",0);
	cvCreateTrackbar("threshold", "snake", &Thresholdness, 255, onChange);
	cvCreateTrackbar("alpha", "snake", &ialpha, 100, onChange);
	cvCreateTrackbar("beta", "snake", &ibeta, 100, onChange);
	cvCreateTrackbar("gamma", "snake", &igamma, 100, onChange);
	cvResizeWindow("snake",600,400);
	onChange(0);
	for(;;)
	{
		if(cvWaitKey(40) == 27) break;
		cvShowImage("snake",image2);
	}
	return 0;
}

void onChange(int pos)
{
	if(image2) cvReleaseImage(&image2);
	if(image) cvReleaseImage(&image);
 
	image2 = cvLoadImage(input_file.c_str(),1);
	image= cvLoadImage(input_file.c_str(),0);
 
	// cvThreshold(image, image, Thresholdness, 255, CV_THRESH_BINARY);
 
	// CvMemStorage* storage = cvCreateMemStorage(0);
	// CvSeq* contours = 0;
 
	// cvFindContours( image, storage, &contours, sizeof(CvContour), CV_RETR_EXTERNAL , CV_CHAIN_APPROX_SIMPLE );
 
	// if(!contours) return ; 
	// const int length = contours->total;	
	// if(length<10) return ; 
	// CvPoint* point = new CvPoint[length];
	// CvSeqReader reader;
	// CvPoint pt= cvPoint(0,0);
	// CvSeq *contour2=contours;
    
	// cvStartReadSeq(contour2, &reader);
	// for (int i = 0; i < length; i++)
	// {
	// 	CV_READ_SEQ_ELEM(pt, reader);
	// 	point[i]=pt;
	// }
	// cvReleaseMemStorage(&storage);
	// for(int i=0;i<length;i++)
	// {
	// 	int j = (i+1)%length;
    //     // cout << point[i].x << " " << point[i].y << endl;
	// 	cvLine( image2, point[i], point[j], CV_RGB( 0, 0, 255 ), 1, 8, 0); 
	// }

    int m ,t, length;
            m = 16;
            t = 3;
            length = 1000;
            CvPoint* point = new CvPoint[length];
            BSPoint *pts = new BSPoint[m+1];
            
            pts[0].x=204;  pts[0].y=63;
            pts[1].x=201;   pts[1].y=90;  
            pts[2].x=200;  pts[2].y= 117;
            pts[3].x=202;  pts[3].y=146;
            pts[4].x=210;  pts[4].y=173;
            pts[5].x=220;  pts[5].y=189;
            pts[6].x=242;  pts[6].y=193;
            pts[7].x=263;  pts[7].y=190;
            pts[8].x=273;  pts[8].y=169;
            pts[9].x=278;  pts[9].y=145;
            pts[10].x=275;  pts[10].y=120;
            pts[11].x=272;  pts[11].y=96;
            pts[12].x=265;  pts[12].y=67;
            pts[13].x=253;  pts[13].y=55;
            pts[14].x=238;  pts[14].y=50;
            pts[15].x=222;  pts[15].y=54;
            pts[16].x=204;  pts[16].y=63;
            // pts[17].x=361-86;  pts[17].y=64;
            // pts[18].x=352-86;  pts[18].y=50;
            // pts[19].x=335-86;  pts[19].y=39;
            // pts[20].x=317-86;  pts[20].y=38;
            // pts[21].x=303-86;  pts[21].y=37;
            // pts[22].x=289-86;  pts[22].y=44;
            // pts[23].x=282-86;  pts[23].y=60;
            BSpline bs = BSpline(m, t,length,pts);
            for (int i = 0; i < 1000; ++i){
                point[i] = bs[i];
            }

	float alpha=ialpha/100.0f; 
	float beta=ibeta/100.0f;
	float gamma=igamma/100.0f;
	CvSize size; 
	size.width=3; 
	size.height=3; 
	CvTermCriteria criteria; 
	criteria.type = CV_TERMCRIT_ITER; 
	criteria.max_iter = 1000; 
	criteria.epsilon = 0.001; 
	cvSnakeImage( image, point,length, &alpha, &beta, &gamma,1,size,criteria,0 );
	for(int i=0;i<length;i++)
	{
		int j = (i+1)%length;
		cvLine( image2, point[i],point[j],CV_RGB( 0, 255, 0 ),1,8,0 ); 
	}
	delete []point;
}
