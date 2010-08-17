#include <iostream>
#include <cv.h>
#include <string>
#include <highgui.h>
using namespace std;
void onChange(int);
IplImage *image = 0 ;
IplImage *image2 = 0 ;
int Thresholdness = 141;
int ialpha = 20;
int ibeta=20; 
int igamma=20;
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
 
	cvThreshold(image, image, Thresholdness, 255, CV_THRESH_BINARY);
 
	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq* contours = 0;
 
	cvFindContours( image, storage, &contours, sizeof(CvContour), CV_RETR_EXTERNAL , CV_CHAIN_APPROX_SIMPLE );
 
	if(!contours) return ; 
	const int length = contours->total;	
	if(length<10) return ; 
	CvPoint* point = new CvPoint[length];
	CvSeqReader reader;
	CvPoint pt= cvPoint(0,0);
	CvSeq *contour2=contours;
    
	cvStartReadSeq(contour2, &reader);
	for (int i = 0; i < length; i++)
	{
		CV_READ_SEQ_ELEM(pt, reader);
		point[i]=pt;
	}
	cvReleaseMemStorage(&storage);
	for(int i=0;i<length;i++)
	{
		int j = (i+1)%length;
        // cout << point[i].x << " " << point[i].y << endl;
		cvLine( image2, point[i], point[j], CV_RGB( 0, 0, 255 ), 1, 8, 0); 
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
	criteria.epsilon = 0.1; 
	cvSnakeImage( image, point,length, &alpha, &beta, &gamma,CV_VALUE,size,criteria,0 );
	for(int i=0;i<length;i++)
	{
		int j = (i+1)%length;
		cvLine( image2, point[i],point[j],CV_RGB( 0, 255, 0 ),1,8,0 ); 
	}
	delete []point;
}
