#include <iostream>
#include <vector>
#include <cv.h>
#include "bspline.h"
#include <highgui.h>



int main()
{
	int n,t;
	n= 8;          // number of control Points = n+1
	t= 3;
	point *pts = new point[n+1];
	pts[0].x=212;  pts[0].y=5;
	pts[1].x=72;   pts[1].y=43;  
	pts[2].x=105;  pts[2].y=158;
	pts[3].x=120;  pts[3].y=276;
	pts[4].x=214;  pts[4].y=308;
	pts[5].x=314;  pts[5].y=276;
	pts[6].x=339;  pts[6].y=174;
	pts[7].x=345;  pts[7].y=27;
	pts[8].x=212;  pts[8].y=5;
    // n = 5 ;
    // t = 3;
    // point *pts = new point[n+1];
    // pts[0].x = 0 ;   pts[0].y = 0;
    // pts[1].x = 1;    pts[1].y = 1;
    // pts[2].x = 2;    pts[2].y = -1;
    // pts[3].x = 3;    pts[3].y = 0;
    // pts[4].x = 4;    pts[4].y = -2;
    // pts[5].x = 5;    pts[5].y = 1;

	const int resolution = 1500;
	BSpline bs = BSpline(n, t , resolution, pts);
    for (int i = 0; i < resolution; ++i){
        std::cout << i << ": " <<  bs[i].x << " " << bs[i].y << std::endl;
    }
    IplImage* img = cvCreateImage(cvSize(491,370),IPL_DEPTH_32F,3);
    CvScalar color;
    int thickness = 0;
    for(int i=0; i< resolution-1; ++i)
	{
        color = CV_RGB( 0, 0, 255 );
        thickness = 1;
        // for (int j = 0 ; j <= n; ++j){
        //     if((out_pts[i].x == pts[j].x) && (out_pts[i].y == pts[j].y) ){
        //         color = CV_RGB( 0, 255, 0 );
        //         thickness = 2;
        //     }
        // }
//        std::cout << thickness << std::endl;
		cvLine( img, bs[i],bs[i+1], color , thickness, 8, 0 ); 
	}

    CvPoint *tmp_pts = new CvPoint[n+1];
    for (int j = 0 ; j <= n; ++j){
        color = CV_RGB( 0, 255, 0 );
        thickness = 2;
        tmp_pts[j].x = pts[j].x;
        tmp_pts[j].y = pts[j].y;
        cvLine( img, tmp_pts[j], tmp_pts[j], color , thickness, 8, 0 ); 
    }
    
    for(;;)
	{
		if(cvWaitKey(40)==27)break;
		cvShowImage("win1",img);
	}
    cvReleaseImage(&img);
	delete pts;
    delete tmp_pts;
	return 0;
}
