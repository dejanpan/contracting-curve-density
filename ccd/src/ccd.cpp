#include "ccd.h"
#include <cv.h>
void computeMean(IplImage *image, CvPoint *curve_points, int n_points){
    CvMemStorage* storage = cvCreateMemStorage(0);
    CvSeq* contour = cvCreateSeq( CV_SEQ_CONTOUR , sizeof(CvContour), sizeof(CvPoint) , storage );
    for (int i = 0; i < n_points; ++i){
        cvSeqPush(contour, &curve_points[i]);        
    }
    CvMat image_points = cvMat(image->width, image->height, CV_8UC1);
    CvPoint2D32f p;
    for (int i = 0; i < image->width; ++i){
        for (int j = 0; j < image->height; ++j){
            p.x = i, p.y = j;
            cvmSet(image_points,i,j,cvPointPolygonTest(contour, p, 0));
        }
    }
}

