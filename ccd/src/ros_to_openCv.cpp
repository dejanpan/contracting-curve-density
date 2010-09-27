#include <ros/ros.h>
#include <ros/node_handle.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv2/legacy/legacy.hpp>
#include "bspline.h"
#include <opencv/highgui.h>
#include <string.h>

class ImageConverter {

public:
    ImageConverter(ros::NodeHandle &n, char **argv) :
        n_(n), it_(n_), resolution_(1000), contour_(new CvPoint[resolution_])
        {
            // manually draw contour using b-spline method
            // but currently we generate the curve using canny edge dectection
            // in future, we can use it
            int m ,t;
            m = 16;
            t = 3;
            resolution_ = 1000;
            // CvPoint* contour = new CvPoint[resolution_];
			CvPoint2D64f *pts = new CvPoint2D64f[m+1];  
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

            BSpline bs = BSpline(m, t,resolution_ ,pts);
            for (int i = 0; i < 1000; ++i){
                contour_[i] = bs[i];
            }
            cvNamedWindow("Image window");
            image_sub_ = it_.subscribe(argv[1], 1, &ImageConverter::imageCallback, this);
            n_.param ("save_image", save_image_, std::string(""));
        }

    ~ImageConverter()
        {
            delete []contour_;
            cvDestroyWindow("Image window");
        }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
        {

            IplImage *cv_image = NULL;
            try
                {
                    cv_image = bridge_.imgMsgToCv(msg_ptr, "bgr8");
                }
            catch (sensor_msgs::CvBridgeException error)
                {
                    ROS_ERROR("error");
                }
            IplImage* img1 = cvCreateImage(cvGetSize(cv_image),IPL_DEPTH_8U,1);

            // convert original image to gray-scale image
            cvCvtColor(cv_image,img1,CV_RGB2GRAY);

            // set the ROI
            cvSetImageROI(img1, cvRect(0, 0, 640, 360 ));
            IplImage* img2 = cvCreateImage(cvGetSize(img1),IPL_DEPTH_8U,1);

            // smooth-> canny edge dectection -> dilation -> erosion
            cvSmooth( img1, img1);
            cvCanny(img1, img2 ,0,80);
            cvDilate(img2, img2, NULL, 10);
            cvErode(img2, img2, NULL, 10);            

            CvMemStorage* storage = cvCreateMemStorage(0);
            CvSeq* contours = 0;
            // find contour, save the contour into as a CvSeq structure
            cvFindContours( img2, storage, &contours, sizeof(CvContour), CV_RETR_EXTERNAL, 2);

            // if contour's length is 0 or too small, skip it
            if(!contours) return ;
            int length = contours->total;
            std::cout<< "length: " << length << std::endl;
            if(length<10) return ; 

            // convert the contour into CvPoints
            CvPoint* point = new CvPoint[length];
            CvSeqReader reader;
            CvPoint pt= cvPoint(0,0);
    
            cvStartReadSeq(contours, &reader);
            for (int i = 0; i < length; i++)
                {
                    CV_READ_SEQ_ELEM(pt, reader);
                    point[i] = pt;
                }
            cvReleaseMemStorage(&storage);

            // draw the contour on the origin image
            for(int i=0;i<length;i++){
            	int j = (i+1)%length;
            	cvLine( cv_image, point[i], point[j], CV_RGB( 0, 0, 255 ), 1, 8, 0); 
            }

            // set up parameters for snake algorithms
            int ialpha = 20;
            int ibeta= 10;
            int igamma=20;
            float alpha=ialpha/100.0f; 
            float beta=ibeta/100.0f;
            float gamma=igamma/100.0f;
            CvSize size ={ 3, 3}; 
            CvTermCriteria criteria; 
            criteria.type = CV_TERMCRIT_ITER; 
            criteria.max_iter = 1000; 
            criteria.epsilon = 0.01;

            // call cvSnakeImage
            cvSnakeImage( img1, point, length, &alpha, &beta, &gamma,1,size,criteria,0 );            
            for(int i=0;i<length;i++)
                {
                    int j = (i+1)%length;
                    cvLine( cv_image, point[i],point[j],CV_RGB( 0, 255, 0 ),1,8,0 );
                }
            
            cvShowImage("Image window", cv_image);
            n_.getParam("save_image", save_image_);
            cvReleaseImage(&img1);
            cvReleaseImage(&img2);
            if(save_image_ != "")
                {
                    ROS_INFO("Saving image to %s", save_image_.c_str());
                    cvSaveImage(save_image_.c_str(), cv_image);
                }
            delete [] point;
            if(cvWaitKey(3) == 27) exit(-1);
        }
protected:
    ros::NodeHandle n_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    sensor_msgs::CvBridge bridge_;
    int resolution_;
    CvPoint *contour_;
    std::string save_image_;
};

int main(int argc, char** argv)
{
    if(argc < 1)
        {
            exit(-1);
            std::cout << "usage: ros_to_openCv topic" << std::endl;
        }

    ros::init(argc, argv, "ros_to_openCv");
    ros::NodeHandle n("~");
    ImageConverter ic(n, argv);
    ros::spin();
    return 0;
}
