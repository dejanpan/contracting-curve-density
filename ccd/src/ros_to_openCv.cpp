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
    ImageConverter(ros::NodeHandle &n) :
        n_(n), it_(n_), resolution_(1000), contour_(new CvPoint[resolution_])
        {
            // IplImage* img1 = cvCreateImage(cvSize(cv_image->width,cv_image->height),IPL_DEPTH_8U,3);
            // IplImage* img2 = cvCreateImage(cvSize(cv_image->width,cv_image->height),IPL_DEPTH_8U,1);
            // cvThreshold(cv_image, img1, Thresholdness, 255, CV_THRESH_BINARY);
            // cvConvertImage(img1, img2, 0);
            // CvMemStorage* storage = cvCreateMemStorage(0);
            // CvSeq* contours = 0;
 
            // cvFindContours( img2, storage, &contours, sizeof(CvContour), CV_RETR_EXTERNAL , CV_CHAIN_APPROX_SIMPLE );
 
            // if(!contours) return ; 
            // const int length = contours->total;	
            // if(length<10) return ; 
            // CvPoint* point = new CvPoint[length];
            // CvSeqReader reader;
            // CvPoint pt= cvPoint(0,0);
            // CvSeq *contour2=contours;

            int m ,t;
            m = 16;
            t = 3;
            resolution_ = 1000;
            // CvPoint* contour = new CvPoint[resolution_];
            BSPoint *pts = new BSPoint[m+1];
            
            // pts[0].x=282-86;  pts[0].y=60;
            // pts[1].x=278-86;   pts[1].y=75;  
            // pts[2].x=276-86;  pts[2].y= 91;
            // pts[3].x=272-86;  pts[3].y=109;
            // pts[4].x=271-86;  pts[4].y=129;
            // pts[5].x=275-86;  pts[5].y=166;
            // pts[6].x=282-86;  pts[6].y=181;
            // pts[7].x=293-86;  pts[7].y=198;
            // pts[8].x=312-86;  pts[8].y=205;
            // pts[9].x=334-86;  pts[9].y=200;
            // pts[10].x=353-86;  pts[10].y=190;
            // pts[11].x=362-86;  pts[11].y=170;
            // pts[12].x=367-86;  pts[12].y=151;
            // pts[13].x=367-86;  pts[13].y=134;
            // pts[14].x=369-86;  pts[14].y=117;
            // pts[15].x=369-86;  pts[15].y=101;
            // pts[16].x=366-86;  pts[16].y=83;
            // pts[17].x=361-86;  pts[17].y=64;
            // pts[18].x=352-86;  pts[18].y=50;
            // pts[19].x=335-86;  pts[19].y=39;
            // pts[20].x=317-86;  pts[20].y=38;
            // pts[21].x=303-86;  pts[21].y=37;
            // pts[22].x=289-86;  pts[22].y=44;
            // pts[23].x=282-86;  pts[23].y=60;
                        
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
            image_sub_ = it_.subscribe("/narrow_stereo/right/image_rect_color", 1, &ImageConverter::imageCallback, this);
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
            // int Thresholdness = 100;
            int ialpha = 0;
            int ibeta= 15;
            int igamma=2;

            // if (cv_image->width > 60 && cv_image->height > 60)
            //     cvCircle(cv_image, cvPoint(50,50), 10, cvScalar(255));

            // cvStartReadSeq(contour2, &reader);
            // for (int i = 0; i < resolution_; i++)
            //     {
            //         CV_READ_SEQ_ELEM(pt, reader);
            //         point[i]=pt;
            //     }
            // cvReleaseMemStorage(&storage);
            // for(int i=0;i<resolution_;i++)
            //     {
            //         int j = (i+1)%resolution_;
            //         // cout << point[i].x << " " << point[i].y << endl;
            //         cvLine( cv_image, point[i], point[j], CV_RGB( 0, 0, 255 ), 1, 8, 0); 
            //     }
            IplImage* img2 = cvCreateImage(cvSize(cv_image->width,cv_image->height),IPL_DEPTH_8U,1);
            // std::cout << cv_image->width  << " "<< cv_image->height << std::endl; // 
            cvConvertImage(cv_image, img2, 0);    
            float alpha=ialpha/100.0f; 
            float beta=ibeta/100.0f;
            float gamma=igamma/100.0f;
            CvSize size; 
            size.width=3; 
            size.height=3; 
            CvTermCriteria criteria; 
            criteria.type = CV_TERMCRIT_ITER; 
            criteria.max_iter = 1000; 
            criteria.epsilon = 0.01; 
            cvSnakeImage( img2, contour_, resolution_, &alpha, &beta, &gamma,1,size,criteria,0 );
            for(int i=0;i<resolution_;i++)
                {
                    int j = (i+1)%resolution_;
                    cvLine( cv_image, contour_[i],contour_[j],CV_RGB( 0, 255, 0 ),1,8,0 );
                }
            cvShowImage("Image window", cv_image);
            n_.getParam("save_image", save_image_);
            cvReleaseImage(&img2);
            if(save_image_ != "")
                {
                    ROS_INFO("Saving image to %s", save_image_.c_str());
                    cvSaveImage(save_image_.c_str(), cv_image);
                }
            // exit(-1);
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
    ros::init(argc, argv, "ros_to_openCv");
    ros::NodeHandle n("~");
    ImageConverter ic(n);
    ros::spin();
    return 0;
}
