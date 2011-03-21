#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <iostream>
#include <string>
#include <cstdio>
#include <algorithm>
#include <siftfast/siftfast.h>

using namespace cv;
Keypoint extract_keypoints(cv::Mat image, bool frames_only)
{
  Image sift_image = CreateImage(image.rows, image.cols);
  for (int i = 0; i < image.rows; ++i) 
  {
    uint8_t* pSrc = image.ptr<uint8_t>(i);;
    float* pDst = sift_image->pixels + i * sift_image->stride;
    for (int j = 0; j < image.cols; ++j)
      pDst[j] = (float) pSrc[j] * (1.0f / 255.0f);
  }
  
  Keypoint keypoints;
  if (frames_only)
    keypoints = GetKeypointFrames(sift_image);
  else
    keypoints = GetKeypoints(sift_image);
  DestroyAllImages();
  return keypoints;
}

int main (int argc, char * argv[]) 
{
  cv::Mat camera_image_in = cv::imread("")
  Keypoint keypoints = extract_keypoints(camera_image_in);
  Keypoint p = keypoints;
  int camera_keypoints_count = 0;
  //declare vector<T*> vt;
	
  //push keypoints in the vocabulary tree document
  while (p != NULL) 
  {
    p = p->next;
    ++camera_keypoints_count;
  }  
  return 0;
}
