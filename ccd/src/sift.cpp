#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <iostream>
#include <string>
#include <cstdio>
#include <algorithm>
#include <siftfast/siftfast.h>

using namespace cv;

inline cv::Scalar random_color(CvRNG* rng)
{
  int color = cvRandInt(rng);
  return CV_RGB(color&255, (color>>8)&255, (color>>16)&255);
}

/* Return squared distance between two keypoint descriptors.
*/
int DistSquared(Keypoint k1, Keypoint k2)
{
    int i, dif, distsq = 0;
    float *pk1, *pk2;

    pk1 = k1->descrip;
    pk2 = k2->descrip;

    for (i = 0; i < 128; i++) {
      dif = (int) *pk1++ - (int) *pk2++;
      distsq += dif * dif;
    }
    return distsq;
}


/* This searches through the keypoints in klist for the two closest
   matches to key.  If the closest is less than 0.6 times distance to
   second closest, then return the closest match.  Otherwise, return
   NULL.
*/
Keypoint CheckForMatch(Keypoint key, Keypoint klist)
{
    int dsq, distsq1 = 100000000, distsq2 = 100000000;
    Keypoint k, minkey = NULL;

    /* Find the two closest matches, and put their squared distances in
       distsq1 and distsq2.
    */
    for (k = klist; k != NULL; k = k->next) {
      dsq = DistSquared(key, k);

      if (dsq < distsq1) {
	distsq2 = distsq1;
	distsq1 = dsq;
	minkey = k;
      } else if (dsq < distsq2) {
	distsq2 = dsq;
      }
    }

    /* Check whether closest distance is less than 0.6 of second. */
    if (10 * 10 * distsq1 < 6 * 6 * distsq2)
      return minkey;
    else return NULL;
}


vector<Keypoint> FindMatches(Keypoint keys1, Keypoint keys2)
{
    Keypoint k, match;
    int count = 0;
    std::vector<Keypoint> match_vector;
    /* Match the keys in list keys1 to their best matches in keys2.
    */
    for (k= keys1; k != NULL; k = k->next)
    {
      match = CheckForMatch(k, keys2);  

      /* Draw a line on the image from keys1 to match.  Note that we
	 must add row count of first image to row position in second so
	 that line ends at correct location in second image.
      */
      if (match != NULL) {
        count++;
        match_vector.push_back(match);
      }
    }

    /* Write result image to standard output. */
    fprintf(stderr,"Found %d matches.\n", count);
    return match_vector;
}


Keypoint extract_keypoints(cv::Mat image)
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
  keypoints = GetKeypoints(sift_image);
  DestroyAllImages();
  return keypoints;
}

int main (int argc, char * argv[]) 
{
  cv::Mat img1 = cv::imread("book.pgm");
  cv::Mat img2 = cv::imread("book_clutter.pgm");
  int show_rows = max(img1.rows, img2.rows);
  int show_cols = img1.cols + img2.cols;
  cv::Mat img_show = Mat::zeros(show_rows, show_cols, CV_8UC3);
  // std::cout << "cols:" << show_cols << " rows: " << show_rows << std::endl;
  for (int i = 0; i < img1.rows; ++i)
  {
    for (int j = 0; j < img1.cols; ++j)
    {
      img_show.at<Vec3b>(i,j) = img1.at<Vec3b>(i,j);
    }
  }
  for (int i = 0; i < img2.rows; ++i)
  {
    for (int j = img1.cols; j < img_show.cols ; ++j)
    {
      img_show.at<Vec3b>(i,j) = img2.at<Vec3b>(i,j-img1.cols);
    }
    std::cout << std::endl;
  }
  // std::cout << "hello" << std::endl;


  // cv::Mat canvas = imread(argv[1], 1);
  cv::namedWindow("convas", 1);
  Keypoint keypoints1 = extract_keypoints(img1);
  Keypoint keypoints2 = extract_keypoints(img2);
  
  Keypoint p = keypoints1;
  int keypoints1_count = 0;
  int keypoints2_count = 0;
  //declare vector<T*> vt;
  CvRNG rng;
  cv::Scalar color = random_color(&rng);

  while (p != NULL) 
  {
    cv::circle(img_show, cv::Point2f(p->col,p->row), 2,color, 2);
    std::cout <<  p->row << " " << p->col  << std::endl;
    p = p->next;
    ++keypoints1_count;
  }

  p = keypoints2;

  while (p != NULL) 
  {
    cv::circle(img_show, cv::Point2f(p->col+img1.cols,p->row), 2,color, 2);
    std::cout <<  p->row << " " << p->col  << std::endl;
    p = p->next;
    ++keypoints2_count;
  }

  cv::imshow("convas", img_show);
  char key;
  while (1)
  {
    key = cv::waitKey(10);
    if (key == 27) break;
  }
  return 0;
}
