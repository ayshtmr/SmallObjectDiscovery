#include <iostream>
#include "super_pixel.h"

using namespace std;

int main(int argc, char** argv) {
  if (argc != 2)
    return EXIT_FAILURE;
  string image = argv[1];
  cout << "Input number of expected clusters..." << endl;
   int K;
   cin >> K;
//  cout << "Input expected STEP size..." << endl;
//  int STEP;
//  cin >> STEP;
  cout << "Input m factor..." << endl;
  float m;
  cin >> m;
  IplImage* img = cvLoadImage(image.c_str(), 1);
  SuperPixel* sp = new  SuperPixel(img);
  cvReleaseImage(&img);
  sp->SegmentK(K, m);
 //  sp->SegmentSTEP(STEP, m);
  vector<Segment> segs = sp->segments();
  int num_segs = sp->num_labels();
  for (int i = 0; i < num_segs; ++i) {
      cout << "------------- " << i << " -----------------" << endl;
      cout << "center.x: " << segs[i].center.x << endl;
      cout << "center.y: " << segs[i].center.y << endl;
      cout << "num of pixels: " << segs[i].pixel_num << endl;
  }
  CvScalar color = CV_RGB(255, 0, 0);
  IplImage* contour = sp->DrawContours(color);
  cvShowImage("Segmented", contour);
  cvWaitKey();
  cvReleaseImage(&contour);
  delete sp;
  return EXIT_SUCCESS;
}

