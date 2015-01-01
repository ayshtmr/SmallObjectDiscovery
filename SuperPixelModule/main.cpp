#include <iostream>
#include "super_pixel.h"
#include "cv.h"
#include "highgui.h"

using namespace cv;
using namespace std;


void binarize(Mat image /*Gray Image*/){
	for(int i=0;i<image.rows;i++){
		for(int j=0;j<image.cols;j++){
			if(image.at<uchar>(i,j)>200){
				image.at<uchar>(i,j)=255;
				}
			else{
				image.at<uchar>(i,j)=0;
			}
		}
	}
	imwrite("./skel.jpg",image);
	//cout<<image<<endl;
}

void fillpatch(Mat image,int x,int y){
	Rect* rect=0;
	floodFill(image, Point(x,y), Scalar(255,255,255),rect,Scalar(0,0,0),Scalar(0,0,0),8);
}











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
  ncmec::SuperPixel* sp = new  ncmec::SuperPixel(img);
  cvReleaseImage(&img);
  sp->SegmentNumber(K, m);
  CvScalar color = CV_RGB(0, 255, 0);
  std::string save_path = "./result.jpg";
  std::string save_path_skel = "./skel.jpg";
  sp->DrawContours(color, save_path,save_path_skel,2);
  Mat outseg=sp->labelledImage(2);
  Mat spimage = imread("./result.jpg",1);
  Mat skimage = imread("./skel.jpg",0);
  binarize(skimage);
  //namedWindow("Patch");
//  sp->SegmentSTEP(STEP, m);
  std::vector<ncmec::Segment> segs = sp->segments();
  int num_segs = sp->num_segments();
  for (int i = 0; i < num_segs; ++i) {
      cout << "------------- " << i << " -----------------" << endl;
      cout << "center.x: " << segs[i].center.x << endl;
      cout << "center.y: " << segs[i].center.y << endl;
      cout << "num of pixels: " << segs[i].pixel_num << endl;
      //fillpatch(skimage,(int)segs[i].center.x,(int)segs[i].center.y);
      //imshow("Patch",skimage);
      //waitKey(100);
      //circle(spimage,Point((int)segs[i].center.x,(int)segs[i].center.y), 1, Scalar(0,0,255));
  }
  
  const int* segmentation_map = sp->segmentation_map();
  int a = segmentation_map[0];
  int b = segmentation_map[50000];
  int c = segmentation_map[100000];  
  delete sp;
  return EXIT_SUCCESS;
}

