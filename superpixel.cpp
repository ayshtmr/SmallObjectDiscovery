#include "cv.h"
#include "highgui.h"
#include "./SuperPixelModule/super_pixel.h"
#include <iostream>


using namespace cv;
using namespace std;


class _superpixel{
			
	public : Mat segmentation;
	public : vector<int>idHp;
	public : void superpixel(string image, vector<Point2f> InterestPoint);
			~_superpixel(){cout<<"Deleting superpixel object "<<endl;};
	
};



void _superpixel :: superpixel(string image,vector<Point2f> InterestPoint){
	int K = 2000; ///Maximum number of SuperPixel
	float m = 25.0;	
	
	IplImage* img = cvLoadImage(image.c_str(), 1);
	
	IplImage *destination = cvCreateImage(cvSize(img->width, img->height),img->depth, img->nChannels);
	
	Mat ground(Size(img->width, img->height),CV_8UC1,Scalar(0));
	
	//segmentation(Size(img->width, img->height),CV_8UC1,Scalar(0));
	ground.copyTo(segmentation);

	
	cvResize(img, destination);
	
	
    ncmec::SuperPixel* sp = new  ncmec::SuperPixel(destination);
    
    
    Mat labelArray(Size(img->width,img->height),CV_64FC1);
    cvReleaseImage(&img);
    
    sp->SegmentNumber(K, m);
    
    CvScalar color = CV_RGB(0, 255, 0);
    
    std::string save_path = "./result.jpg";
    std::string save_path_skel = "./skel.jpg";
    int maxno_ofcluster = 0;
    
    
	sp->DrawContours(color, save_path,save_path_skel, maxno_ofcluster , labelArray);
	
	
	vector<Point2f> :: iterator itIp = InterestPoint.begin();
		while(itIp!= InterestPoint.end()){
			int l = (int)labelArray.at<double>(itIp->y,itIp->x);
			Mat Patch=sp->labelledImage(l);
			ground = ground + Patch;
			itIp++;
			Patch.release();
		}
		
	ground.copyTo(segmentation);
	ground.release();
}



