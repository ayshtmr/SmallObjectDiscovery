#include "cv.h"
#include "highgui.h"
#include "cxcore.h"
#include <string.h>
#include <cstdlib>
#include <stdio.h>
#include <fstream>
#include <iostream>


using namespace cv;
using namespace std;

vector<Point2f> window1_point;
vector<Point2f> window2_point;

Mat image1_global;
int count1=0;



void Mousehandlertwo(int event, int x, int y,int flags, void * param){
		string *s = (string *)param;
		if(event == CV_EVENT_LBUTTONDOWN){
		if(*s=="Image1"){
			std::stringstream ss1;
			count1++;
			ss1<<count1;
			string buffer1=ss1.str();
			window1_point.push_back(Point(x,y));
			cout<<"(x, y) "<<x<<" "<<y<<endl;
			circle(image1_global,Point(x,y),1,Scalar(0,255,0),2);
			putText(image1_global,buffer1,Point(x,y),FONT_HERSHEY_PLAIN,1.0,Scalar(0,255,0),1);
		}
	}
}


Mat panoroma_new(Mat image1,Mat H_Matrix){
	Mat output;
	cv::warpPerspective(image1,output,H_Matrix,cv::Size(2*image1.cols,image1.rows));
	return(output);
}

Mat convert2RGB(Mat image){
	Mat color(image.size(),CV_8UC3,Scalar(0));
		for(int i = 0; i < image.rows; i++ ){
			for(int j = 0; j< image.cols; j++ ){
				if(image.at<uchar>(i,j)==255){
					color.at<Vec3b>(i,j)[0]=255;
					color.at<Vec3b>(i,j)[1]=255;
					color.at<Vec3b>(i,j)[2]=255;
				}
			}
		}
		return color;
}




Mat warp(Mat image1,Mat HMatrix){
	image1.copyTo(image1_global);
	
	//string msg1 = "Image1";
	
	window2_point.push_back(Point(image1_global.rows+0,0));
	window2_point.push_back(Point(image1_global.rows+image1_global.rows,0));
	window2_point.push_back(Point(image1_global.rows+image1_global.rows,image1_global.cols));
	window2_point.push_back(Point(image1_global.rows+0,image1_global.cols));
	
	
	window1_point.push_back(Point(200,150));
	window1_point.push_back(Point(250,150));
	window1_point.push_back(Point(400,460));
	window1_point.push_back(Point(3,460));
	
	
	
	
	
		/*for(;;){
		namedWindow("Image1");
		
		cvMoveWindow("Image1",100,100);
		
		setMouseCallback("Image1",Mousehandlertwo,&msg1);

		imshow("Image1",image1_global);
		
		char cnew = (char)waitKey(10);
					if(cnew==27){
					break;
					}
		}*/
		
Mat H_RANSAC = findHomography(window1_point,window2_point,CV_RANSAC,3);
H_RANSAC.copyTo(HMatrix);
Mat output = panoroma_new(image1_global,H_RANSAC);
//cout<<H_RANSAC<<endl;
return output;
}
