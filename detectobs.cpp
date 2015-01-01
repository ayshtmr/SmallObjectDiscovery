#include <iostream>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include "cv.h"
#include "highgui.h"
#include "file2Mat.cpp"
#include "warpImage.cpp"
#include "Normalisefunc.cpp"
#define COLS 7
#define DIMX 640
#define DIMY 480

using namespace std;
using namespace cv;



class _obstacle{
	
	public : _obstacle(char *filename);
			 ~_obstacle(){cout<<"deleting Obstacle Object: "<<endl;};	
			Mat drawObstacle(char *imagefile);
			Mat drawHotSpot(char *imagefile, Mat ip);
			Mat Output;
			Mat hotSpot;
			
};

_obstacle :: _obstacle(char *filename){
	
_readfile rf;

cout<<filename<<endl;
///Read from file (x1,y1) and (x2,y2) of the line and filter the non-horizontal line
Mat fileData = rf.file2Mat(filename);
rf.filterL(fileData); //First filtering

///Draw all the non-horizontal lines
Mat Image(Size(DIMX,DIMY),CV_8UC1,Scalar(0));
rf.drawLine(Image,rf.X,rf.Xp);


///Use this Image to calculate the homography between the top view and selected view.
Mat HMatrix(3,3,CV_64F,Scalar(0));
Mat colorImage = convert2RGB(Image);
Mat output=warp(colorImage,HMatrix); ///Homography is stored in HMatrix
//cout<<HMatrix<<endl;

///Now give this X, Xp and HMatrix and get the transformed X and Xp after wraping all the lines;
_normalise nr;
///Transforming the lines by multiplying each line by HMatrix
nr.transform(rf.X,rf.Xp,HMatrix);
///Removing all the lines that are not consistent with the HMatrix;
nr.checkvalidline(rf.X,rf.Xp,nr.p1Trans,nr.p2Trans); //Second filtering
Mat filteredLine(Size(DIMX,DIMY),CV_8UC1,Scalar(0));
Mat linepivot(Size(DIMX,DIMY),CV_8UC1,Scalar(0));

rf.drawLine(filteredLine,nr.p1Filtered,nr.p2Filtered);
onesImage(linepivot,nr.p1Filtered,nr.p2Filtered);

linepivot.copyTo(hotSpot);
filteredLine.copyTo(Output);

//return filteredLine;
}


Mat _obstacle :: drawObstacle(char *imagefile){
	Mat image = imread(imagefile,1);
	
	for(int i = 0; i<Output.rows; i++){
		for(int j = 0; j<Output.cols; j++){
			if(Output.at<uchar>(i,j)==255){
				image.at<Vec3b>(i,j)[0]=0;
				image.at<Vec3b>(i,j)[1]=0;
				image.at<Vec3b>(i,j)[2]=255;
			}
		}
	}
	return image;
}




Mat _obstacle :: drawHotSpot(char *imagefile, Mat ip){
	Mat image = imread(imagefile,1);
	
	
	for(int i = 0; i<ip.rows; i++){
		for(int j = 0; j<ip.cols; j++){
				if(ip.at<uchar>(i,j)==255){
				circle(image,Point(j,i),1,Scalar(0,0,255),2);
			}
		}
	}
	return image;
}
