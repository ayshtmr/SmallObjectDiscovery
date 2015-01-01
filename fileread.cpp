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

using namespace std;
using namespace cv;



int main(int argc ,char *argv[]){

char filename[30]  = "ResultFile/745.txt";
_readfile rf;
///Read from file (x1,y1) and (x2,y2) of the line and filter the non-horizontal line
Mat fileData = rf.file2Mat(filename);
rf.filterL(fileData);

///Draw all the non-horizontal lines
Mat Image(Size(480,360),CV_8UC1,Scalar(0));
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
nr.checkvalidline(rf.X,rf.Xp,nr.p1Trans,nr.p2Trans);
Mat filteredLine(Size(480,360),CV_8UC1,Scalar(0));

rf.drawLine(filteredLine,nr.p1Filtered,nr.p2Filtered);

namedWindow("Outputnf");
namedWindow("Outputfl");
imshow("Outputnf",output);
imshow("Outputfl",filteredLine);

waitKey(0);
return 0;
}
