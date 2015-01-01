#include "cv.h"
#include "highgui.h"
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <cmath>

#define BUF 20
#define COLS 7
#define PI 3.141592
using namespace std;
using namespace cv;





class _readfile{
	public : vector<Point2f> X,Xp;
	public : Mat file2Mat(char *filename);
		     void filterL (Mat fileData);
		     void drawLine(Mat Image, vector<Point2f> p1 , vector<Point2f> p2);
		     
};


///Function for storing files to Mat
Mat _readfile :: file2Mat(char *filename){
ifstream infile;
infile.open(filename,ios::in);
	if (!infile) {
  	cerr << "Can't open input file "<< endl;
  	exit(1);
	}
	char number[BUF];
	int rows=0;
	int cols=0;
	while (!infile.eof()) {
 	infile>>number;
	cols++;
		if(cols==COLS){
		rows++;
		cols=0;
		}
	}			
	infile.close();
	
ifstream readfile;
readfile.open(filename,ios::in);
Mat filemat(rows,COLS,CV_64F,Scalar(0));	
rows = 0;
cols = 0;
char val[BUF];
	while (!readfile.eof()) {	
 	readfile>>val;
	filemat.at<double>(rows,cols)=atof(val);
	cols++;
		if(cols==COLS){
		rows++;
		cols=0;
		}
	}
return filemat;						 
}


///Method for filtering non-horizontal Lines
void _readfile :: filterL(Mat fileData){	
	for(int i = 0; i<fileData.rows; i++){
		double x1 = fileData.at<double>(i,0);
		double y1 = fileData.at<double>(i,1);
		double x2 = fileData.at<double>(i,2);
		double y2 = fileData.at<double>(i,3);
		//cout<<x1<<" "<<y1<<" "<<x2<<" "<<y2<<endl;
		double theta = (atan2((y2-y1),(x2-x1)))*180/PI;
		//cout<<abs(theta)<<endl; // First filtering
				if(abs(theta)>90 && abs(theta)<=89){ //if(abs(theta)>10 && abs(theta)<=170)
 					//cout<<"Yes"<<endl;
					X.push_back(Point(x1,y1));
					Xp.push_back(Point(x2,y2));
				}
		}
	//cout<<endl;
}


///Drawing all the non-horizontal Lines
void _readfile :: drawLine(Mat Image, vector<Point2f> p1 , vector<Point2f> p2){
	
	
	vector<Point2f> :: iterator itp1 = p1.begin();
	vector<Point2f> :: iterator itp2 = p2.begin(); 
	while(itp1!=p1.end()){
		line(Image,*itp1,*itp2,Scalar(255),1.5);
		itp1++;
		itp2++;
	}
	
}


void onesImage(Mat linepivot, vector<Point2f> p1 , vector<Point2f> p2){
	vector<Point2f> :: iterator itp1 = p1.begin();
	vector<Point2f> :: iterator itp2 = p2.begin(); 
	while(itp1!=p1.end()){
		linepivot.at<uchar>(itp1->y,itp1->x) = 255;
		linepivot.at<uchar>(itp2->y,itp2->x) = 255;
		itp1++;
		itp2++;
	}
	
}
