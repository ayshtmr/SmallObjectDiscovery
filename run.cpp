#include <iostream>
#include <string.h>
#include <stdio.h>
#include <cstdlib>
#include <fstream>



using namespace std;


int main(){

	system("./runlsd");
	system("./seg");

	// Mat img = imread("blob.jpg");
	// vector<vector<Point> > contours;
 //    vector<Vec4i> hierarchy;

 //    findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
 //    vector<Moments> mu(contours.size() );
 //    for( int i = 0; i < contours.size(); i++ )
 //    { mu[i] = moments( contours[i], false ); }
 //    vector<Point2f> mc( contours.size() );
 //    for( int i = 0; i < contours.size(); i++ )
 //    { mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }

 //    for(int k=0; k<contours.size();k++)
 //    {if(contourArea(contours[k])>500)
 //        {
 //            counter++;
 //        }
 //    }

 //    for(int l=0; l<contours.size();l++)
 //    {if(contourArea(contours[k])>500)
 //        {
 //            go[l]=mc[l];
 //        }
 //    }


	return 0;
}