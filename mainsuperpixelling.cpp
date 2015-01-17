#include "cv.h"
#include "highgui.h"
#include "detectobs.cpp"
#include <fstream>
#include <iostream>
#include "./SuperPixelModule/super_pixel.h"
#include "superpixel.cpp"
#include "./MarkovModule/markov.cpp"

#define RADIUS 40
#define FDIM 2
#define NO_OF_CLUSTER 5
#define NO_OF_SUPERP 50

using namespace std;
using namespace cv;


vector<Point2f> obspoint;
int no_of_filteredpoint=0;
//Mat samples(Size(IROWS,ICOLS),CV_32FC1);
//Mat label(Size(IROWS,ICOLS),CV_32FC1);
//Mat centers(Size(IROWS,ICOLS),CV_32FC1);

void res(Mat result){
	for(int i = 100; i<result.rows ; i++){
		for(int j= 0; j<result.cols ; j++){
			if(result.at<Vec3b>(i,j)[0]<150 && result.at<Vec3b>(i,j)[0]<150 && result.at<Vec3b>(i,j)[2]<200){
				//result.at<Vec3b>(i,j)[0] = 0;
				result.at<Vec3b>(i,j)[1] = 255;
				//result.at<Vec3b>(i,j)[2] = 0;
			}
			
		}
	}
}



void filterPatch(Mat Patch , int x , int y, Mat ip){
    circle(Patch,Point(y,x),RADIUS,Scalar(255),-1);
    Mat masked = ip.mul(Patch/255);
	int count = countNonZero(masked);	
	if(count>2){
		obspoint.push_back(Point(y,x));
		//hotSpot_superpixel.at<uchar>(x,y)=255;
		no_of_filteredpoint++;
	}
}


void returncount(Mat labels, Mat no_of_samplec){
	int counter = 0;
	for(int i=0; i< NO_OF_CLUSTER ; i++){
		for( int j = 0; j < no_of_filteredpoint; j++ )
        {
            int clusterIdx = labels.at<int>(j);
            if(clusterIdx == i){
				counter++;
			}
        }
        //cout<<counter<<endl;
        no_of_samplec.at<double>(i)=counter;
        counter = 0;
	}
}





void dofiltering(Mat hotSpot){
		for(int i =0 ; i< hotSpot.rows ; i++){
			for(int j = 0 ; j< hotSpot.cols; j++){
				if(hotSpot.at<uchar>(i,j)==255){
				Mat circlePatch(Size(640,480),CV_8UC1,Scalar(0));
				filterPatch(circlePatch,i,j,hotSpot);		
				}
			}
		}
}


Mat drawPoint(char *imagefile, Mat &sample , Mat &hotSpot_superpixel){
	Mat image = imread(imagefile,1);
	int count = 0;
	vector<Point2f> :: iterator itp =  obspoint.begin();
	while(itp!=obspoint.end()){
	circle(image,*itp,1,Scalar(0,255,0),2);
	sample.at<float>(count,0)=(itp->x);	
	sample.at<float>(count,1)=(itp->y);
	hotSpot_superpixel.at<uchar>((int)itp->y,(int)itp->x)=255;
	//sample.at<float>(count,2)=image.at<Vec3b>(itp->x,itp->y)[0];		
	//sample.at<float>(count,3)=image.at<Vec3b>(itp->x,itp->y)[1];		
	//sample.at<float>(count,4)=image.at<Vec3b>(itp->x,itp->y)[2];				
	count++;
	itp++;
	}
	return image;
}



int main(int argc, char *argv[]){
	
	

    for(int i=1;i<3;i++){
		stringstream ss;	
		ss<<i;
        string st = "./" + ss.str() + ".txt";
        string si = "./" + ss.str() + ".jpg";

    	char *s = new char[st.length() + 1];
    		strcpy(s, st.c_str());

		char *simage = new char[si.length() + 1];
    		strcpy(simage, si.c_str());
		
		Mat hotSpot_superpixel(Size(640,480),CV_8UC1,Scalar(0));
///Constructor for detecting the segment which belongs to obstacle

		_obstacle obs(s);
		_superpixel sp;
		
		
		
		
				 
		///now for each super pixel note the number of hotspot and classify	
		dofiltering(obs.hotSpot); // second filtering
		///Adding the Matrix for clustering purposes.
		Mat sample(Size(FDIM,no_of_filteredpoint),CV_32FC1,Scalar(0));
		Mat center(Size(FDIM,no_of_filteredpoint),CV_32FC1,Scalar(0));
		Mat labels(Size(FDIM,no_of_filteredpoint),CV_32FC1,Scalar(0));
		
		
		///Drawing the filtered Point
		
		Mat filter = drawPoint(simage,sample,hotSpot_superpixel);
		
		//Second argument in this method is not using that Mat
		
		//Mat drawingImage = imread(simage,1);
		//cout<<sample<<endl;
		
		//namedWindow("Filter");
		//imshow("Filter",filter);
		
		sp.superpixel(si,obspoint);
        Mat drawingImage = imread("result.jpg",1);
        Mat savesparse = imread(si,1);
		
		/*kmeans(sample, NO_OF_CLUSTER, labels, 
               TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 50, 1.0),
               20, KMEANS_RANDOM_CENTERS, center);
               
        //cout<<"Labels= "<<labels<<endl;
        //cout<<"Centers = "<<center<<endl;
        Scalar colorTab[] =
        {
        Scalar(0, 0, 255),
        Scalar(0,255,0),
        Scalar(255,100,100),
        Scalar(255,0,255),
        Scalar(0,128,255),
        Scalar(255,255,255),
        Scalar(100,128,255),
        Scalar(100,0,255),
        Scalar(100,128,0),
        Scalar(80,100,100),
        
        Scalar(200, 200, 100),
        Scalar(100,0,100),
        Scalar(0,100,100),
        Scalar(150,150,0),
        Scalar(0,0,100),
        Scalar(100,0,0),
        Scalar(150,150,150),
        Scalar(150,0,150),
        Scalar(100,200,200),
        Scalar(150,100,100)         
		};
		
		Mat no_of_samplec(NO_OF_CLUSTER,1,CV_64FC1,Scalar(0));*/
		
		//returncount(labels,no_of_samplec);
		//cout<<no_of_samplec<<endl;
        
        for( int j = 0; j < no_of_filteredpoint; j++ )
        {
            int clusterIdx = labels.at<int>(j);
            Point2f ipt = obspoint[j];	
				circle(drawingImage, Point((int)ipt.x,(int)ipt.y), 2, Scalar(0,255,0), 2);
				
				
        }
        
		
        
        //namedWindow("rs");
        //imshow("rs",~sp.segmentation); 
        
        
              

        
        ///Using Markov Random Field for making precise boundary
        //cout << "here" << endl;
		Mat imageGray = imread(si,0);
		Mat imageColor = imread(si,1);
		MarkovRandomField mrf;
		mrf.Mrf(imageGray, ~sp.segmentation, imageColor);
		//cout << "here" << endl;
		//namedWindow("Image");
        //imshow("Image",drawingImage);
        //res(imageColor);
        
        //namedWindow("rs");
		//imshow("rs",imageColor);
        //waitKey(5);
        
		///deleting the vector after being used for an image
		obspoint.erase(obspoint.begin(),obspoint.end());
		
		no_of_filteredpoint = 0;
		
		
		
        string ssave = "./" + ss.str()+ ".jpg";
		//cout<<ssave<<endl;
		imwrite(ssave,imageColor);
		
		waitKey(30);
		
		hotSpot_superpixel.release();
		sample.release();
		center.release();
		labels.release();
		filter.release();
		drawingImage.release();
	//	no_of_samplec.release();
			
			
			
		
	}
		
		
return 0;
}























