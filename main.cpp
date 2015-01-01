#include "cv.h"
#include "highgui.h"
#include "detectobs.cpp"
#include <fstream>
#include <iostream>

#define RADIUS 40
#define FDIM 2
#define NO_OF_CLUSTER 5

using namespace std;
using namespace cv;


vector<Point2f> obspoint;
int no_of_filteredpoint=0;
//Mat samples(Size(IROWS,ICOLS),CV_32FC1);
//Mat label(Size(IROWS,ICOLS),CV_32FC1);
//Mat centers(Size(IROWS,ICOLS),CV_32FC1);



void filterPatch(Mat Patch , int x , int y, Mat ip){
    circle(Patch,Point(y,x),RADIUS,Scalar(255),-1);
    Mat masked = ip.mul(Patch/255);
	int count = countNonZero(masked);	
	if(count>2){
		obspoint.push_back(Point(y,x));
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


Mat drawPoint(char *imagefile,Mat &sample){
	Mat image = imread(imagefile,1);
	int count = 0;
	vector<Point2f> :: iterator itp =  obspoint.begin();
	while(itp!=obspoint.end()){
	circle(image,*itp,1,Scalar(0,255,0),2);
	sample.at<float>(count,0)=(itp->x);	
	sample.at<float>(count,1)=(itp->y);
	//sample.at<float>(count,2)=image.at<Vec3b>(itp->x,itp->y)[0];		
	//sample.at<float>(count,3)=image.at<Vec3b>(itp->x,itp->y)[1];		
	//sample.at<float>(count,4)=image.at<Vec3b>(itp->x,itp->y)[2];				
	count++;
	itp++;
	}
	return image;
}



int main(int argc, char *argv[]){

	for(int i=170;i<200;i++){
		stringstream ss;	
		ss<<i;
		string st = "./Diagonal_Run/Dataset_3_TXT/" + ss.str() + ".txt";
		string si = "./Diagonal_Run/Dataset_3/frame0" + ss.str() + ".jpg";
		
		

    		char *s = new char[st.length() + 1];
    		strcpy(s, st.c_str());

		char *simage = new char[si.length() + 1];
    		strcpy(simage, si.c_str());
		
		
///Constructor for detecting the segment which belongs to obstacle
		_obstacle obs(s);
		
		
		///now for each super pixel note the number of hotspot and classify	
		dofiltering(obs.hotSpot);
		///Adding the Matrix for clustering purposes.
		Mat sample(Size(FDIM,no_of_filteredpoint),CV_32FC1,Scalar(0));
		Mat center(Size(FDIM,no_of_filteredpoint),CV_32FC1,Scalar(0));
		Mat labels(Size(FDIM,no_of_filteredpoint),CV_32FC1,Scalar(0));
		
		
		///Drawing the filtered Point
		
		Mat filter = drawPoint(simage,sample);
		Mat drawingImage = imread(simage,1);
		//cout<<sample<<endl;
		namedWindow("Filter");
		imshow("Filter", obs.Output);
		
		
		kmeans(sample, NO_OF_CLUSTER, labels, 
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
		
		Mat no_of_samplec(NO_OF_CLUSTER,1,CV_64FC1,Scalar(0));
		returncount(labels,no_of_samplec);
		//cout<<no_of_samplec<<endl;
        
        for( int j = 0; j < no_of_filteredpoint; j++ )
        {
            int clusterIdx = labels.at<int>(j);
            Point2f ipt = obspoint[j];	
				circle(drawingImage, Point((int)ipt.x,(int)ipt.y), 2, colorTab[clusterIdx], 2);
				
        }
        
      
       
        namedWindow("Image");
        imshow("Image",drawingImage);
        waitKey(0);
        //cout<<pt<<endl;
		
		
		///deleting the vector after being used for an image
		obspoint.erase(obspoint.begin(),obspoint.end());
		
		no_of_filteredpoint = 0;
		
		//string ssave = "IrosResults/VirtualPlane/" + ss.str()+ ".jpg";
		//cout<<ssave<<endl;
		
		//imwrite(ssave,drawingImage);
		//waitKey(0);
		
	}
	
		
return 0;
}






















//_superpixel sp;
		//sp.superpixel(si,obs.hotSpot);
		//Mat spImage = imread("result.jpg",1);
		
		//namedWindow("Output");
		//namedWindow("Superpixel");
		//namedWindow("Segment");
		
		//imshow("Output",Segment);
		//imshow("Superpixel",spImage);
		//imshow("Segment",sp.segmentation);
		
//Mat Segment = obs.drawObstacle(simage);
		//Mat Hp = obs.drawHotSpot(simage,obs.hotSpot);
		//namedWindow("HotSpot");
		//imshow("HotSpot",Hp); ///hot pixels are available

//waitKey(100);
		//namedWindow("Patch");
//imshow("Patch",circlePatch);
				//waitKey(100);
