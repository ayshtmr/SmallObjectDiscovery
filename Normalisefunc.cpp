#include "cv.h"
#include "highgui.h"
#include "cxcore.h"

using namespace cv;
using namespace std;


vector <Vec3f> vector3d( vector<Point2f> corner){
	vector<Vec3f> corner_norm;
	std::vector<cv::Point2f> :: iterator it = corner.begin();
		while(it!=corner.end()){
			corner_norm.push_back(Vec3f(it->x, it->y, 1));
			++it;
		}
		return corner_norm;
}




class _normalise{
	private : vector<Vec3f>p1norm,p2norm;
	public  : vector<Point2f> p1Trans,p2Trans;
			  vector<Point2f> p1Filtered,p2Filtered;
			 void transform(vector<Point2f> p1 , vector<Point2f> p2,Mat H);
			 void checkvalidline(vector<Point2f> p1 , vector<Point2f> p2, vector<Point2f>p1T,vector<Point2f>p2T);
};


void _normalise :: transform(vector<Point2f> p1, vector<Point2f> p2, Mat H){
	
	vector<Vec3f> p1V3 = vector3d(p1);
	vector<Vec3f> p2V3 = vector3d(p2);
	
	vector< Vec3f > :: iterator itp1 = p1V3.begin();
	vector< Vec3f > :: iterator itp2 = p2V3.begin();
	
	Mat tempp1(3,1,CV_64FC1);
	Mat tempp2(3,1,CV_64FC1);
	
	for(; itp1!=p1V3.end(); itp1++){
		tempp1.at<double>(0,0) = (*itp1)[0];
		tempp1.at<double>(1,0) = (*itp1)[1];
		tempp1.at<double>(2,0) = (*itp1)[2];
		
		tempp2.at<double>(0,0) = (*itp2)[0];
		tempp2.at<double>(1,0) = (*itp2)[1];
		tempp2.at<double>(2,0) = (*itp2)[2];
		
		Mat X1 = H*tempp1;
		Mat X2 = H*tempp2;
		
		///Normalise
		
		X1.at<double>(0, 0) = X1.at<double>(0,0)/X1.at<double>(2,0);
		X1.at<double>(1, 0) = X1.at<double>(1,0)/X1.at<double>(2,0);
		p1Trans.push_back(Point(X1.at<double>(0, 0),X1.at<double>(1, 0)));
		
		X2.at<double>(0, 0) = X2.at<double>(0,0)/X2.at<double>(2,0);
		X2.at<double>(1, 0) = X2.at<double>(1,0)/X2.at<double>(2,0);
		p2Trans.push_back(Point(X2.at<double>(0, 0),X2.at<double>(1, 0)));
		
		itp2++;
	}
}

void _normalise :: checkvalidline(vector<Point2f> p1 , vector<Point2f> p2, vector<Point2f>p1T,vector<Point2f>p2T){
	
	vector< Point2f > :: iterator itp1 = p1.begin();
	vector< Point2f > :: iterator itp2 = p2.begin();
	
	vector< Point2f > :: iterator itp1T = p1T.begin();
	vector< Point2f > :: iterator itp2T = p2T.begin();
	
	while(itp1T!=p1T.end()){
		//cout<<(*itp1T).x<<" "<<(*itp1T).y<<endl;
		//cout<<(*itp2T).x<<" "<<(*itp2T).y<<endl;
		///<60 and greater than 105 to remove outliers low-lying object initial images;
		///<82 and greater than 95 to to remove small box in the later images
 		double theta = (atan2(((*itp2T).y - (*itp1T).y),((*itp2T).x - (*itp1T).x)))*180/PI;
			if(abs(theta)>90 && abs(theta)<90){
				/// for Dataset_1 (abs(theta)<120 && abs(theta)>10)
				/// for Dataset_2 (abs(theta)<120 && abs(theta)>70)
				/// for Dataset_3 (abs(theta)<120 && abs(theta)>30)
				p1Filtered.push_back(*itp1);
				p2Filtered.push_back(*itp2);
				///Adding few lines just for the corridor images due to noise				
				
			}
		itp1++;
		itp2++;
		itp1T++;
		itp2T++;
	}
}






