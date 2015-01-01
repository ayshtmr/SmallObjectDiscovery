#include "cv.h"
#include "highgui.h"
#include "cxcore.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <cmath>
#include "graph.h"


#define H 300
#define L 0
#define KC 60
#define ROW 480
#define COL 640


using namespace std;
using namespace cv;

///refining the result using basic image processing code
void clusterSeg(Mat BImage){
	for(int i=0 ;i<50; i++){
		for(int j=0 ; j<80;j++){
			if(BImage.at<uchar>(i,j)=255){
				BImage.at<uchar>(i,j)=0;
			}
		}
	}
	
	//return contourImage;
}





Mat convert3channel(Mat imageB){
	Mat imageC(imageB.size(),CV_8UC3,Scalar(0,0,0));
	for(int i = 280; i<imageB.rows; i++){
		for(int j = 0; j<imageB.cols; j++){
			if((int)imageB.at<uchar>(i,j)==255){
				imageC.at<Vec3b>(i,j)[0] = 255;
				imageC.at<Vec3b>(i,j)[1] = 255;
				imageC.at<Vec3b>(i,j)[2] = 255;
			}
		}
	}
	return imageC;
}


class MarkovRandomField{
	public : Mat MrfOutput;
	         void Mrf(Mat image, Mat mask, Mat colorImage);

	private :vector<Vec3i> smooth;
         	 vector<Vec2i> nodeW; 
	         void unary(Mat image, Mat imagecolor);
	         void smoothness(Mat image, Mat image_index, Mat imagecolor);
	         void correct(Mat binary,Mat image);
};



void MarkovRandomField :: unary(Mat image,Mat imagecolor){
		for(int y = 0; y < image.rows; y++){
			for(int x = 0; x< image.cols; x++){
				int imageVal = image.at<bool>(y,x);
				
				int imageValb = (int)imagecolor.at<Vec3b>(y,x)[0];
				int imageValg = (int)imagecolor.at<Vec3b>(y,x)[1];
				int imageValr = (int)imagecolor.at<Vec3b>(y,x)[2]; /**if color cut is needed**/ 
				
				//int sourceVal = (H-imageVal);
				int sourceValc = (H-imageValb) + (H-imageValg);// + (H-imageValg); //if color cut is needed for initial images
				///int sourceValc = (H-imageValb) + (H-imageValg)+ (H-imageValr); //if color cut is needed for initial images
				
				//int sinkVal = imageVal;
				
				int sinkValc = imageValb + imageValg ;//+ imageValr; if color cut is needed
				///int sinkValc = imageValb + imageValg + imageValr; 
				
				//call a function that train on a given image and gives us the unary value of source and sink.
				//nodeW.push_back(Vec2i(sourceVal,sinkVal));
				
				nodeW.push_back(Vec2i(sourceValc,sinkValc)); ///if color cut is needed
			}
		}
}




void MarkovRandomField :: smoothness(Mat image, Mat image_index, Mat imagecolor){
	int count = 0;
		for(int y = 0; y<image.rows; y++){
			for(int x = 0; x<image.cols; x++){
				int sum =0;
				int sumc = 0;
				//int nodeVal = image.at<bool>(y,x);
				
				int nodeValb = (int)imagecolor.at<Vec3b>(y,x)[0];
				int nodeValg = (int)imagecolor.at<Vec3b>(y,x)[1];
				int nodeValr = (int)imagecolor.at<Vec3b>(y,x)[2];
				
				
				for(int i = y-1; i <= y+1; i++){
					for(int j = x-1; j<= x+1; j++){
						if(i<0 || i >= image.rows || j<0 || j>=image.cols){
							
							}
						
						else{
							//int pixelVal =image.at<bool>(i,j);
							
							int pixelValb = (int)imagecolor.at<Vec3b>(i,j)[0];
							int pixelValg = (int)imagecolor.at<Vec3b>(i,j)[1];
							int pixelValr = (int)imagecolor.at<Vec3b>(i,j)[2];
							//cout<<"Neighbouring node to "<<count<<" "<<image_index.at<int>(i,j)<<endl;
							sumc = abs((pixelValb-nodeValb) + (pixelValg-nodeValg) + (pixelValr-nodeValr));
							
							//sum = (pixelVal-nodeVal)*(pixelVal-nodeVal);
							
							///here i am pushing the values of current node id , neighbour node id , and edge potential between them
							if(sumc<KC)
							smooth.push_back(Vec3i(count , image_index.at<int>(i,j), sumc ));
							else
							smooth.push_back(Vec3i(count , image_index.at<int>(i,j), KC ));
						}
						
					}
				}
				count++;
			}
		}	
}

void MarkovRandomField :: correct(Mat binary,Mat image){
	for(int i = 0 ; i<binary.rows ; i++){
		for(int j=0 ; j<binary.cols ; j++){
			if(binary.at<uchar>(i,j)==255){
				image.at<Vec3b>(i,j)[0]=0;
			    image.at<Vec3b>(i,j)[1]=255;
			    image.at<Vec3b>(i,j)[0]=0;
			}
		}
	}
			
}






void MarkovRandomField :: Mrf(Mat imageGray, Mat mask, Mat colorImage){

	cout << mask.rows << endl ;
	cout << mask.cols << endl ;
	
	resize(imageGray,imageGray,cv::Size(COL,ROW));
	resize(mask,mask,cv::Size(COL,ROW));
	Mat image = imageGray.mul(mask/255);   ///dividing to get multiply by 1 and take only the masked value.
	
	Mat initialise(Size(COL,ROW),CV_8UC1,Scalar(0));
	
	
	//MrfOutput(Size(ROW,COL),CV_8UC1,Scalar(0));
	initialise.copyTo(MrfOutput);
	
	
	Mat delement(8,8,CV_8UC1);
	Mat eelement(20,20,CV_8UC1,Scalar(255));
	
	//Mat eselement(4,4,CV_8UC1,Scalar(255));

	
	int node_index=0;

	Mat image_index=Mat_<int>(image.rows,image.cols);
	

	for(int i=0;i<image.rows;i++){
		for(int j=0;j<image.cols;j++){
			image_index.at<int>(i,j)=node_index;
			node_index++;
		}
	}

	
	Mat colorSeg = convert3channel(mask);
	cout << "now here" << endl;
	Mat cutColor = colorImage.mul(colorSeg/255);
	
	
	unary(image,cutColor);
	smoothness(image, image_index, cutColor);
	std::vector<Vec2i>:: iterator ssnode_weight = nodeW.begin();
	std::vector<Vec3i>:: iterator bnode_weight = smooth.begin();
	
	int node=0;




	typedef Graph<int,int,int> GraphType;
	GraphType *g = new GraphType(nodeW.size(),smooth.size()-nodeW.size()); 

	
	while(ssnode_weight!=nodeW.end()){
		Vec2i temp =  *ssnode_weight;
		g -> add_node(); 
		g -> add_tweights( node,(int) temp(0), (int)temp(1) );
		//cout<<"Source Weight"<<" "<<"Sink Weight "<<temp(0)<<" "<<temp(1)<<endl;
		*ssnode_weight++;
		node++;
	}
	
	while(bnode_weight!=smooth.end()){
		Vec3i linkw =  *bnode_weight;
			if(linkw(0)!=linkw(1)){
				g -> add_edge( (int)linkw(0), (int)linkw(1), (int)linkw(2), (int)linkw(2) );
				//cout<<"Node_id1"<<" "<<"Node_id2 "<<" Distance "<< linkw(0) <<" "<<linkw(1)<<" "<<linkw(2)<<endl;
				}
		*bnode_weight++;
	}
	
	int flow = g -> maxflow();
	
	Mat binary(image.size(),CV_8UC1);
	for(int i=0;i<nodeW.size();i++){
		if (g->what_segment(i) == GraphType::SOURCE){
			//printf("node %d is in the SOURCE set\n",i);
			binary.at<uchar>(i/image.cols,i%image.cols)=0;
			}
		else{
			//printf("node %d is in the SINK set\n",i);
			binary.at<uchar>(i/image.cols,i%image.cols)=255;
		}

	}
	
	erode(binary,binary,eelement);
	
	dilate(binary,binary,eelement);
	
	clusterSeg(binary);
	
	imwrite("blob.jpg",binary);
	
	binary.copyTo(MrfOutput);
	correct(binary, colorImage);
	
	delete g;

}
