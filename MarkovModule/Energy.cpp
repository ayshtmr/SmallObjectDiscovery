#include "cv.h"
#include "highgui.h"
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <cmath>
#include "graph.h"


#define H 200
#define L 0
#define K 100
#define ROWS 640
#define COLS 480

using namespace std;
using namespace cv;


class energy{
	public : vector<Vec3i> smooth;
	public : vector<Vec2i> nodeW; 
	public : void unary(Mat image);
	public : void smoothness(Mat image, Mat image_index);
	public : void correct(Mat binary,Mat image);
};


void energy :: unary(Mat image){
		for(int y = 0; y < image.rows; y++){
			for(int x = 0; x< image.cols; x++){
				int imageVal = image.at<bool>(y,x);
				int sourceVal = (H-imageVal);
				int sinkVal = imageVal;
				//call a function that train on a given image and gives us the unary value of source and sink.
				nodeW.push_back(Vec2i(sourceVal,sinkVal));
			}
		}
}



void energy :: smoothness(Mat image, Mat image_index){
	int count = 0;
		for(int y = 0; y<image.rows; y++){
			for(int x = 0; x<image.cols; x++){
				int sum =0;
				int nodeVal = image.at<bool>(y,x);
				for(int i = y-1; i <= y+1; i++){
					for(int j = x-1; j<= x+1; j++){
						if(i<0 || i >= image.rows || j<0 || j>=image.cols){
							
							}
						
						else{
							int pixelVal =image.at<bool>(i,j);
							//cout<<"Neighbouring node to "<<count<<" "<<image_index.at<int>(i,j)<<endl;
							sum = (pixelVal-nodeVal)*(pixelVal-nodeVal);
							///here i am pushing the values of current node id , neighbour node id , and edge potential between them
							if(sum<K)
							smooth.push_back(Vec3i(count , image_index.at<int>(i,j), sum ));
							else
							smooth.push_back(Vec3i(count , image_index.at<int>(i,j), K ));
						}
						
					}
				}
				count++;
			}
		}	
}

void energy :: correct(Mat binary,Mat image){
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


int main(int argc , char *argv[]){
	Mat image = imread(argv[1],0);
	Mat imageplot = imread(argv[1],1);
	resize(image,image,cv::Size(ROWS,COLS));
	resize(imageplot,imageplot,cv::Size(ROWS,COLS));
	
	Mat delement(8,8,CV_8UC1);
	Mat eelement(20,20,CV_8UC1);
	int node_index=0;

	Mat image_index=Mat_<int>(image.rows,image.cols);
	
	for(int i=0;i<image.rows;i++){
		for(int j=0;j<image.cols;j++){
			image_index.at<int>(i,j)=node_index;
			node_index++;
		}
	}
	
	
	energy en;
	en.unary(image);
	en.smoothness(image, image_index);
	
	std::vector<Vec2i>:: iterator ssnode_weight = en.nodeW.begin();
	std::vector<Vec3i>:: iterator bnode_weight = en.smooth.begin();
	
	int node=0;

	typedef Graph<int,int,int> GraphType;
	GraphType *g = new GraphType(en.nodeW.size(),en.smooth.size()-en.nodeW.size()); 

	
	while(ssnode_weight!=en.nodeW.end()){
		Vec2i temp =  *ssnode_weight;
		g -> add_node(); 
		g -> add_tweights( node,(int) temp(0), (int)temp(1) );
		//cout<<"Source Weight"<<" "<<"Sink Weight "<<temp(0)<<" "<<temp(1)<<endl;
		*ssnode_weight++;
		node++;
	}
	
	while(bnode_weight!=en.smooth.end()){
		Vec3i linkw =  *bnode_weight;
			if(linkw(0)!=linkw(1)){
				g -> add_edge( (int)linkw(0), (int)linkw(1), (int)linkw(2), (int)linkw(2) );
				//cout<<"Node_id1"<<" "<<"Node_id2 "<<" Distance "<< linkw(0) <<" "<<linkw(1)<<" "<<linkw(2)<<endl;
				}
		*bnode_weight++;
	}
	
	int flow = g -> maxflow();
	
	Mat binary(image.size(),CV_8UC1);
	for(int i=0;i<en.nodeW.size();i++){
		if (g->what_segment(i) == GraphType::SOURCE){
			//printf("node %d is in the SOURCE set\n",i);
			binary.at<uchar>(i/image.cols,i%image.cols)=0;
			}
		else{
			//printf("node %d is in the SINK set\n",i);
			binary.at<uchar>(i/image.cols,i%image.cols)=255;
		}

	}
	dilate(binary,binary,delement);
	erode(binary,binary,eelement);
	en.correct(binary,imageplot);
	delete g;
	namedWindow("Image");
	imshow("Image",imageplot);
	waitKey(0);	
	return 0;
}



