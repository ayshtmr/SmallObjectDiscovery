#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <stdlib.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Bool.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "image_transport/image_transport.h"
#include <ros/callback_queue.h>

using namespace cv;
using namespace std;

void wait ( double seconds )
{
clock_t endwait;
endwait = clock () + seconds * CLOCKS_PER_SEC ;
while (clock() < endwait) {}
}

float heightofcam=0.33;
vector<Point2f>mc;
int counter;
Mat src_gray;
int thresh=100;
int h;
int h1=480;
ros::Publisher goal_pub;
ros::Publisher pub2;
ros::Publisher start_pub;
double x,y,z=1.0;
int ox,oy;
float resolution;
double xg,yg,theta;
int focal=530;

void callback( const sensor_msgs::ImageConstPtr &msg)
{
    // int iLowH = 0;
    // int iHighH = 179;

    // int iLowS = 0; 
    // int iHighS = 255;

    // int iLowV = 0;
    // int iHighV = 255;

    // //Create trackbars in "Control" window
    // cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    // cvCreateTrackbar("HighH", "Control", &iHighH, 179);

    // cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    // cvCreateTrackbar("HighS", "Control", &iHighS, 255);

    // cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
    // cvCreateTrackbar("HighV", "Control", &iHighV, 255);

    
    if(imwrite("x.jpg", cv_bridge::toCvShare(msg, "bgr8")->image))std::cout<<"done"<<endl;
    wait(1);
    Mat im = imread("x.jpg");
    imshow("1",im);
    waitKey(10);
    
    Mat imgHSV;

    cvtColor(im, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

    Mat imgThresholded;

    inRange(imgHSV, Scalar(2, 150, 150), Scalar(6, 255, 255), imgThresholded); //Threshold the image
          
    //morphological opening (remove small objects from the foreground)
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

    //morphological closing (fill small holes in the foreground)
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    imshow("Thresholded Image", imgThresholded); 

    int posX=-1;
    int posY=-1;

    //cout<<imgThresholded.rows<<"fdfdf"<<imgThresholded.cols<<endl;
    for(int i=0; i<imgThresholded.rows;i++)
        for(int j=0;j<imgThresholded.cols;j++){

            if(imgThresholded.at<uchar>(i,j)==255)
                if(i>=posY){
                    posY=i;
                    posX=j;

            }

        }
    cout<<endl<<"Pos X "<<posX<<" "<<"Pos Y "<<posY<<endl;

    //  Moments oMoments = moments(imgThresholded);

    // double dM01 = oMoments.m01;
    // double dM10 = oMoments.m10;
    // double dArea = oMoments.m00;

    // int posX = dM10 / dArea;
    // int posY = dM01 / dArea;   

     posY -= 240 ;

     float z1=focal*heightofcam/posY;
     z1 = z1*z1*z1*0.4025 - z1*z1*0.7903 + z1*2.4968 - 0.3587;

    // //float x1= posX*heightofcam/posY;
    // float x1=(320-posX)*z1/focal;

     cout<<endl<<"Depth "<<z1<<endl;



    waitKey(1000);





}




int main(int argc, char** argv)
{
    ros::init(argc,argv,"moving");
    ros::NodeHandle n1;
    image_transport::ImageTransport it1(n1);

     namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"



    image_transport::Subscriber sub2=it1.subscribe("/camera/rgb/image_color",1,callback);

    ros::Rate r(1000);

    while(ros::ok())
    {
        ros::spinOnce();

        r.sleep();
    }

    return 0;

}