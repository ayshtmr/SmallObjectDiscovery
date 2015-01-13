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

using namespace cv;
using namespace std;

int state;

int px,py,ox,oy;

int thresh=50;

int flag=0;

double home_theta;

float resolution;

double now_x;
double now_y;
double now_theta;

int h,w;


ros::Publisher start_pub;
ros::Publisher goal_pub;

void moveTo(double x, double y, double theta)
{

    ROS_INFO("Move to (%lf, %lf, %lf)", x, y, theta);
    cout << "Move" << x <<" "<< y;
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id="/map";
    goal.header.stamp=ros::Time::now();

    goal.pose.position.x=y;
    goal.pose.position.y=x;
    goal.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(0, 0, theta);

    goal_pub.publish(goal);

}


void GoalCallback( const nav_msgs::OccupancyGrid &msg)
{

    if(flag==0)
  {
        resolution=msg.info.resolution;
        // cout<<"resolution" <<resolution << endl;
        ox=-1*msg.info.origin.position.x;
        oy=-1*msg.info.origin.position.y;
        cout << " ox " <<ox << " oy "<< oy<< endl;
        h=msg.info.height;
        w=msg.info.width;

        cout << " h " <<h << " w "<< w<< endl;

        ox=int (ox/resolution);
        oy=int (oy/resolution);

        cout << " ox1 " <<ox << " oy "<< oy<< endl;

        oy=h-oy;

        cout << " ox2 " <<ox << " oy "<< oy<< endl;

        px=ox;
        py=oy;

        Mat src(h,w,CV_8UC1,Scalar(0));
        // src = imread( "/tmp/map.pgm", 1 );


        int c=0;
        for(int a=0;a<h;a++){
            for(int b=0;b<w;b++){
                 if(msg.data[c]==-1){
                    src.at<uchar>(a,b)=120;
                }
            else if(msg.data[c]==0){
                    src.at<uchar>(a,b)=255;
                }
            else
                    src.at<uchar>(a,b)=0;

            c++;
            }
        }


        blur( src, src, Size(3,3) );

        Mat canny_output;
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        Canny( src, canny_output, thresh, thresh*2, 3 );

        int x=0,y=0;


        findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

        for( int i = 0; i< contours.size(); i++ )
        {

            drawContours( src, contours, i, Scalar(90), 2, 8, hierarchy, 0, Point() );
        }

        int clusterCount = 1;//contours.size();//contours.size();
        //int clusterCount = 1;
        Mat labels;
        //  int attempts = 5;
        Mat centers;


        vector<Point2f> l;

        for(int i = 0; i < contours.size(); i++)
        {
            for(int j = 0; j < contours[i].size(); j++){
            // cout << (float)contours[i][j].x << "x" << (float)contours[i][j].y << " ";
            l.push_back(Point((float)contours[i][j].x,(float)contours[i][j].y));
            }
        }

        cout<<"con size "<<contours.size();
        cout<<" l size "<<l.size();

        Mat xx(l.size(),2,CV_32FC1);
        int cz=0;
        for(int i = 0; i < contours.size(); i++)
            for(int j = 0; j < contours[i].size(); j++){

                xx.at<float>(cz,0)=contours[i][j].x;
                xx.at<float>(cz,1)=contours[i][j].y;
                //  cout<<"x "<<int(xx.at<float>(cz,0))<<" "<<int(xx.at<float>(cz,1));
                cz++;

            }

        //cout<<endl<<xx.rows<<" fsdfdsf "<<xx.cols;



        kmeans(xx, clusterCount, labels,TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,10000,0.0001), 10, KMEANS_RANDOM_CENTERS, centers);



        vector<float>k(clusterCount);
        for (int i=0; i<clusterCount; i++)
        {
            int a=centers.at<float>(i,0);
            int b=centers.at<float>(i,1);
            k[i]=std::sqrt((float) ((a-px)^2+(b-py)^2));
            //cout<<endl<<"a "<<a<<"b "<<b;
        }

        float min=k[0];
        for (int i=0; i<clusterCount; i++){
            // {for (int j=0;j<contours.size(); j++)
            {
                if(min>k[i])
                {
                    min=k[i];
                    x=centers.at<float>(i,0);
                    y=centers.at<float>(i,1);
                }
            }
        }


        cout<<endl<<"x "<<x<<"y "<<y<<endl;




        for (int i=0; i<clusterCount; i++){

            circle(src,Point(centers.at<float>(i,0),centers.at<float>(i,1)),2,Scalar(0),10);
            cout<<"point : "<<int(centers.at<float>(i,0))<<" "<<int(centers.at<float>(i,1))<<endl;

        }

        circle(src,Point(x,y),2,Scalar(110),10);

        imwrite( "/home/ayush/Desktop/img.png",src);



        ROS_INFO("Get Goal");
        const geometry_msgs::Quaternion msg_q=msg.info.origin.orientation;
        //home_theta=tf::getYaw(msg_q);
        //double ln=resolution*sqrt(x*x+y*y);
        double beta=atan2(x-ox,y-oy);
        //double x_vector=ln*cos(beta+home_theta);
        //double y_vector=ln*sin(beta+home_theta);
        double x_inner=(x-ox)*resolution;
        double y_inner=(y-oy)*resolution;
        //home_theta=beta+home_theta;
        moveTo(x_inner, y_inner, beta);
        px=x;py=y;
        std_msgs::Bool t;
        t.data=true;
        start_pub.publish(t);

      //  ros::Duration(20).sleep();

        flag=1;

  }

  else
  {

        resolution=msg.info.resolution;
        // cout<<"resolution" <<resolution << endl;
        ox=-1*msg.info.origin.position.x;
        oy=-1*msg.info.origin.position.y;
        cout << " ox " <<ox << " oy "<< oy<< endl;
        h=msg.info.height;
        w=msg.info.width;

        cout << " h " <<h << " w "<< w<< endl;

        ox=int (ox/resolution);
        oy=int (oy/resolution);

        cout << " ox1 " <<ox << " oy "<< oy<< endl;

        oy=h-oy;

        cout << " ox2 " <<ox << " oy "<< oy<< endl;


        Mat src(h,w,CV_8UC1,Scalar(0));
        // src = imread( "/tmp/map.pgm", 1 );


        int c=0;
        for(int a=0;a<h;a++){
            for(int b=0;b<w;b++){
                 if(msg.data[c]==-1){
                    src.at<uchar>(a,b)=120;
                }
            else if(msg.data[c]==0){
                    src.at<uchar>(a,b)=255;
                }
            else
                    src.at<uchar>(a,b)=0;

            c++;
            }
        }


        blur( src, src, Size(3,3) );

        Mat canny_output;
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        Canny( src, canny_output, thresh, thresh*2, 3 );

        int x=0,y=0;


        findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

        for( int i = 0; i< contours.size(); i++ )
        {

            drawContours( src, contours, i, Scalar(90), 2, 8, hierarchy, 0, Point() );
        }

         int clusterCount = 1;//contours.size();//contours.size();
         //int clusterCount = 1;
         Mat labels;
         //  int attempts = 5;
         Mat centers;


         vector<Point2f> l;

         for(int i = 0; i < contours.size(); i++)
         {
             for(int j = 0; j < contours[i].size(); j++){
             // cout << (float)contours[i][j].x << "x" << (float)contours[i][j].y << " ";
             l.push_back(Point((float)contours[i][j].x,(float)contours[i][j].y));
             }
         }

         cout<<"con size "<<contours.size();
         cout<<" l size "<<l.size();

         Mat xx(l.size(),2,CV_32FC1);
         int cz=0;
         for(int i = 0; i < contours.size(); i++)
             for(int j = 0; j < contours[i].size(); j++){

                 xx.at<float>(cz,0)=contours[i][j].x;
                 xx.at<float>(cz,1)=contours[i][j].y;
                 //  cout<<"x "<<int(xx.at<float>(cz,0))<<" "<<int(xx.at<float>(cz,1));
                 cz++;

             }

         //cout<<endl<<xx.rows<<" fsdfdsf "<<xx.cols;



         kmeans(xx, clusterCount, labels,TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,10000,0.0001), 10, KMEANS_RANDOM_CENTERS, centers);



         vector<float>k(clusterCount);
         for (int i=0; i<clusterCount; i++)
         {
             int a=centers.at<float>(i,0);
             int b=centers.at<float>(i,1);
             k[i]=std::sqrt((float) ((a-px)^2+(b-py)^2));
             //cout<<endl<<"a "<<a<<"b "<<b;
         }

         float min=k[0];
         for (int i=0; i<clusterCount; i++){
             // {for (int j=0;j<contours.size(); j++)
             {
                 if(min>k[i])
                 {
                     min=k[i];
                     x=centers.at<float>(i,0);
                     y=centers.at<float>(i,1);
                 }
             }
         }


         cout<<endl<<"x "<<x<<"y "<<y<<endl;




         for (int i=0; i<clusterCount; i++){

             circle(src,Point(centers.at<float>(i,0),centers.at<float>(i,1)),2,Scalar(0),10);
             cout<<"point : "<<int(centers.at<float>(i,0))<<" "<<int(centers.at<float>(i,1))<<endl;

         }

    circle(src,Point(px,py),2,Scalar(50),10);
    circle(src,Point(x,y),2,Scalar(110),10);

        imwrite( "/home/ayush/Desktop/img2.png",src);



        ROS_INFO("Get Goal 2");
        const geometry_msgs::Quaternion msg_q=msg.info.origin.orientation;
        //home_theta=tf::getYaw(msg_q);
        //double ln=resolution*sqrt(x*x+y*y);
        double beta=atan2(y-oy,x-ox);
        //double x_vector=ln*cos(beta+home_theta);
        //double y_vector=ln*sin(beta+home_theta);
        double x_inner=(x-ox)*resolution;
        double y_inner=(y-oy)*resolution;
        //home_theta=beta+home_theta;
        moveTo(x_inner, y_inner, beta);
        px=x;py=y;
        std_msgs::Bool t;
        t.data=true;
        start_pub.publish(t);

        //ros::Duration(10).sleep();
        flag=1;




    }
}




  int main(int argc,char **argv){
    state=1;
    ros::init(argc, argv, "testing");
    ros::NodeHandle n;

    ros::Subscriber sub=n.subscribe("map",1000,GoalCallback);
    start_pub=n.advertise<std_msgs::Bool>("start",1000);
    goal_pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1000);

    ros::spin();

    return 0;
  }
