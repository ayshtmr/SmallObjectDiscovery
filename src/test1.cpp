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


typedef union U_FloatParse {
    float float_data;
    unsigned char byte_data[4];
} U_FloatConvert;


sensor_msgs::ImageConstPtr msga,msgc;
nav_msgs::OccupancyGrid msgb;


int counter;
Mat im;
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
float heightofcam=0.5;
double xg,yg,theta;
int focal=530;//focal length kinect-RGB
int explore_var=0;


void moveTo(double a, double b, double alpha)
{
    ROS_INFO("Move to (%lf, %lf, %lf)", a, b, alpha);
    cout << "Move" << a <<" "<< b;
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id="base_footprint";
    goal.header.stamp=ros::Time::now();

    goal.pose.position.x=b;
    goal.pose.position.y=a;
    goal.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(0, 0, alpha);


    pub2.publish(goal);
    waitKey(15000);
    

}

int flag1=0,flag2=0,flag3=0;

void goalCallback(const nav_msgs::OccupancyGrid &msg2)
{
    msgb=msg2;
    flag1=1;
    std::cout<<"Map <-"<<endl;

}

void callback( const sensor_msgs::ImageConstPtr &msg)
{
    msga=msg;
    flag2=1;
    std::cout<<"Img <-"<<endl;

}

int ctr=1;


void callit(const sensor_msgs::ImageConstPtr msg3, const nav_msgs::OccupancyGrid msg4/*, const sensor_msgs::ImageConstPtr& msg5*/)
{

    if(explore_var){

        switch(ctr)
        {
            case 1 : moveTo(0.00,1.00,0.00);
                    break;
            case 2 : moveTo(0.00,0.00,1.0467);
                    break;
            case 3 : moveTo(0.00,0.00,-2.0933);
                    break;        
            case 4 : moveTo(0.00,0.00,1.0467);
                    break;
        }
    ctr=(ctr+1)%4;
    explore_var=0;
    return;
    }


    if(imwrite("1.jpg", cv_bridge::toCvShare(msg3, "bgr8")->image))std::cout<<"done"<<endl;
    waitKey(10);
    if(imwrite("2.jpg", cv_bridge::toCvShare(msg3, "bgr8")->image))std::cout<<"done"<<endl;
    waitKey(10);
    system("convert 1.jpg 1.pgm");
    system("convert 2.jpg 2.pgm");


    system("./run");
    

    std::cout<<endl<<"Returned"<<endl;

    Mat src; Mat src_gray;
    int thresh = 255;
    int max_thresh = 255;
    RNG rng(12345);

    src=imread("blob.jpg");
    cout<<"blob"<<endl;

    int counter=0;
    
    cvtColor( src, src_gray, CV_BGR2GRAY );
    blur( src_gray, src_gray, Size(3,3) );
    Mat canny_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;


    Canny( src_gray, canny_output, thresh, thresh*2, 3 );

    findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );


    vector <Point> min(contours.size());

    for( int i = 0; i < contours.size(); i++ )
    {   
        min.push_back(Point(-1,-1));

        for(int j=0;j< contours[i].size();j++){

            if(contours[i][j].y>min[i].y){
                min.pop_back();
                min.push_back(Point(contours[i][j].x,contours[i][j].y));
            }
        }
    }

    

    vector<Point2f>mc;
    for( int i = 0; i < contours.size(); i++ )
    { 
        if(contourArea(contours[i])>100)
         {
            
            if(i==0)
                mc.push_back(Point2f( float(min[i].x), float(min[i].y) ) );
            else
                if(min[i].y!=min[i-1].y)
                    mc.push_back(Point2f( float(min[i].x), float(min[i].y) ) );
            cout<<"x"<<" "<<int (mc[i].x)<<"y"<<" "<<int (mc[i].y)<<endl;
            counter++;
        }
        
    }

    if(mc.size()==0){
        explore_var=1;
        return;
    }


    Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );

    for( int i = 0; i< contours.size(); i++ )
    {
        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
        circle( drawing, mc[i], 4, color, -1, 8, 0 );
    }


    vector<Point2f>goal;
    namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
    imshow( "Contours", drawing );
    waitKey(1000);

    for (int k=0;k<counter;k++)
    {
        float obj_height=mc[k].y-240;
        float z1=focal*heightofcam/obj_height;
        z1 = z1*z1*z1*0.2549 - z1*z1*0.9714 + z1*2.9818 - 1.0008;
        float x1=(320-mc[k].x)*z1/focal;
        cout<<endl<<"Depth "<<z1<<endl<<"X "<<x1<<endl;
        goal.push_back(Point2f(x1,z1));
    
    }    

    
    for(int a=0;a<counter;a++)
        {
            waitKey(15000);
            if (a==0)
            {
               ROS_INFO("Get Goal : %d",a);
        //const geometry_msgs::Quaternion msg_q=msg2.info.origin.orientation;
               double beta=atan2(goal[a].x,goal[a].y);
               double x_inner=(goal[a].x);
               double y_inner=(goal[a].y);
               moveTo(x_inner, y_inner, beta); /* code */
            }
            else
            {
               ROS_INFO("Get Goal : %d",a);
               double xa,ya,tempang;
               tempang=atan2(goal[a-1].x,goal[a-1].y);
               xa=goal[a].x-goal[a-1].x;ya=goal[a].y-goal[a-1].y;
        //const geometry_msgs::Quaternion msg_q=msg2.info.origin.orientation;
               double beta=atan2(xa*cos(tempang)-ya*sin(tempang),xa*sin(tempang)+ya*cos(tempang));
               double x_inner=xa*cos(tempang)-ya*sin(tempang)/*(goal[a].x-goal[a-1].x)*/;
               double y_inner=xa*sin(tempang)+ya*cos(tempang)/*(goal[a].y-goal[a-1].y)*/;
               moveTo(x_inner, y_inner, beta);
            }    
   
    }

    ROS_INFO("Goals over \n");

}


int main(int argc, char** argv)
{
    ros::init(argc,argv,"moving");
    ros::NodeHandle n1;
    image_transport::ImageTransport it(n1);
    image_transport::ImageTransport it1(n1);
    image_transport::Subscriber sub2=it1.subscribe("/camera/rgb/image_color",1,callback);
    ros::Subscriber sub3=n1.subscribe("map",1,goalCallback);   
    pub2 = n1.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1);

    while(ros::ok())
    {
        ros::spinOnce();

        if(flag1&&flag2)
            callit(msga, msgb);

        ros::Rate r(30000);
        r.sleep();
    }

    return 0;

}