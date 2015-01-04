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



namespace enc = sensor_msgs::image_encodings;
using namespace cv;
using namespace std;
float heightofcam=0.3;

typedef union U_FloatParse {
    float float_data;
    unsigned char byte_data[4];
} U_FloatConvert;

// typedef message_filters::TimeSynchronizer<sensor_msgs::Image,nav_msgs::OccupancyGrid> Sync;
sensor_msgs::ImageConstPtr msga,msgc;
nav_msgs::OccupancyGrid msgb;
vector<Point2f>mc;

int counter;
Mat im;
Mat src_gray;
int thresh=100;
int h;
//int heightofcam=0.25;
int h1=480;
ros::Publisher goal_pub;
ros::Publisher pub2;
ros::Publisher start_pub;
double x,y,z=1.0;
int ox,oy;
float resolution;
double xg,yg,theta;
int focal=525;//focal length kinect-IR



int ReadDepthData(unsigned int height_pos, unsigned int width_pos, sensor_msgs::ImageConstPtr depth_image)
{
    // If position is invalid
    if ((height_pos >= depth_image->height) || (width_pos >= depth_image->width))
        return -1;
    int index = (height_pos*depth_image->step) + (width_pos*(depth_image->step/depth_image->width));
    // If data is 4 byte floats (rectified depth image)
    if ((depth_image->step/depth_image->width) == 4) {
        U_FloatConvert depth_data;
        int i, endian_check = 1;
        // If big endian
        if ((depth_image->is_bigendian && (*(char*)&endian_check != 1)) ||  // Both big endian
                ((!depth_image->is_bigendian) && (*(char*)&endian_check == 1))) { // Both lil endian
            for (i = 0; i < 4; i++)
                depth_data.byte_data[i] = depth_image->data[index + i];
            // Make sure data is valid (check if NaN)
            if (depth_data.float_data == depth_data.float_data)
                return int(depth_data.float_data*1000);
            return -1;  // If depth data invalid
        }
        // else, one little endian, one big endian
        for (i = 0; i < 4; i++)
            depth_data.byte_data[i] = depth_image->data[3 + index - i];
        // Make sure data is valid (check if NaN)
        if (depth_data.float_data == depth_data.float_data)
            return int(depth_data.float_data*1000);
        return -1;  // If depth data invalid
    }
    // Otherwise, data is 2 byte integers (raw depth image)
    int temp_val;
    // If big endian
    if (depth_image->is_bigendian)
        temp_val = (depth_image->data[index] << 8) + depth_image->data[index + 1];
        // If little endian
    else
        temp_val = depth_image->data[index] + (depth_image->data[index + 1] << 8);
    // Make sure data is valid (check if NaN)
    if (temp_val == temp_val)
        return temp_val;
    return -1;  // If depth data invalid
}


void moveTo(double a, double b, double alpha)
{
    ROS_INFO("Move to (%lf, %lf, %lf)", a, b, alpha);
    cout << "Move" << a <<" "<< b;
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id="map";
    goal.header.stamp=ros::Time::now();

    goal.pose.position.x=a;
    goal.pose.position.y=b;
    goal.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(0, 0, alpha);
    // std_msgs::Bool t;
    // t.data=true;
    // start_pub.publish(t);

    pub2.publish(goal);
    waitKey(10000);

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

/*void depthCallback(const sensor_msgs::ImageConstPtr& msg1){

    msgc=msg1;
    flag3=1;
    std::cout<<"Depth <-"<<endl;

}*/


void callit(const sensor_msgs::ImageConstPtr msg3, const nav_msgs::OccupancyGrid msg4/*, const sensor_msgs::ImageConstPtr& msg5*/)
{

    


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

        int counter=0;
        
        //cout<<int(src_gray.channels())<<"<--- depth";


        //part for lsd and wlc
        cvtColor( src, src_gray, CV_BGR2GRAY );
        blur( src_gray, src_gray, Size(3,3) );
        Mat canny_output;
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;


        Canny( src_gray, canny_output, thresh, thresh*2, 3 );

        findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );


        vector<Moments> mu(contours.size() );
        vector <Point> min(contours.size());

        for( int i = 0; i < contours.size(); i++ )
        {   
            min[i].x=-1;
            min[i].y=-1;
            mu[i] = moments( contours[i], false );
            for(int j=0;j< contours[i].size();j++){

                if(contours[i][j].y>=min[i].y){
                    min[i].y=contours[i][j].y;
                    min[i].x=contours[i][j].x;
                }
            }
        }

        

        //vector<Point2f> mc( contours.size() );
        for( int i = 0; i < contours.size(); i++ )
        { 
            if(contourArea(contours[i])>500)
             {
                
                //mc.push_back(Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ));
                mc.push_back(Point2f( float(min[i].x), float(min[i].y) ) );
                cout<<"x"<<" "<<int (mc[i].x)<<"y"<<" "<<int (mc[i].y)<<endl;
                counter++;
            }
            
        }


        Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
        for( int i = 0; i< contours.size(); i++ )
        {
            Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
            circle( drawing, mc[i], 4, color, -1, 8, 0 );
        }
        //circle(drawing, Point(100,100),4,Scalar(100,100,100),-1,8,0);
        //circle(drawing, Point(100,150),4,Scalar(255,255,255),-1,8,0);

        vector<Point2f>goal;
        namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
        imshow( "Contours", drawing );
        waitKey(1000);

        /*    for (int j = 0; j < 1 ; ++j)
            {
                //minimum point of the contour
                float depth = ReadDepthData(int(mc[j].y),int(mc[j].x), msg5);
                float x1 = mc[j].x* depth / focal ;
                float y1 = mc[j].y* depth / focal ;
                float z1 = depth;
                z1 /=1000;
                x1 /=1000;
                y1 /=1000;
                ROS_INFO("Coordinates (camera_frame): %f,%f,%f", x1, y1, z1);
                
                goal.push_back(Point2f(-x1,z1));

            }*/
               for (int k=0;k<1;k++)
               {
                float obj_height=mc[k].y;
                float z1=focal*heightofcam/obj_height;
                float x1=mc[k].x*heightofcam/obj_height;
                goal.push_back(Point2f(-x1,z1));


               } 


        

    ox=-1*msg4.info.origin.position.x;
    oy=-1*msg4.info.origin.position.y;
    std::cout<<ox<<endl;
    std::cout<<oy<<endl;    
    resolution=msg4.info.resolution;
    h=msg4.info.height;
    //ox=int (ox/resolution);
    oy=oy/resolution;
    oy=h-oy;
    oy= oy*resolution;
    ROS_INFO("Get Goal");
    //const geometry_msgs::Quaternion msg_q=msg2.info.origin.orientation;
    double beta=atan2(goal[0].y-oy,goal[0].x-ox);
    double x_inner=(goal[0].x-ox);
    double y_inner=(goal[0].y-oy);
    moveTo(x_inner, y_inner, beta);
    //px=x;py=y;
   
    //ros::Duration(20).sleep();


}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"moving");
    ros::NodeHandle n1;
    image_transport::ImageTransport it(n1);
    image_transport::ImageTransport it1(n1);
    //message_filters::Subscriber<sensor_msgs::Image> image_sub(n1, "camera/rgb/image_color", 1);
    //message_filters::Subscriber<nav_msgs::OccupancyGrid> map_sub(n1, "map", 1);
    //Sync sync(image_sub, map_sub, 10);
    //sync.registerCallback(boost::bind(&callback,_1,_2));
    image_transport::Subscriber sub2=it1.subscribe("/camera/rgb/image_color",1,callback);
    //image_transport::Subscriber sub1 = it.subscribe("camera/rgb/image_color", 1, callback);
    //image_transport::Subscriber sub1 = it.subscribe("camera/depth/image", 1, depthCallback);
    ros::Subscriber sub3=n1.subscribe("map",1,goalCallback);
    
    pub2 = n1.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1);
    //ros::spin();

    
    while(ros::ok())
    {
        ros::spinOnce();
        //ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0));

        // ros::MultiThreadedSpinner spinner(3); // Use 4 threads
        // spinner.spin();
        if(flag1&&flag2)
            callit(msga, msgb);
        waitKey(10000);
        ros::Rate r(30000);
        r.sleep();
    }

    return 0;

}