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


typedef union U_FloatParse {
    float float_data;
    unsigned char byte_data[4];
} U_FloatConvert;
//sensor_msgs::CvBridge obj1;
//namespace enc = sensor_msgs::image_encodings;
using namespace cv;
using namespace std;

vector<Point2f>mc;
int counter=0;
vector<Point2f>goal;
Mat im;
int h;
Mat debug;
Mat cmat;
ros::Publisher goal_pub;
ros::Publisher pub2;
ros::Publisher start_pub;
int x=1,y=1,z=1;
float ox=2.0,oy=2.0;
float resolution=2.0;
double xg,yg,theta;
int focal=580;//focal length kinect-IR

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

    goal_pub.publish(goal);

}

void goalCallback(const nav_msgs::OccupancyGrid &msg2)
{
    ox=(-1*msg2.info.origin.position.x);
    oy=(-1*msg2.info.origin.position.y);
    resolution=msg2.info.resolution;
    h=msg2.info.height;
    ox=(ox/resolution);
    oy=int (oy/resolution);
    oy=float(h-oy);

    
    ROS_INFO("Get Goal");
    cout<<"s";
    const geometry_msgs::Quaternion msg_q=msg2.info.origin.orientation;
    cout<<"aaa";
    double beta=atan2(goal[0].y-oy,goal[0].x-ox);
    double x_inner=(goal[0].x-ox)*resolution;
    double y_inner=(goal[0].y-oy)*resolution;
    moveTo(x_inner, y_inner, beta);
    //px=x;py=y;
    std_msgs::Bool t;
    t.data=true;
    start_pub.publish(t);
    //ros::Duration(20).sleep();



}

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





void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    
       
        // if(imwrite("1.pgm", cv_bridge::toCvShare(msg, "bgr8")->image))std::cout<<"done"<<endl;
        // waitKey(10);
        // if(imwrite("2.pgm", cv_bridge::toCvShare(msg, "bgr8")->image))std::cout<<"done"<<endl;
        // waitKey(10);
        if(imwrite("1.jpg", cv_bridge::toCvShare(msg, "bgr8")->image))std::cout<<"done"<<endl;
        waitKey(10);
        if(imwrite("2.jpg", cv_bridge::toCvShare(msg, "bgr8")->image))std::cout<<"done"<<endl;
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
            min[i].x=9999;
            min[i].y=9999;
            mu[i] = moments( contours[i], false );
            for(int j=0;j< contours[i].size();j++){

                if(contours[i][j].y<=min[i].y){
                    min[i].y=contours[i][j].y;
                    min[i].x=contours[i][j].x;
                }
            }
        }

        int flag=0;

        //vector<Point2f> mc( contours.size() );
        for( int i = 0; i < contours.size(); i++ )
        { 
            if(contourArea(contours[i])>500)
             {
                
                //mc.push_back(Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ));
                mc.push_back(Point2f( float(min[i].x), float(min[i].y) ) );
                cout<<"x"<<" "<<int (mc[i].x)<<"y"<<" "<<int (mc[i].y)<<endl;
                flag=1;
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

        namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
        imshow( "Contours", drawing );


        //if(!flag){
        waitKey(10000);
       
   


    /*cv_bridge::CvImagePtr cv_ptr=toCvCopy(msg,enc::BGR8);
    cout<<"rows"<<cv_ptr->image.rows<<" "<<"cols"<<cv_ptr->image.cols<<endl;
    im=Mat(cv_ptr->image);

    
//to be written
*/
}

void depthCallback(const sensor_msgs::ImageConstPtr& msg1)
{
     for(int a=0;a<1;a++)
     {
        
        int depth = ReadDepthData(int(mc[a].y),int(mc[a].x), msg1);
       // depth /= 1000;
        int x1 = int(mc[a].x)* depth / focal ;
        int y1 = int(mc[a].y)* depth / focal ;
        int z1 = depth;
        z1 /=1000;
        x1 /=1000;
        y1 /=1000;
        ROS_INFO("Coordinates (camera_frame): %d,%d,%d", x1, y1, z1);
        ROS_INFO("Depth: %d", z1);
        // xg = -x;
        // yg = z;
        // goal[a].x=float(xg);
        // goal[a].y=float(yg);
        // std::cout<<"hhjnjn"<<endl;
    }

}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"moving");
    ros::NodeHandle n1;
    ros::NodeHandle n2;
    ros::NodeHandle n3;
    image_transport::ImageTransport it(n1);
    image_transport::ImageTransport it1(n2);

   // ros::Subscriber sub1=n1.subscribe("/camera/rgb/color_image",1,imageCallback);
   // ros::Subscriber sub2=n1.subscribe("/camera/depth/image_raw",1,depthCallback);
    //ros::Subscriber sub3=n3.subscribe("map",1,goalCallback);
    //pub2 = n3.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1);
    //image_transport::Subscriber sub2 = it.subscribe("camera/depth/image", 1, depthCallback);
    image_transport::Subscriber sub1 = it1.subscribe("camera/rgb/image_color", 1, imageCallback);
    
    


    ros::Rate r(1000);

    while(ros::ok()){

      ros::spinOnce();
      r.sleep();
    }
    return 0;
    //image=imread("folder_name",0);

}