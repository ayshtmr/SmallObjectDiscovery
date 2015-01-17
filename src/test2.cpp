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
#include <string.h>
#include <fstream>
#include <sstream>
#include <std_msgs/Header.h>
#include <nav_msgs/MapMetaData.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>

#define NO_CLUSTERS 8
#define ATTEMPTS 10
#define SIZE 100000

using namespace cv;
using namespace std;


typedef union U_FloatParse {
    float float_data;
    unsigned char byte_data[4];
} U_FloatConvert;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

typedef pcl::PointXYZRGB Point1;
typedef pcl::PointCloud<pcl::PointXYZRGB> cloudxyz;
typedef cloudxyz::Ptr cloudptr;
typedef pcl::search::KdTree<Point1>::Ptr KdTreePtr;

cloudxyz PC;
string object_ID;
int kn=0;


sensor_msgs::ImageConstPtr msga,msgc;
nav_msgs::OccupancyGrid msgb;
sensor_msgs::PointCloud2::ConstPtr msgd;



int counter;
int ind;
Mat im;
Mat src_gray;
int thresh=100;
int h;
int h1=480;
ros::Publisher goal_pub;
ros::Publisher pub2;
ros::Publisher start_pub;
double x,y,zx=1.0;
int ox,oy;
float resolution;
float heightofcam=0.33; // in meters
double xg,yg,theta;
int focal=530; // Fixed value -- Kinect Focal Lenght
int explore_var=0;
float origin_x;
float origin_y;

float x_last;
float y_last;

float get_min ( float dis[], int size);
int a1;
int b1;
int z=0;

cv::Mat occ(b1,a1,CV_8U,cv::Scalar(0));
float x_nex=0;
float y_nex=0;

int flag;

float get_min( float dis[] ,int size  )
{
    int i;
    float min;
        int min_i=0;
    min=dis[0];
    for (i=1; i<size; i++)
    {   
        if (dis[i] < min)
        {
            min=dis[i];         
            min_i=i;            
        }   
    } 
    return min_i;
}


// Publishing goal coordinates

void moveTo(double a, double b, double alpha)
{
    ROS_INFO("Move to (%lf, %lf, %lf)", a, b, alpha);
    cout << "Move" << a <<" "<< b;
    //geometry_msgs::PoseStamped goal;
    MoveBaseClient ac("move_base", true);
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
            ROS_INFO("Waiting for the move_base action server to come up");
        }


    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id="base_footprint";
    goal.target_pose.header.stamp=ros::Time::now();

    goal.target_pose.pose.position.x=b;
    goal.target_pose.pose.position.y=a;
    goal.target_pose.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(0, 0, alpha);
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
    ac.waitForResult(); 
    return;
    

}

// Storing messages globally for sequential execution

int flag1=0,flag2=0,flag3=0,flag4=0;

void goalCallback(const nav_msgs::OccupancyGrid &msg2)
{
    msgb=msg2;
    flag1=1;
    //std::cout<<endl<<"Map <-";

}

void callback( const sensor_msgs::ImageConstPtr &msg)
{
    msga=msg;
    flag2=1;
   // std::cout<<endl<<"Img <-";

}

void depthcallback( const sensor_msgs::ImageConstPtr &msg1)
{
    msgc=msg1;
    flag3=1;
   // std::cout<<endl<<"Depth <-";

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

int ctr=1,toggle=1;


void chatterCallback2 (const nav_msgs::OccupancyGrid point1 ) // Function for frontier exploration
{
                 
    toggle=1;     // Toggle set for fixed steps before going for exploration  

    move_base_msgs::MoveBaseGoal next_goal;
    
    int a2 = point1.info.width; 
    int b2 = point1.info.height;
    int c= (a2) * (b2); 
    int8_t arr[c];
    
    for(int i=0; i<c; i++)
    {
        arr[i]=point1.data[i];
    } 
    
    int arr2d[b2][a2];
    int count1 =0 ;
    
    for (int i=0; i<b2;i++)
    {       
        for (int j=0;j<a2;j++)
        {
                        
            arr2d[i][j]=arr[j+a2*i];
            
            if (arr2d[i][j]==0)
                {arr2d[i][j]=254;}
            
            if (arr2d[i][j]==-1)
                {arr2d[i][j]=205;}
            
            if (arr2d[i][j]==100)
                {arr2d[i][j]=0;}                                
                    
            if(arr2d[i][j] ==0)
            {               
 
            count1++;
            }                               
        } 
    }
    //cout<<" flag 2 "<<endl;
    cv::Mat occ1(b2,a2,CV_8U,cv::Scalar(0));
                    //occ=occ1.reshape(a1,b1);  
    
    for(int i=0; i<b2 ;i++)
    {
        for (int j=0;j<a2;j++)
        {
        occ1.at<uchar>((b2-i-1),(j))=arr2d[i][j];
                    //ROS_INFO("%d", occ.at<uchar>(i,j));       
        }               
    }
    //cout<<" flag 3 "<<endl;
    Mat occ(occ1.size(),CV_8U,cv::Scalar(0));
    
    int morph_elem = 0;
    int morph_size = 3;
    int morph_operator = 0;
    int operation=2;
    //const char* window_name = "Morphology Transformations ";

    //namedWindow( window_name, WINDOW_AUTOSIZE );

    Mat element = getStructuringElement( morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
    morphologyEx( occ1, occ, operation, element );          //Morphological transformation
    
    //imshow( window_name, occ );
    //imwrite("/home/sud/morph4.pgm",);
        

    Mat new_image(occ.size(),CV_8UC3,Scalar(0));
    int no_rows = new_image.rows;
        
    //*******************FRONTIER_SEARCH*****************************   
    int count=0;
    int k=0;
    int l=0;
    int a[SIZE];
    int b[SIZE];
    //cout<< "rows" << new_image.rows<< "columns" << new_image.cols<<endl;
    for(int i=0;i<new_image.rows;i++)
    {
        for( int j=0;j<new_image.cols;j++)
        {
            if (abs((occ.at<uchar>(i,j))-(occ.at<uchar>(i,j+1)))==49)
            {
                a[k]=i;
                b[l]=j;
                k++; l++;
                //int intensity=map.at<uchar>(i,j); 
                //cout<<"value of i and j and intensity value"<<i<<"  "<<j<<"  "<< intensity << std::endl ;
                count++;
                new_image.at<Vec3b>(i,j)[0]=255;
                new_image.at<Vec3b>(i,j)[1]=255;
                new_image.at<Vec3b>(i,j)[2]=255;
                                //cout<<endl<< " Count is :" << count<<endl ;
            }           
        }   
    }   
                //int mid_count=count;
    //cout<<" flag 4 "<<endl; 
    for(int j=0;j<new_image.cols;j++)
    {
        for( int i=0;i<new_image.rows;i++)
        {
            if (abs((occ.at<uchar>(i,j))-(occ.at<uchar>(i+1,j)))==49)
            {
                a[k]=i;
                b[l]=j;     
                k++; l++;
                            //int intensity =map.at<uchar>(i,j); 
                            //cout<<"value of i and j and intensity value"<<i<<"  "<<j<<"  "<< intensity << std::endl ;
                count++;
                new_image.at<Vec3b>(i,j)[0]=255;
                new_image.at<Vec3b>(i,j)[1]=255;
                new_image.at<Vec3b>(i,j)[2]=255;
            }           
        }   
    }
    //cout<<" flag 5 "<<endl;   
        
    //*************CLUSTERING****************
    
    Mat samples(count,2,CV_32FC3,Scalar(0));
    for (int i=0 ; i<count ; i++)
    {       
        samples.at<float>(i,0)=a[i];    
    }
    for (int j=0; j<count; j++)
    {
        samples.at<float>(j,1)=b[j];    
    }

    //cout<<" flag 6 "<<endl;
    int clustercount=NO_CLUSTERS;   
    Mat labels;
    int attempts=ATTEMPTS;
    Mat centers;        
    cv::kmeans(samples,clustercount,labels,TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,10000,0.0001),attempts,KMEANS_PP_CENTERS,centers);


    ROS_INFO("ORIGIN FROM LL CORNER %lf %lf %lf",point1.info.origin.position.x, point1.info.origin.position.y, point1.info.origin.position.z);

    origin_x=(point1.info.origin.position.x);
    origin_y=(point1.info.origin.position.y);
    //cout<<" flag 7 "<<"origin x "<< origin_x <<"origin y: " << origin_y <<endl;
    
    float origin_y_tl = -origin_x/point1.info.resolution;
    float origin_x_tl = no_rows + origin_y/point1.info.resolution;

    int d[NO_CLUSTERS];
    int e[NO_CLUSTERS];
    float dis[NO_CLUSTERS];
    int y_prev = (-origin_x-x_nex)/point1.info.resolution;
    int x_prev = (no_rows + origin_y/point1.info.resolution)-(y_nex/point1.info.resolution);
    
    for (int j=0; j<NO_CLUSTERS ;j++)
    {   
         d[j]= centers.at<float>(j,0);
         e[j]= centers.at<float>(j,1);
         dis[j]= sqrt((d[j]-x_prev)*(d[j]-x_prev) + (e[j]-y_prev)*(e[j]-y_prev));
        // cout<<"centers value " << d[j]<<" "<< e[j]<<" "<< dis[j]<<std::endl;     
    }
    //cout<<" flag 8 "<<endl;
        // cout<< "Origin are :" << a[mid_count]*0.05 << "and" << b[mid_count]*0.05 << endl;
    
    for (int i=0 ;i<NO_CLUSTERS;i++)            // 
    {
        d[i]=d[i]-((d[i]-y_prev)/dis[i])*10;
        e[i]=e[i]-((e[i]-x_prev)/dis[i])*10;
    }

    circle(new_image, cvPoint(y_prev,x_prev), 4, CV_RGB(255,255,0), 1,8,0);         // YELLOW : PREVIOUS POSITION
    
    circle(new_image, cvPoint(origin_y_tl,origin_x_tl), 2, CV_RGB(255,255,0), 1,8,0);
    circle(new_image, cvPoint(origin_y_tl,origin_x_tl), 4, CV_RGB(0,255,255), 1,8,0);
    circle(new_image, cvPoint(origin_y_tl,origin_x_tl), 6, CV_RGB(255,0,255), 1,8,0);

    //ROS_INFO ("MY POSITION FROM TL CORNER x-coor: %d y-coor: %d", y_prev,x_prev); 
    
    for (int k=0; k<NO_CLUSTERS;k++)                    //NEW ADDITION TO CODE: STILL TO BE TESTED
    {   
    if( (occ.at<uchar>(d[k],e[k])==205) || (occ.at<uchar>(d[k],e[k])==0) )  
        {
        dis[k]=100000;      
        for (int i=1; i<5; i++)
            for (int j=-i;j<=i;j++)
                for (int r=-i;r<=i;r++)
                    {
                    new_image.at<Vec3b>(d[k]+j,e[k]+r)[0]=255;
                    new_image.at<Vec3b>(d[k]+j,e[k]+r)[1]=255;
                    new_image.at<Vec3b>(d[k]+j,e[k]+r)[2]=255;
                    }
        }
    }
    
    for(int p=0; p<NO_CLUSTERS;p++)
        for (int i=1; i<5; i++)
            for (int j=-i;j<=i;j++)
                for (int k=-i;k<=i;k++)
                {
                if(occ.at<uchar>(d[p]+j,e[p]+k)!=254)
                {
                    for (int i=1; i<5; i++)
                        for (int j=-i;j<=i;j++)
                            for (int k=-i;k<=i;k++)
                                {
                                dis[p]=100000;
                                new_image.at<Vec3b>(d[p]+j,e[p]+k)[0]=255;
                                new_image.at<Vec3b>(d[p]+j,e[p]+k)[1]=255;
                                new_image.at<Vec3b>(d[p]+j,e[p]+k)[2]=255;
                                }       
                }
                }   
    
    //cout<<" flag 9 "<<endl;   
    int min_i;
    min_i=get_min(dis,NO_CLUSTERS);     

    y_nex= ((x_prev)-(d[min_i]))*point1.info.resolution;//point1.info.resolution;
    //cout<<" flag 9.2 "<<endl;     
    x_nex= ((y_prev)-(e[min_i]))*point1.info.resolution;//point1.info.resolution;
     
    float x_next=(e[min_i]-origin_y_tl)*point1.info.resolution;   //FOR DIRECT MAP FRAME
    float y_next= (-(d[min_i]-origin_x_tl))*point1.info.resolution;

    int c0=0;                               //NEW ADDITION TO CODE : TO BE TESTED
    int c1=0;
    int c2=0;
    int c3=0;
    int c4=0;
    int c5=0;
    int c6=0;
    int c7=0;
    int c8=0;
    int c9=0;
    
    for (int i=0;i<count;i++)       
        {       
            int c= labels.at<int>(i,0);
            //float d= centers.at<int>(1,0);
            //float e= centers.at<int>(1,1);
            //cout<<"labels value " << c <<std::endl;               
            
            if(c==0)
            {
            c0++;       
            new_image.at<Vec3b>(a[i],b[i])[0]=255;
            new_image.at<Vec3b>(a[i],b[i])[1]=255;
            new_image.at<Vec3b>(a[i],b[i])[2]=255;          
                if( (occ.at<uchar>(d[0],e[0])!=205) && (occ.at<uchar>(d[0],e[0])!=0) )  
                { 
                circle(new_image, cvPoint(e[0],d[0]), 2, CV_RGB(255,255,255), 2,8,0); 
                }           
            }

            if(c==1) //RED
            {
            c1++;   
            new_image.at<Vec3b>(a[i],b[i])[0]=0; 
            new_image.at<Vec3b>(a[i],b[i])[1]=0;
            new_image.at<Vec3b>(a[i],b[i])[2]=255;      
            if ((occ.at<uchar>(d[1],e[1])!=205) && (occ.at<uchar>(d[1],e[1])!=0))
                { 
                circle(new_image, cvPoint(e[1],d[1]), 2, CV_RGB(255,0,0), 2,8,0); 
                }           
            }
            
            if(c==2) //ORANGE
            {
            c2++;           
            new_image.at<Vec3b>(a[i],b[i])[0]=0;
            new_image.at<Vec3b>(a[i],b[i])[1]=127;
            new_image.at<Vec3b>(a[i],b[i])[2]=255;
            if((occ.at<uchar>(d[2],e[2])!=205) && (occ.at<uchar>(d[2],e[2])!=0))    
                { 
                circle(new_image, cvPoint(e[2],d[2]), 2, CV_RGB(255,127,0), 2,8,0); 
                }
            }

            if(c==3) //YELLOW
            {
            c3++;           
            new_image.at<Vec3b>(a[i],b[i])[0]=0;
            new_image.at<Vec3b>(a[i],b[i])[1]=255;
            new_image.at<Vec3b>(a[i],b[i])[2]=255;      
            if((occ.at<uchar>(d[3],e[3])!=205) && (occ.at<uchar>(d[3],e[3])!=0))    
                { 
                circle(new_image, cvPoint(e[3],d[3]), 2, CV_RGB(255,255,0), 2,8,0); 
                }           
            }
    
            if(c==4) //GREEN
            {
            c4++;           
            new_image.at<Vec3b>(a[i],b[i])[0]=0;
            new_image.at<Vec3b>(a[i],b[i])[1]=255;
            new_image.at<Vec3b>(a[i],b[i])[2]=0;            

            if((occ.at<uchar>(d[4],e[4])!=205)&& (occ.at<uchar>(d[4],e[4])!=0)) 
                { 
                circle(new_image, cvPoint(e[4],d[4]), 2, CV_RGB(0,255,0), 2,8,0);  
                }           
            }
            
            if(c==5) //BLUE
            {
            c5++;           
            new_image.at<Vec3b>(a[i],b[i])[0]=255;
            new_image.at<Vec3b>(a[i],b[i])[1]=0;
            new_image.at<Vec3b>(a[i],b[i])[2]=0;
            if((occ.at<uchar>(d[5],e[5])!=205) && (occ.at<uchar>(d[5],e[5])!=0))    
                { 
                circle(new_image, cvPoint(e[5],d[5]), 2, CV_RGB(0,0,255), 2,8,0); 
                }           
            }

            if(c==6) //INDIGO
            {
            c6++;           
            new_image.at<Vec3b>(a[i],b[i])[0]=130;
            new_image.at<Vec3b>(a[i],b[i])[1]=0;
            new_image.at<Vec3b>(a[i],b[i])[2]=75;
            if((occ.at<uchar>(d[6],e[6])!=205) && (occ.at<uchar>(d[6],e[6])!=0))    
                { 
                circle(new_image, cvPoint(e[6],d[6]), 2, CV_RGB(75,0,130), 2,8,0); 
                }           
            }

            if(c==7) //VIOLET
            {
            c7++;           
            new_image.at<Vec3b>(a[i],b[i])[0]=255;
            new_image.at<Vec3b>(a[i],b[i])[1]=150;
            new_image.at<Vec3b>(a[i],b[i])[2]=143;
            if((occ.at<uchar>(d[7],e[7])!=205) && (occ.at<uchar>(d[7],e[7])!=0))    
                { 
                circle(new_image, cvPoint(e[7],d[7]), 2, CV_RGB(143,150,255), 2,8,0); 
                }           
            }
            
            if(c==8)
            {
            c8++;           
            new_image.at<Vec3b>(a[i],b[i])[0]=55;
            new_image.at<Vec3b>(a[i],b[i])[1]=55;
            new_image.at<Vec3b>(a[i],b[i])[2]=200;
            if((occ.at<uchar>(d[8],e[8])!=205) && (occ.at<uchar>(d[8],e[8])!=0))    
                { 
                circle(new_image, cvPoint(e[8],d[8]), 2, CV_RGB(200,55,55), 2,8,0); 
                }           
            }   
            
            if(c==9)
            {
            c9++;           
            new_image.at<Vec3b>(a[i],b[i])[0]=200;
            new_image.at<Vec3b>(a[i],b[i])[1]=200;
            new_image.at<Vec3b>(a[i],b[i])[2]=200;
            if((occ.at<uchar>(d[9],e[9])!=205) && (occ.at<uchar>(d[9],e[9])!=0))    
                { 
                circle(new_image, cvPoint(e[9],d[9]), 2, CV_RGB(200,200,200), 2,8,0); 
                }           
            }   
                
        }
    //cout<<" flag 11 "<<endl;

    //cout <<"c0 :"<< c0<<" c1 :"<< c1<<" c2 :"<< c2<<" c3 :"<< c3<<" c4 :"<< c4<<" c5 :"<< c5<<" c6 :"<< c6<<" c7 : "<< c7 <<" c8 :"<< c8<<"c9 :"<< c9 <<endl; 
        
    circle(new_image, cvPoint(e[min_i],d[min_i]), 8, CV_RGB(0,255,0), 1,8,0);       //GREEN : GOAL / NEAREST CLUSTER
        
    //***************************************
    

    cv::imwrite("occ.jpg",occ);

    //cv::namedWindow("blank2");
    cv::imwrite("occ1.jpg",occ1);
    
//  String filename="/workspace/karthik/RRC/ros_workspace/frontier_explore/maps/result"+z+".jpg";
    char filename[100]="result";
//  filename=filename.c_str();
    char extension[10] = ".jpg";

    sprintf(filename,"%s%d%s",filename,z,extension);

    z++;
    //************** PUBLISHING ****************
    
    //cout<<" flag 13 "<<endl;
    
    if (x_last==x_next)
    {
    next_goal.target_pose.header.frame_id="/map";
    next_goal.target_pose.pose.position.x=0.0;
    next_goal.target_pose.pose.position.y=0.0;
    next_goal.target_pose.pose.position.z=0.0;
    next_goal.target_pose.pose.orientation.x=0.0;
    next_goal.target_pose.pose.orientation.y=0.0;
    next_goal.target_pose.pose.orientation.z=0.0;
    next_goal.target_pose.pose.orientation.w=1.0;   
    }
    else
    {
    next_goal.target_pose.header.frame_id="/map";
    next_goal.target_pose.pose.position.x=x_next;
    next_goal.target_pose.pose.position.y=y_next;
    next_goal.target_pose.pose.position.z=0.0;
    next_goal.target_pose.pose.orientation.x=0.0;
    next_goal.target_pose.pose.orientation.y=0.0;
    next_goal.target_pose.pose.orientation.z=0.0;
    next_goal.target_pose.pose.orientation.w=1.0;       
    }
    ROS_INFO(" NEXT GOAL x:%lf y:%lf ",next_goal.target_pose.pose.position.x,next_goal.target_pose.pose.position.y);
    MoveBaseClient ac("move_base", true);
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
            ROS_INFO("Waiting for the move_base action server to come up");
        }
    ROS_INFO("Sending goal");
    ac.sendGoal(next_goal);
    ac.waitForResult();
    x_last = x_next;
    y_last = y_next;
    //cout<<" flag sleep now "<<endl;   
    // sleep(30);
    //waitKey(2000);  
     return;   

}



int loop1=0,loop2=0;
int depth=9999;
int rot=1;
actionlib_msgs::GoalStatus goalStatus;
int prn=1;
int pr=1;


// Parent Function

void callit(const sensor_msgs::ImageConstPtr msg3, const nav_msgs::OccupancyGrid msg4, const sensor_msgs::ImageConstPtr& msg5)
{  
    namedWindow("segmented",WINDOW_NORMAL);

    flag2=0; //reset flags 
    flag3=0; 


    loop1++;

    if(explore_var){ // Frontier exploration called


        explore_var=0;
        toggle=1;      
        chatterCallback2(msg4);   
        return;
    }


    if(loop1>1)
    {   depth = ReadDepthData(240,320, msg5); // Calculate depth of objects in front of turtlebot using Depthstream
       // cout<<endl<<"depth"<<depth<<endl;
    }
    if(depth==-1)
        depth=9999;

    Mat src_hsv;
    src_hsv=cv_bridge::toCvShare(msg3,"bgr8")->image;
    cvtColor(src_hsv, src_hsv, COLOR_BGR2HSV);


    if(imwrite("1.jpg", cv_bridge::toCvShare(msg3, "bgr8")->image))
        waitKey(10);
    if(imwrite("2.jpg", cv_bridge::toCvShare(msg3, "bgr8")->image))
        waitKey(10);
    system("convert 1.jpg 1.pgm");
    system("convert 2.jpg 2.pgm");


    system("./run");

    Mat seg;
    seg=imread("1.jpg");
    

    //std::cout<<endl<<"Returned"<<endl;

    Mat src; Mat src_gray,src2,src3,src4;
    int thresh = 255;
    int max_thresh = 255;
    RNG rng(12345);


    src=imread("blob.jpg");
    cvtColor( src, src_gray, CV_BGR2GRAY );
    //blur( src_gray, src_gray, Size(3,3) );
    cout<<endl<<"Blob image generated"<<endl;


    Mat canny_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;



    Canny( src_gray, canny_output, thresh, thresh*2, 3 );

    findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    //     return;

    if(!contours.size()){
        explore_var=1;
        return;
    }


    vector<Moments> mu(contours.size() );
    vector <Point> min(contours.size());
    vector<Point> mc1(contours.size()); 

    for( int i = 0; i < contours.size(); i++ )
    {
        min[i].x=-1;
        min[i].y=-1;
        mu[i] = moments( contours[i], false );
        mc1.push_back(Point(mu[i].m10/mu[i].m00,mu[i].m01/mu[i].m00));
        for(int j=0;j< contours[i].size();j++){
            if(contours[i][j].y>min[i].y){
                min[i].y=contours[i][j].y;
                min[i].x=contours[i][j].x;
            }
        }
        
    }



    int counter=0;
    int pp=1;
    int xp,yp;

    std::vector<Point> mc;

    for( int i = 0; i < contours.size(); i++ )
    {
        
        if(depth>1500)
            if(contourArea(contours[i])>70 )
               if(src_gray.at<uchar>((min[i].y-1),min[i].x)==0){

                mc.push_back(Point(min[i].x, min[i].y) );
                //cout<<"x"<<" "<<mc[i].x<<"y"<<" "<<mc[i].y<<endl;
                counter++;
                // break;
               
                
            }
    }




    if(mc.size()==0){ //no objects found on the floor

        //explore_var=1;
        cout<<endl<<"Returning as no object found";

         if(!toggle){
            explore_var=1;
            return;

        }

        switch (rot) { //Perform some fixed rotations to check for objects before going for frontier exploration
            case 1:
                moveTo(0.0,0.0,-1.0); //rotate 60 deg clockwise
                break;
            case 2:
                moveTo(0.0,1.0,2.0); //rotate 120 deg anticlockwise
                break;


        }


        rot = (rot+1)%3;

        if(!rot){
            toggle=0;
            rot=1;
        }
        
        return;

        
        if(depth<1500){ // 1500 mm
            cout<<endl<<"Wall/obstacle in front : Aborting and going for exploration:: depth = "<<depth;
            return;
        }
        

    }

    
    for (int i = 0; i < mc1.size(); i++)
    {
        //Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        if(src_gray.at<uchar>(mc1[i].y,mc1[i].x)==0){
           // cout<<" v: "<<src_gray.at<uchar>(mc1[i].y,mc1[i].x);
            circle(src,mc1[i],5,Scalar(50,50,50),-1,8,0);

        }
        
    }


    for (int i = 0; i < mc.size(); i++)
    {
        //Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       circle( src, mc[i], 10, Scalar(100,200,200), -1, 8, 0 );
       //cout<<endl<<"MC x "<<mc[i].x<<" MC y "<<mc[i].y<<endl;        
    }


    imshow("final",src);
    waitKey(100);
    imwrite("final.jpg",src);
    waitKey(100);
    // imshow("segmented",seg);
    // waitKey(100);
    // imwrite("segmented_recg.jpg",seg);
    // waitKey(100);


    // for( int i = 0; i< contours.size(); i++ )
    // {
    //     Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
    //     drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
    //     circle( drawing, mc[i], 10, color, -1, 8, 0 );
    // }


    
    //namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
    // imwrite( "Contours.jpg", drawing );
    // waitKey(100);

    // Sending goals if objects detected on floor
    vector<Point2f>goal;
    for(int k=0;k<counter;k++){
        cout<<endl<<mc[k].x<<" "<<mc[k].y;
    }
    cout<<endl;

    int counter1=0;

    for (int k=0;k<counter;k++) // IMPORTANT ** Calibration required ** Pinhole formula used to estimate object distance 
    {                           // from kinect (turtlebot) based on Focal lenght = 530mm and Height of camera from ground = 0.33 m
        float obj_height=mc[k].y-240;
        float z1=focal*heightofcam/obj_height;
        z1 = z1*z1*z1*0.4025 - z1*z1*0.7903 + z1*2.4968 - 0.3587; // Function for depth estimation * Compute your own
        float x1=(320-mc[k].x)*z1/focal;

        if(mc[k].y!=0){ // eliminating redundant cordinates

            if(k==0){

                cout<<endl<<"Depth "<<z1<<endl<<"X "<<x1<<endl;
                goal.push_back(Point2f(x1,z1));
                counter1++;
            }
        else
            if(mc[k].y!=mc[k-1].y){
                cout<<endl<<"Depth "<<z1<<endl<<"X "<<x1<<endl;
                goal.push_back(Point2f(x1,z1));
                counter1++;
            }
        }
        else{
            explore_var=1;
            cout<<endl<<"Objects found but not relevant"<<endl;
            return;
        }

    
    }    

    
    for(int a=0;a<counter1;a++)
        {

            if (a==0)
            {
               ROS_INFO("Object found: Get Goal : %d",a);
               double beta=atan2(goal[a].x,goal[a].y);
               double x_inner=(goal[a].x);
               double y_inner=(goal[a].y);
               moveTo(x_inner, y_inner, beta);
               waitKey(3000);

            }
            else
            {
               ROS_INFO("Object found: Get Goal : %d",a);
               double xa,ya,tempang;
               tempang=atan2(goal[a-1].x,goal[a-1].y);
               xa=goal[a].x-goal[a-1].x;ya=goal[a].y-goal[a-1].y;
               double beta=atan2(xa*cos(tempang)-ya*sin(tempang),xa*sin(tempang)+ya*cos(tempang));
               double x_inner=xa*cos(tempang)-ya*sin(tempang);
               double y_inner=xa*sin(tempang)+ya*cos(tempang);
               moveTo(x_inner, y_inner, beta);
               waitKey(3000);
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
    image_transport::Subscriber sub = it.subscribe("camera/depth/image", 1, depthcallback);
    ros::Subscriber sub3=n1.subscribe("map",1,goalCallback); 


    while(ros::ok())
    {
        ros::spinOnce();

        if(flag1&&flag2&&flag3)
            callit(msga,msgb,msgc);

        ros::Rate r(30);
        r.sleep();
    }

    return 0;

}
