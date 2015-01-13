#include <iostream>
#include <string.h>
#include <fstream>
#include <sstream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
//#include <pcl/visualization/pcl_visualizer.h>
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

using namespace std;

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<pcl::PointXYZRGB> cloudxyz;
typedef cloudxyz::Ptr cloudptr;
typedef pcl::search::KdTree<Point>::Ptr KdTreePtr;

cloudxyz PC;
string object_ID;
int k=0;

void writeCloudPCD(cloudptr &cloud, string S)
{
	pcl::io::savePCDFileASCII(S, *cloud);	
}

cv::Mat image(640,480,CV_8UC3,cv::Scalar(0));


void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	cloudptr cloud_in (new cloudxyz());
	pcl::fromROSMsg(*msg, *cloud_in);
	
	cloudptr cloud (new cloudxyz());

	pcl::PassThrough<Point> pass_;
	pass_.setFilterFieldName("z");
	pass_.setFilterLimits(0.0, 3.0);
	pass_.setInputCloud(cloud_in);
	pass_.filter(*cloud);

	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_filtered=cloud;	
	
	// Create the segmentation object for the planar model and set all the parameters
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::PCDWriter writer;
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setAxis(Eigen::Vector3f(0,-1,0));
        seg.setEpsAngle(  40.0f * (M_PI/180.0f) );
	seg.setMaxIterations (100);
	seg.setDistanceThreshold (0.02);

	
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud (cloud_filtered);
		seg.segment (*inliers, *coefficients);
		
		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZRGB> extract;
		extract.setInputCloud (cloud_filtered);
		extract.setIndices (inliers);
		extract.setNegative (false);

		// Write the planar inliers to disk
		extract.filter (*cloud_plane);
		//std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

		// Remove the planar inliers, extract the rest
		extract.setNegative (true);
		extract.filter (*cloud_f);
		cloud_filtered = cloud_f;
	

	//Remove the outliers
	cloudptr object (new cloudxyz());
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud (cloud_filtered);
	sor.setMeanK(50);
	sor.setStddevMulThresh (1.0);
	sor.filter(*object);

	//vfh estimation of the each object cluster

	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr normals_tree_ (new pcl::search::KdTree<pcl::PointXYZRGB> ());
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_;
	normal_.setKSearch(10);
	normal_.setSearchMethod(normals_tree_);
	normal_.setInputCloud(object);
	normal_.compute(*normals);
	//		cout<<"Normals for vfh is computed"<<endl;
		cout<<"The vfh signature are computed"<<endl;

	cv::Mat image_1(640,480,CV_8UC3,cv::Scalar(0));
	pcl::PointCloud<pcl::PointXYZRGB> cloud_1;
	
	cout<<cloud_plane->points.size()<<endl;	
	cv::Mat occ(480,640,CV_8UC1,cv::Scalar(0));

	float x=0,y=0,z=0;
	float image_x=0; 
	float image_y=0;
	float focus=525;
	float cx=319.5;
	float cy=239.5;
	float f=5000;
	cout<<"hi "<<cloud_plane->points.size();
	for(int i=1; i<cloud_plane->points.size(); i++)
	{
		x=cloud_plane->points[i].x;
		y=cloud_plane->points[i].y;
		z=cloud_plane->points[i].z;
		//cout << " 	"<< x<<"	"<<y<<"		"<<z<<endl;
		image_x=x/z*focus;
		image_y=y/z*focus;
		//cout <<"image_x:	"<<image_x<<"	image_y:	"<<image_y<<endl;	
		image_x=cx+image_x;	
		image_y=cy+image_y;	

		occ.at<uchar>(image_y,image_x)=255;
		

	}	
	if(cloud_plane->points.size () > 40000)
	{
	cv::imwrite("gg.jpg",occ);
	}
	
	//stringstream num;
	//num<<k;
	//string path = "/home/sud/fuerte_work/sandbox/obj_segment/bottle/"+object_ID+"/"+object_ID+"_orientation"+num.str()+".pcd";
	//writeCloudPCD(cloud_plane, path);
	//string path2 = "/home/sud/fuerte_work/sandbox/obj_segment/bottle/"+object_ID+"/"+object_ID+"_orientation_"+num.str()+".pcd";
	//writeCloudPCD(cloud_in, path2);

	//static void pcl::io::OrganizedConversion< PointT, false >::convert (cloud_plane, 525, 0 , 0, )
	//cout<<"Done writng the object cloud"<<endl;
	
	
	//cout<<"Done creating the vfh feature for object "<<endl;
	
	//cout<<object->width <<" " << object->height<<" "<< object->points.size() <<endl;
	
	/* for (size_t i = 0; i < object->points.size (); ++i)
	{   	 
		cout << "  " << object->points[i].x<<"\t";
              	cout<< " " << object->points[i].y<<"\t";
             	cout<< " "<< object->points[i].z<<endl; ;	
	}*/	
	k++;
	
	sleep(5);
	cout<<"sleeping";
}	



int main(int argc, char **argv)
{
	ros::init(argc, argv, "obj_segment");
	// if(argc<2)
	// {
	// 	cout<<"Enter the ID of the object"<<endl;
	// 	exit(0);

	// }

	// object_ID =argv[1];
	
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("camera/depth_registered/points", 1, callback);
	//pub1 = nh.advertise<sensor_msgs::PointCloud2> ("/camera/depth/objects", 1);
	//pub2 = nh.advertise<sensor_msgs::PointCloud2> ("/camera/depth/passthrough", 1);
	
	ros::spin();
}
