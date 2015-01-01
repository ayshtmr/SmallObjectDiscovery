#include <image_transport/image_transport.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <ros/callback_queue.h>

int count=0;

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

typedef union U_FloatParse {
    float float_data;
    unsigned char byte_data[4];
} U_FloatConvert;

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
            // if (depth_data.float_data == depth_data.float_data)
            //     return int(depth_data.float_data*1000);
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
  // if (temp_val == temp_val)
       return temp_val;
   return -1;  // If depth data invalid
}

// Image Callback
void imageCallback(const sensor_msgs::ImageConstPtr& image) {
    int depth = ReadDepthData(100, 320, image); // Width = 640, Height = 480
    ROS_INFO("Depth: %d", depth);
    ROS_INFO("Pixels: %d %d", 100*depth/580, 320*depth/580 );
}

void imageCallback1(const sensor_msgs::ImageConstPtr& image) {
    // Width = 640, Height = 480
    ROS_INFO("yoyo");
    
}

//*** Main ***//
int main(int argc, char **argv)
{
    ros::init(argc, argv, "imagegrab");
    ros::NodeHandle n,n2;
    printf("READY to get image\n");
    system("./run");


   //  n2.setCallbackQueue(imageCallback1);

   //  ros::Subscriber sub = n.subscribe("/camera/depth/image_raw", 1, chatterCallback);

   //  image_transport::ImageTransport it(n);
   //  image_transport::ImageTransport it1(n2);
    
   // // image_transport::Subscriber sub = it.subscribe("/camera/depth/image_raw", 1, imageCallback);
    
   //  //image_transport::Subscriber sub1 = it1.subscribe("/camera/depth/image_raw", 1, imageCallback1);

   //  ros::Rate r(10);

   //  while(ros::ok()){

   //    ros::spinOnce();
   //    r.sleep();
   //  }
    return 0;
}
