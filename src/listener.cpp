#include "ros/ros.h"
//#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"

//The resolution of the output image
#define IMAGE_WIDTH 480
#define IMAGE_HEIGHT 200
#define HALF_WIDTH 240
#define HALF_HEIGHT 100
//1/tan(.5 * FOV)
#define VERTICAL_N 2.146
#define HORIZONTAL_N 2.9

ros::Publisher pub;

void callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  
  //Store the amount of points received
  uint32_t length = msg->width;
  
  //Create a buffer for the output image
  uint8_t outputImage[IMAGE_WIDTH*IMAGE_HEIGHT];
  //Clear the buffer to erase random data
  for (uint16_t xi = 0; xi < IMAGE_WIDTH; xi++) {
    for (uint16_t yi = 0; yi < IMAGE_HEIGHT; yi++) {
      outputImage[xi + yi * IMAGE_WIDTH] = 0;
    }
  }
  
  //Loop through each received point
  for (uint32_t i = 0; i < length; i++) {
    //If its x value is below zero the point is not relevant to us
    float x = *((float *) &(msg->data[i * msg->point_step]));
    if (x < 0) {
      continue;
    }
    //Check if the x value is smaller than the y value, if it is the point is not interesting
    //This leaves us with a 90 degree fov with the x axis in the middle
    float y = *((float *) &(msg->data[i * msg->point_step + 4]));
    if (x < abs(y)) {
      continue;
    }
    //The point is valid
    
    //Calculate the normalized vertical screen coordinate (from -1 to 1)
    float normalScreenX = -(VERTICAL_N / x) * y;
    //If the pixel is off the screen don't bother
    if (normalScreenX < -1 || normalScreenX > 1) {
      continue;
    }
    
    //Calculate the normalized horizontal screen coordinate (from -1 to 1)
    float z = *((float *) &(msg->data[i * msg->point_step + 8]));
    float normalScreenY = -(HORIZONTAL_N / x) * z;
    if (normalScreenY < -1 || normalScreenY > 1) {
      continue;
    }
    
    //Calculate the pixel the point falls within
    uint16_t pixelX = normalScreenX * HALF_WIDTH + HALF_WIDTH;
    uint16_t pixelY = normalScreenY * HALF_HEIGHT + HALF_HEIGHT;
    
    //Calculate the distance from the point to the camera
    float dist = sqrt(x * x + y * y + z * z);
    
    //TODO: Create a better way of displaying the pixels
    //Color the output pixel according to the distance
    //outputImage[pixelX + pixelY * IMAGE_WIDTH] = dist * 50;
    //Color an area surrounding the pixel (temporarely, for better viewing)
    for (int8_t offX = -1; offX <= 1; offX++) {
      for (int8_t offY = -5; offY <= 5; offY++) {
        outputImage[pixelX + offX + (pixelY + offY) * IMAGE_WIDTH] = dist * 50;
      }
    }
    
  }
  
  //TODO: Do some processing to create a better image
  
  //Construct and publish the image
  sensor_msgs::Image outMsg;
  //TODO: Create a custom header
  outMsg.header = std_msgs::Header();
  outMsg.height = (IMAGE_HEIGHT);
  outMsg.width = (IMAGE_WIDTH);
  outMsg.encoding = "mono8";
  //TODO: Check the endianess
  outMsg.is_bigendian = false;
  outMsg.step = (IMAGE_WIDTH);
  //TODO: Optimize the following statement
  std::vector<uint8_t> output(outputImage, outputImage + IMAGE_WIDTH * IMAGE_HEIGHT);
  outMsg.data = output;
  pub.publish(outMsg);
  
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "listener");

  ros::NodeHandle nh;
  
  pub = nh.advertise<sensor_msgs::Image>("depth_image", 10);

  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("velodyne_points", 1, callback);

  ros::spin();

  return 0;
  
}
