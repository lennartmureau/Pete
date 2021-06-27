#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"

//The resolution of the output image (one fifth full HD (1920x1080))
#define IMAGE_WIDTH 384
#define IMAGE_HEIGHT 216
#define HALF_WIDTH 192
#define HALF_HEIGHT 108
//The N values used for calculating the fov (N = 1/tan(.5 * FOV))
#define HORIZONTAL_N 1.376
#define VERTICAL_N 2.475
//The multiplier for the distance value (max distance = 255 / MULTIPLIER)
#define MULTIPLIER 50
//The size off the kernel for the final processing (in one direction)
#define KERNEL_N 7
//The frame from which the image is published
#define PUBLISHING_FRAME "depth_frame"

ros::Publisher pub;

void callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  
  //Create an array to store the found points
  uint8_t pointImage[IMAGE_WIDTH * IMAGE_HEIGHT];
  //Clear the buffer to erase random data
  for (uint16_t pixelX = 0; pixelX < IMAGE_WIDTH; pixelX++) {
    for (uint16_t pixelY = 0; pixelY < IMAGE_HEIGHT; pixelY++) {
      pointImage[pixelX + pixelY * IMAGE_WIDTH] = 0;
    }
  }
  
  //Store the amount of points received
  uint32_t length = msg->width;
  
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
    float normalScreenX = -(HORIZONTAL_N / x) * y;
    //If the pixel is off the screen don't bother
    if (normalScreenX < -1 || normalScreenX > 1) {
      continue;
    }
    
    //Calculate the normalized horizontal screen coordinate (from -1 to 1)
    float z = *((float *) &(msg->data[i * msg->point_step + 8]));
    float normalScreenY = -(VERTICAL_N / x) * z;
    if (normalScreenY < -1 || normalScreenY > 1) {
      continue;
    }
    
    //Calculate the pixel the point falls within
    uint16_t pixelX = normalScreenX * HALF_WIDTH + HALF_WIDTH;
    uint16_t pixelY = normalScreenY * HALF_HEIGHT + HALF_HEIGHT;
    
    //Calculate the distance from the point to the camera
    float dist = sqrt(x * x + y * y + z * z);

    //Color the output pixel according to the distance
    pointImage[pixelX + pixelY * IMAGE_WIDTH] = std::min(int(dist * MULTIPLIER), 255);
  }
  
  //Create the output image array
  uint8_t outputImage[IMAGE_WIDTH * IMAGE_HEIGHT];
  
  //This algorithm uses inverse distance weighting for the final image
  /*
  //Loop through all the pixels off the output image
  for (uint16_t pixelX = 0; pixelX < IMAGE_WIDTH; pixelX++) {
    for (uint16_t pixelY = 0; pixelY < IMAGE_HEIGHT; pixelY++) {
      //If the image is already colored, just copy it over
      if (pointImage[pixelX + pixelY * IMAGE_WIDTH] != 0 ) {
        outputImage[pixelX + pixelY * IMAGE_WIDTH] = pointImage[pixelX + pixelY * IMAGE_WIDTH];
        continue;
      }
      //Create the top and bottom half of the division for inverse distance weighting
      float top = 0.0;
      float bottom = 0.0;
      //Go through each of the surrounding pixels
      for (int8_t offX = -KERNEL_N; offX <= KERNEL_N; offX++) {
        for (int8_t offY = -KERNEL_N; offY <= KERNEL_N; offY++) {
          //If the pixel is offscreen, don't bother
          if (pixelX + offX >= IMAGE_WIDTH || pixelX + offX < 0 || pixelY + offY >= IMAGE_HEIGHT || pixelY + offY < 0) {
            continue;
          }
          //Only use pixels that are already colored
          if (pointImage[pixelX + offX + (pixelY + offY) * IMAGE_WIDTH] != 0) {
            float dist = (offX * offX + offY * offY);
            top += pointImage[pixelX + offX + (pixelY + offY) * IMAGE_WIDTH] / dist;
            bottom += 1 / dist;
          }
        }
      }
      //Give the pixel the correct color
      outputImage[pixelX + pixelY * IMAGE_WIDTH] = top / bottom;
    }
  }
  */
  
  //This algorithm uses nearest neighbour for the final image
  
  //Loop through all the pixels off the output image
  for (uint16_t pixelX = 0; pixelX < IMAGE_WIDTH; pixelX++) {
    for (uint16_t pixelY = 0; pixelY < IMAGE_HEIGHT; pixelY++) {
      //If the image is already colored, just copy it over
      if (pointImage[pixelX + pixelY * IMAGE_WIDTH] != 0 ) {
        outputImage[pixelX + pixelY * IMAGE_WIDTH] = pointImage[pixelX + pixelY * IMAGE_WIDTH];
        continue;
      }
      //
      uint16_t minDistance = KERNEL_N * KERNEL_N;
      uint8_t color = 0;
      for (int8_t offX = -KERNEL_N; offX <= KERNEL_N; offX++) {
        for (int8_t offY = -KERNEL_N; offY <= KERNEL_N; offY++) {
          //If the pixel is offscreen, don't bother
          if (pixelX + offX >= IMAGE_WIDTH || pixelX + offX < 0 || pixelY + offY >= IMAGE_HEIGHT || pixelY + offY < 0) {
            continue;
          }
          if (pointImage[pixelX + offX + (pixelY + offY) * IMAGE_WIDTH] != 0 && (offX * offX + offY * offY) < minDistance) {
            minDistance = (offX * offX + offY * offY);
            color = pointImage[pixelX + offX + (pixelY + offY) * IMAGE_WIDTH];
          }
        }
      }
      //Give the pixel the correct color
      outputImage[pixelX + pixelY * IMAGE_WIDTH] = color;
    }
  }
  
  //Construct and publish the image
  sensor_msgs::Image outMsg;
  //TODO: Add the sequence to the header
  outMsg.header.stamp = ros::Time::now();
  outMsg.header.frame_id = PUBLISHING_FRAME;
  outMsg.height = (IMAGE_HEIGHT);
  outMsg.width = (IMAGE_WIDTH);
  outMsg.encoding = "mono8";
  outMsg.is_bigendian = false;
  outMsg.step = (IMAGE_WIDTH);
  std::vector<uint8_t> output(outputImage, outputImage + IMAGE_WIDTH * IMAGE_HEIGHT);
  outMsg.data = output;
  pub.publish(outMsg);
  
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "listener");

  ros::NodeHandle nh;
  
  pub = nh.advertise<sensor_msgs::Image>("depth_image8", 10);

  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("velodyne_points", 1, callback);

  ros::spin();

  return 0;
  
}
