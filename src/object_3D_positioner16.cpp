#include "ros/ros.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "sensor_msgs/Image.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "iostream"
#include "string"

//The resolution of the output image (one fifth full HD (1920x1080))
#define IMAGE_WIDTH 384
#define HALF_WIDTH 192
#define IMAGE_HEIGHT 216
#define HALF_HEIGHT 108
//The N values used for calculating the fov (N = 1/tan(.5 * FOV))
#define HORIZONTAL_N 1.376
#define HORIZONTAL_N_SQUARE 1.893
#define VERTICAL_N 2.475
#define VERTICAL_N_SQUARE 6.126
//The multiplier for the distance value (max distance = 65535 / MULTIPLIER)
#define MULTIPLIER 600

void callback(const sensor_msgs::ImageConstPtr& image, const darknet_ros_msgs::BoundingBoxesConstPtr& boundingBoxes) {

  for (int i; i < sizeof(*image); i++) {

    double probability = boundingBoxes->bounding_boxes[i].probability;
    int64_t xmin = boundingBoxes->bounding_boxes[i].xmin;
    int64_t ymin = boundingBoxes->bounding_boxes[i].ymin;
    int64_t xmax = boundingBoxes->bounding_boxes[i].xmax;
    int64_t ymax = boundingBoxes->bounding_boxes[i].ymax;
    int16_t id = boundingBoxes->bounding_boxes[i].id;
    std::string Class = boundingBoxes->bounding_boxes[i].Class;
  
    int16_t xmiddle = (xmin + xmax) / 2;
    int16_t ymiddle = (ymin + ymax) / 2;
  
    uint8_t depth_bits01to08 = (image->data[xmiddle + ymiddle * IMAGE_WIDTH]) / MULTIPLIER;
    uint8_t depth_bits09to16 = (image->data[xmiddle + ymiddle * IMAGE_WIDTH]) / MULTIPLIER;
    uint16_t depth = depth_bits01to08 || (depth_bits09to16 >> 8);

    float x = std::sqrt(
                         (
                           std::pow(depth, 2)
                         )
                         /
                         (
                           1 
                           +
                           ( 1 / ((float) HORIZONTAL_N_SQUARE) )
                           +
                           (
                             (  ((float) xmiddle)  /  ( ((float) HORIZONTAL_N_SQUARE) * ((float) HALF_WIDTH) )  )
                             *
                             ( -2 + ((float) xmiddle) / ((float) HALF_WIDTH) )
                           )
                           +
                           ( 1 / ((float) VERTICAL_N_SQUARE) )
                           +
                           (
                             (  ((float) ymiddle)  /  ( ((float) VERTICAL_N_SQUARE) * ((float) HALF_HEIGHT) )  ) 
                             * 
                             ( -2 + ((float) ymiddle) / ((float) HALF_HEIGHT) )
                           )
                         )
                       );
    
    float y = (
                ( 1 - ((float) xmiddle) / ((float) HALF_WIDTH) )
                / 
                ( ((float) HORIZONTAL_N) / x )
              );

    float z = (
                ( 1 - ((float) xmiddle) / ((float) HALF_WIDTH) )
                /
                ( ((float) VERTICAL_N) / x )
              );
  
    std::cout << "object id: " << id << std::endl;
    std::cout << "object class: " << Class << std::endl;  
    std::cout << "probability: " << probability << std::endl;
    std::cout << "image 2D coordinates: (x = " << xmiddle << ", y = " << ymiddle << ")" << std::endl;
    std::cout << "depth: " << depth << std::endl;
    std::cout << "3D coordinates: (x = " << x << ", y = " << y << ", z = " << z << ")" << std::endl;
    std::cout << std::endl;

  }

}

int main(int argc, char **argv) {

  ros::init(argc, argv, "object_3D_positioner16");

  ros::NodeHandle nh;
  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/depth_image16", 2);
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> boundingBoxes_sub(nh, "/darknet_ros/bounding_boxes", 4);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes> MySyncPolicy;

  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(4), image_sub, boundingBoxes_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;

}
