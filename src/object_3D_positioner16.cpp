#include "ros/ros.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "sensor_msgs/Image.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "geometry_msgs/TransformStamped.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
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

	static tf2_ros::TransformBroadcaster br;

  for (uint8_t i = 0; i < boundingBoxes->count; i++) {

    double probability = boundingBoxes->bounding_boxes[i].probability;
    int64_t xmin = boundingBoxes->bounding_boxes[i].xmin;
    int64_t ymin = boundingBoxes->bounding_boxes[i].ymin;
    int64_t xmax = boundingBoxes->bounding_boxes[i].xmax;
    int64_t ymax = boundingBoxes->bounding_boxes[i].ymax;
    int16_t id = boundingBoxes->bounding_boxes[i].id;
    std::string Class = boundingBoxes->bounding_boxes[i].Class;
  
    int16_t xmiddle = (xmin + xmax) / 2 * 3 / 5;
    int16_t ymiddle = (ymin + ymax) / 2 * 3 / 5;
    
    
    uint8_t depth_bits01to08 = (image->data[(xmiddle + ymiddle * IMAGE_WIDTH) * 2]);
    uint8_t depth_bits09to16 = (image->data[(xmiddle + ymiddle * IMAGE_WIDTH) * 2 + 1]);
    uint16_t rawDepth = (depth_bits09to16 << 8) | depth_bits01to08;
    float depth = rawDepth / (float)MULTIPLIER;
    
    
    //float depth = *((uint16_t *) &(image->data[(xmiddle + ymiddle * IMAGE_WIDTH) * 2])) / (float)MULTIPLIER;
		
		if (depth == 0) {
      std::cout << "Invalid" << std::endl;
      continue;
    }

		std::string name = Class + std::to_string(i);

    float x = sqrt((depth*depth) / (1 +
                                    (1 / HORIZONTAL_N_SQUARE) +
                                    ((xmiddle / (HORIZONTAL_N_SQUARE * HALF_WIDTH)) * (-2 + xmiddle / ((float)HALF_WIDTH))) +
                                    (1 / VERTICAL_N_SQUARE) +
                                    ((ymiddle / (VERTICAL_N_SQUARE * HALF_HEIGHT)) * (-2 + ymiddle / ((float)HALF_HEIGHT)))
                                    ));
    
    float y = ((1 - (xmiddle) / ((float)HALF_WIDTH)) /  ((HORIZONTAL_N) / x));
    
    float z = ((1 - (ymiddle) / ((float)HALF_HEIGHT)) /  ((VERTICAL_N) / x));
  
    std::cout << "object id: " << id << std::endl;
    std::cout << "object class: " << Class << std::endl;  
    std::cout << "probability: " << probability << std::endl;
    std::cout << "image 2D coordinates: (x = " << xmiddle << ", y = " << ymiddle << ")" << std::endl;
    std::cout << "depth: " << depth << std::endl;
    std::cout << "3D coordinates: (x = " << x << ", y = " << y << ", z = " << z << ")" << std::endl;
    std::cout << std::endl;
		
		geometry_msgs::TransformStamped transformStamped;
  
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "velodyne";
    transformStamped.child_frame_id = name;
    transformStamped.transform.translation.x = x;
    transformStamped.transform.translation.y = y;
    transformStamped.transform.translation.z = z;
    transformStamped.transform.rotation.x = 0;
    transformStamped.transform.rotation.y = 0;
    transformStamped.transform.rotation.z = 0;
    transformStamped.transform.rotation.w = 1;

    br.sendTransform(transformStamped);
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

	ROS_INFO("Started");

  ros::spin();

  return 0;

}
