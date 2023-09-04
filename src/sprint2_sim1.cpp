#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <chrono>
#include <thread>
using namespace std;

int main(int argc, char **argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "cmd_vel_publisher");

  // NodeHandle is the main access point to communications with the ROS system
  ros::NodeHandle n;

  // Create a Publisher object that can publish messages to the "cmd_vel" topic
  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  // Set the loop rate in Hz
  ros::Rate loop_rate(10);

  int step = 0;

  while (ros::ok())
  {
    // Create a geometry_msgs::Twist message object
    geometry_msgs::Twist twist;

    switch (step)
    {
      case 0:
        // Step 0: Move forward
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
        
        break;
        
      case 1:
        // Step 1: Rotate
        twist.linear.x = 0.0;
        twist.angular.z = -0.2;
        
        break;

      case 2:
        
        twist.linear.x = 0.1;
        twist.angular.z = 0.0;
        
        break;

      case 3:
      
        twist.linear.x = 0.0;
        twist.angular.z = 0.2;
        
        break;

      case 4:
        
        twist.linear.x = 0.5;
        twist.angular.z = 0.0;
        
        break;

      case 5:
        
        twist.linear.x = 0.5;
        twist.angular.z = 0.0;
        
        break;

      case 6:
        
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
        
      break;

      case 7:
        
        twist.linear.x = 0.0;
        twist.angular.z = 0.1;
        
      break;

      case 8:
        
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
        
      break;

      case 9:
        
        twist.linear.x = 0.3;
        twist.angular.z = 0.05;
        
      break;

      case 10:
        
        twist.linear.x = 0.5;
        twist.angular.z = 0.0;
        
      break;

      case 11:
        
        twist.linear.x = 0.5;
        twist.angular.z = 0.0;
        
      break;


      case 12:
        
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
        
      break;


      case 13:
        
        twist.linear.x = 0.0;
        twist.angular.z = 0.4;
        
      break;


      case 14:
        
        twist.linear.x = 0.5;
        twist.angular.z = 0.0;
        
      break;


      default:
        // Stop
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
        std::cout<<"step: default" <<std::endl;
        
        break;
    }


    // Publish the message
    cmd_vel_pub.publish(twist);

    std::cout<<"step: "<< step<<std::endl;
    this_thread::sleep_for(chrono::seconds(3));

    // Move to the next step (this could also be triggered by external events or timers)
    step = (step + 1);

    // Send any buffered output
    ros::spinOnce();

    // Sleep for the remaining time to hit our 10 Hz rate
    loop_rate.sleep();
  }

  return 0;
}
