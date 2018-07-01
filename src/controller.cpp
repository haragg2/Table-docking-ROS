#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2/utils.h>

#define BACKDIST 4

float Kp_x = 0.25;
float Kp_y = 0.25;
float Kp_yaw = 1.0;

int main(int argc, char** argv){
  ros::init(argc, argv, "controller");

  ros::NodeHandle node;

  ros::Publisher omnibotVel = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  // If set to false it will enable the omnibot to try to achieve local goal which is
  // BACKDIST meters behind the table
  bool isBotSet = false;

  ros::Rate rate(10.0);
  while (node.ok()){
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform("base_footprint", "goal", ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    float yaw = tf2::getYaw(transformStamped.transform.rotation);
    float x = transformStamped.transform.translation.x;
    float y = transformStamped.transform.translation.y;
    float locDist = sqrt(pow((x - BACKDIST*cos(yaw)), 2) + pow((y - BACKDIST*sin(yaw)), 2));
    float glbDist = sqrt(pow(x, 2) + pow(y, 2));
    geometry_msgs::Twist velMsg;

    // Try to set the location of bot BACKDIST meters behind the table before it tries to enter 
    if (locDist > 0.15 && glbDist > 0.25 && !isBotSet){
      x = x - BACKDIST*cos(yaw);
      y = y - BACKDIST*sin(yaw);
      ROS_INFO("Trying to set the omnibot");
    }
    else{
      isBotSet = true;
    }

    // If goal is reached stop the omnibot
    if (glbDist < 0.1){
      ROS_INFO("Robot reached at goal");;
      velMsg.linear.x = 0;
      velMsg.linear.y = 0;
      velMsg.angular.z = 0;
      // Enable the omnibot to take next goal
      isBotSet = false;
    }
    else{
      velMsg.linear.x = Kp_x * x;
      velMsg.linear.y = Kp_y * y;
      velMsg.angular.z = Kp_yaw * yaw;  
    }

    omnibotVel.publish(velMsg);
    rate.sleep();
  }
  return 0;
}