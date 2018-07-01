#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

#define INFI 1000

class Laser2pc
{
private:
  laser_geometry::LaserProjection projector;
  tf::TransformListener listener;
  ros::NodeHandle n; 
  ros::Subscriber Lasersub;
  ros::Publisher vis_pub;

public:
  Laser2pc()
  {
    Lasersub = n.subscribe("scan", 100, &Laser2pc::scanCallback, this);
    vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 2, false );
  }

  geometry_msgs::Point cloud2point(int i, const sensor_msgs::PointCloud2& cloud){
    geometry_msgs::Point p;

    int arrayPosX = i*cloud.point_step + cloud.fields[0].offset; // X has an offset of 0
    int arrayPosY = i*cloud.point_step + cloud.fields[1].offset; // Y has an offset of 4
    int arrayPosZ = i*cloud.point_step + cloud.fields[2].offset; // Z has an offset of 8

    float X = 0.0;
    float Y = 0.0;
    memcpy(&X, &cloud.data[arrayPosX], sizeof(float));
    memcpy(&Y, &cloud.data[arrayPosY], sizeof(float));

    p.x = X;
    p.y = Y;
    return p;
  }

  visualization_msgs::Marker marker(geometry_msgs::Point p, float yaw){
    visualization_msgs::Marker marker;
    tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, yaw);
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::MODIFY;
    marker.pose.position.x = p.x;
    marker.pose.position.y = p.y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = q[0];
    marker.pose.orientation.y = q[1];
    marker.pose.orientation.z = q[2];
    marker.pose.orientation.w = q[3];
    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    return marker;
  }

  void sendTransform(geometry_msgs::Point p, float yaw){
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id = "goal";
    transformStamped.transform.translation.x = p.x;
    transformStamped.transform.translation.y = p.y;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);
  }

  void findCenter(int n, const std::vector<geometry_msgs::Point> &pointList, geometry_msgs::Point &center){
    geometry_msgs::Point temp = pointList[n];
    for (int i = 0; i < pointList.size(); ++i){
      float dist = sqrt(pow((temp.x - pointList[i].x), 2) + pow((temp.y - pointList[i].y), 2));
      // Find center using diagonal of the table
      if (dist > 3.1){
        center.x = (temp.x + pointList[i].x)/2;
        center.y = (temp.y + pointList[i].y)/2; 
      }
      // Use legs with smaller distance between them to find orientation
      if(dist > 2.0 && dist < 2.6){
        center.z = (temp.x > pointList[i].x)?atan2((temp.y - pointList[i].y),(temp.x - pointList[i].x)):atan2((pointList[i].y - temp.y),(pointList[i].x - temp.x));
      }
    }
  }

  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
    if(!listener.waitForTransform(
        scan_in->header.frame_id,
        "odom",
        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        ros::Duration(1.0))){
     return;
    }

    sensor_msgs::PointCloud2 cloud;
    projector.transformLaserScanToPointCloud("odom", *scan_in, cloud, listener);
    
    std::vector<geometry_msgs::Point> pointList;
    geometry_msgs::Point p = cloud2point(0, cloud);
    pointList.push_back(p);

    int n = cloud.width;
    for(int i=1; i<n; i++){
      geometry_msgs::Point p = cloud2point(i, cloud);
      float dist = sqrt(pow((pointList.back().x - p.x), 2) + pow((pointList.back().y - p.y), 2));
      // If ponint is within a radius of 20 cm then it's the same point 
      if (dist < 0.2){
        pointList.back().x = (pointList.back().x + p.x)/2;
        pointList.back().y = (pointList.back().y + p.y)/2; 
      }
      else{
        pointList.push_back(p);
      }
    }
    // Using z component to store orientation
    geometry_msgs::Point center;
    center.x =INFI;
    center.y =INFI;
    center.z =INFI;

    for (int n = 0; n < pointList.size(); ++n){
      findCenter(n, pointList, center);
      if(center.x != INFI && center.y != INFI && center.z != INFI){
        break;
      }
    }
    
    vis_pub.publish(marker(center, center.z)); 
    sendTransform(center, center.z);   
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Laser2pc");
  Laser2pc pc;
  ros::spin();
  return 0;
}