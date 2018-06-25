#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <visualization_msgs/Marker.h>

class Laser2pc
{
private:
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  ros::NodeHandle n_; 
  ros::Publisher PC2pub_;
  ros::Subscriber Lasersub_;
  ros::Publisher vis_pub_;

public:
  Laser2pc()
  {
    PC2pub_ = n_.advertise<sensor_msgs::PointCloud2> ("cloud", 100, false);
    Lasersub_ = n_.subscribe("scan", 100, &Laser2pc::scanCallback, this);
    vis_pub_ = n_.advertise<visualization_msgs::Marker>( "visualization_marker", 2, false );
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

  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
    if(!listener_.waitForTransform(
        scan_in->header.frame_id,
        "odom",
        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        ros::Duration(1.0))){
     return;
    }

    sensor_msgs::PointCloud2 cloud;
    projector_.transformLaserScanToPointCloud("odom", *scan_in, cloud, listener_);

    PC2pub_.publish(cloud);
    std::vector<geometry_msgs::Point> point_list;

    geometry_msgs::Point p = cloud2point(0, cloud);
    point_list.push_back(p);

    int n = cloud.width;
    for(int i=1; i<n; i++){
      geometry_msgs::Point p = cloud2point(i, cloud);
      float dist = sqrt(pow((point_list.back().x - p.x), 2) + pow((point_list.back().y - p.y), 2));
      // If ponint is within a radius of 20 cm then it's the same point 
      if (dist < 0.2){
        point_list.back().x = (point_list.back().x + p.x)/2;
        point_list.back().y = (point_list.back().y + p.y)/2; 
      }
      else{
        point_list.push_back(p);
      }
    }

    geometry_msgs::Point center;
    float angle = 0;
    geometry_msgs::Point temp = point_list[0];
    std::vector<geometry_msgs::Point>::iterator i;

    for (i = point_list.begin(); i != point_list.end(); ++i){
      float dist = sqrt(pow((temp.x - i->x), 2) + pow((temp.y - i->y), 2));
      // Find center using diagonal of the table
      if (dist >= 3.0){
        center.x = (temp.x + i->x)/2;
        center.y = (temp.y + i->y)/2; 
      }
      if(dist <= 2.5){
        angle = (temp.x > i->x)?atan2((temp.y - i->y),(temp.x - i->x)):atan2((i->y - temp.y),(i->x - temp.x));
      }
    }
    vis_pub_.publish(marker(center, angle));    
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Laser2pc");
  Laser2pc pc;
  ros::spin();
  return 0;
}