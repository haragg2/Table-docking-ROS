#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class SendGoal
{
private:
  ros::NodeHandle n; 
  ros::Subscriber goalSub;
  MoveBaseClient ac;
  geometry_msgs::Pose lastGoal;

public:
    SendGoal():ac("move_base", true)
    {
        //wait for the action server to come up
        // while(!ac.waitForServer(ros::Duration(5.0))){
        //     ROS_INFO("Waiting for the move_base action server to come up");
        // }
        goalSub  =  n.subscribe("visualization_marker", 10, &SendGoal::goalCallback, this);
    }

    bool new_goal(const visualization_msgs::Marker& marker){
        float x_dist = lastGoal.position.x - marker.pose.position.x;
        float y_dist = lastGoal.position.y - marker.pose.position.y;
        float dist = sqrt(pow(x_dist, 2) + pow(y_dist, 2));

        tf::Quaternion diffQuat, a, b;
        tf::quaternionMsgToTF(marker.pose.orientation, a);
        tf::quaternionMsgToTF(lastGoal.orientation, b);
        diffQuat = a * b.inverse();
        tf::Matrix3x3 m(diffQuat);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        if(dist>0.3 || abs(yaw) > 0.35){
            lastGoal = marker.pose;
            return true;
        }
        return false;
    }

    void goalCallback(const visualization_msgs::Marker& marker)
    {
        // Send a pose as a goal only if it differ from the previous one significantly
        if(new_goal(marker)){
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "odom";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose = marker.pose;

            ROS_INFO("Sending goal");
            ac.sendGoal(goal);
        }

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Hooray, the goal succeeded");
    }
};


int main(int argc, char** argv){
  ros::init(argc, argv, "Docking_goals");
  SendGoal goals;
  ros::spin();
  return 0;
}