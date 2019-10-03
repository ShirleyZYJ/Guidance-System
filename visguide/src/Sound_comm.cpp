#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "geometry_msgs/PoseStamped.h"
#include "iostream"
#include "unistd.h"
using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	cout<<"checkpoint"<<endl;
	
	MoveBaseClient ac("move_base", true);
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.pose = (*msg).pose;
	goal.target_pose.header.frame_id = (*msg).header.frame_id;
	
	for (int i = 0;i < 10;++i) {

    usleep(5000000);
    cout<<"checkpoint2"<<endl;
		goal.target_pose.header.stamp = ros::Time::now();
		ac.sendGoal(goal);
    cout<<"checkpoint3"<<endl;
  		//ac.waitForResult();
      //cout<<"checkpoint3"<<endl;
		cout<<"Publishing goal again"<<endl;
	}
}
int main(int argc, char** argv){
  ros::init(argc, argv, "Nav_goals");

  //tell the action client that we want to spin a thread by default
  //MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  //while(!ac.waitForServer(ros::Duration(5.0))){
   // ROS_INFO("Waiting for the move_base action server to come up");
 // }

  //move_base_msgs::MoveBaseGoal goal;
	/**
  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = 1.0;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);


  ac.waitForResult();
  */
  ros::NodeHandle n;
  ros::Subscriber GoalSub = n.subscribe("/move_base_simple/goal", 100, callback);
  ros::Rate loop_rate(10);
	
  ros::spin();
/**
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");
*/
  return 0;
}