#include "ros/ros.h"
#include <ros/console.h>
#include <string> 

#include <robot_control/RobotVision.h>
#include <geometry_msgs/Twist.h>
#include <robot_control/BallPose.h>

#include <iostream> 
#include <utility> 
#include <cmath>

#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define PI_Local 3.14159265


// Author: Astha Gupta

int missionPhase = 1; // mission phase 

int ballAngle;
int goalAngle;

ros::Publisher cmdVel_pub;

// camera parameters
std::pair <int, int> imageInfo((640/2),(480/2));
float camera_base = 0.125;

// ************* Do not forget to set for the final work 
float goal_x = 2.0;
float goal_y = 0.0;
float goal_buff = 0.5;
float center_threshold = 0.05;

float ball_pose_x = 0;
float ball_pose_y = 0;
float ball_pose_s = false;

float ballDist_global;


geometry_msgs::Twist navigation_cmd_vel;
bool navigation_status = false;

ros::ServiceClient client;
// typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// MoveBaseClient action_client();
// MoveBaseClient action_client("move_base", true);

float saturateVelocity(float vel, bool lin_ang){
  ROS_DEBUG_STREAM("start");
  ROS_DEBUG_STREAM(vel);

  int sign = 1;
  if(vel < 0) sign = -1;
  
  vel = std::abs(vel);

  if(lin_ang == true){
  	// linear velocity saturation 
  	 if(vel > 0.1){
	    vel = 0.1;
	  }
	  if(vel < 0.02) vel = 0.02;
	  ROS_DEBUG_STREAM("end");
	  ROS_DEBUG_STREAM(vel);
	  return sign*vel;
  }
  else{
    // angular velocity saturation 
  	if(vel > 0.1){
	    vel = 0.1;
	  }
	  if(vel < 0.02) vel = 0.02;
	  ROS_DEBUG_STREAM("end");
	  ROS_DEBUG_STREAM(vel);
	  return sign*vel;
  }

}


// bool action_handler(float x, float y, float theta){
//   move_base_msgs::MoveBaseGoal goal;

//   //we'll send a goal to the robot to move 1 meter forward
//   goal.target_pose.header.frame_id = "base_link";
//   goal.target_pose.header.stamp = ros::Time::now();

//   goal.target_pose.pose.position.x = x;
//   goal.target_pose.pose.position.y = y;

//   tf2::Quaternion myQuaternion;
//   myQuaternion.setRPY( 0, 0, theta);
//   myQuaternion.normalize();

//   goal.target_pose.pose.orientation = tf2::toMsg(myQuaternion);

//   ROS_INFO("Sending goal");
//   action_client.sendGoal(goal);

//   action_client.waitForResult();

//   if(action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
//     ROS_INFO("Hooray, the base moved 1 meter forward");
//     return true;
//   }
//   else{
//     ROS_INFO("The base failed to move forward 1 meter for some reason");
//     return false;
//   }
// }

void odom_call_back(const nav_msgs::Odometry::ConstPtr& msg){
    if(missionPhase == 4){
    // if are very close to the ball 
    // keep going straight and check if
    // pushed too much -> change phase 
    // keep pushing 
    float robot_x = msg->pose.pose.position.x;
    float robot_y = msg->pose.pose.position.y;
    float x_sq = pow((robot_x - goal_x),2);
    float y_sq = pow((robot_y - goal_y),2);
    float robot_goal_dist = std::sqrt( x_sq + y_sq);

    if(robot_goal_dist < goal_buff){
      // kick
      geometry_msgs::Twist robot_cmd_vel;
      if(ballDist_global > .05) {
        float vel = (ballDist_global - .05 );
        robot_cmd_vel.linear.x = saturateVelocity(vel,true);
        cmdVel_pub.publish(robot_cmd_vel);

        ROS_DEBUG_STREAM(robot_cmd_vel.linear.x);
        ros::Duration(1).sleep();

        // 1 second push 
        robot_cmd_vel.linear.x = 0;
        cmdVel_pub.publish(robot_cmd_vel);
        missionPhase = 1;

      }
    }
    else{
      // push
    geometry_msgs::Twist robot_cmd_vel;
    if(ballDist_global > .05) {
      float vel = (ballDist_global - .05 );
      robot_cmd_vel.linear.x = saturateVelocity(vel,true);

      ROS_DEBUG_STREAM(robot_cmd_vel.linear.x);
    }
    cmdVel_pub.publish(robot_cmd_vel);

    }

  }
}

void call_back_vision(const robot_control::RobotVision::ConstPtr& msg){


  float ballDist_global =  msg->DistBall; // in mmm

  // if (msg->Ball  == false){
  //   missionPhase = 1;
  // }
  // else{
  //   missionPhase = 2;
  // }


  // if (missionPhase == 1){
  //   //  find the ball: keep rotating 
  //   geometry_msgs::Twist robot_cmd_vel;
  //   robot_cmd_vel.angular.z  = 0.1;

  // }
  // if(missionPhase == 2){
      // ball centring: center to the ball and find its position
      geometry_msgs::Twist robot_cmd_vel;
    
      int xerror = ( (imageInfo.first) - msg->BallCenterX);
      ROS_DEBUG_STREAM("the error is: " << xerror);
      ROS_DEBUG_STREAM("Threshold is: "  << center_threshold*(imageInfo.first));
      //if ball is on left of center => + xerror => + rotation  
      //if ball is on right of center => - xerror => - rotation
      if(std::abs(xerror) < center_threshold*(imageInfo.first)){
        robot_cmd_vel.angular.z  = 0;
        ROS_DEBUG_STREAM("xerror is in range : "  << center_threshold*(imageInfo.first));

          robot_control::BallPose srv;
          srv.request.dist = ballDist_global;
          if (client.call(srv))
            {
              ball_pose_x = srv.response.x;
              ball_pose_y = srv.response.y;
              ball_pose_s = true;
              ROS_INFO("BallPose service request complete");
            }
          else
            {
              ROS_ERROR("BallPose service request failed");
            }
            // if(navigation_status = false){

            // }
          // missionPhase = 3;
    }
    else{
      float vel = float(xerror)/(imageInfo.first);
      robot_cmd_vel.angular.z = saturateVelocity(vel,false);
      ROS_DEBUG_STREAM("xerror is very large : " );
    }
    cmdVel_pub.publish(robot_cmd_vel);
  //  }
  // if(missionPhase == 3){
  //   //get to the ball
  //   // for the right hand rule
  //   float theta = atan2(goal_y-ball_pose_y,goal_x - ball_pose_x);
  //   if(ball_pose_s){
  //     // set a point 20 cm  behind the ball
  //     float x = ball_pose_x + cos(-1*theta+(PI_Local/2))*.20;
  //     float y = ball_pose_y + sin(-1*theta+(PI_Local/2))*.20;
  //     // if the bal is sufficiently at the center 
  //     // call move_goal
  //     // wait for goal -> set command velocity pub
  //     for(int i = 0;  i < 3;  i++){
  //       if(action_handler(x,y,theta)){
  //         missionPhase = 4;
  //         break;
  //       }
  //     }
      
  //   }else{
  //     return;
  //   }

  // }



}

// void call_back_move_base_vel(const geometry_msgs::Twist::ConstPtr& msg){

// navigation_cmd_vel = msg;
// navigation_status = true;
// missionPhase = 2;
// }


int main(int argc, char **argv)
{

  ros::init(argc, argv, "central_server");
  ros::NodeHandle n("~");

  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    ros::console::notifyLoggerLevelsChanged();

  ros::Subscriber sub_vision = n.subscribe("/robot_vision", 1000, call_back_vision);
  ros::Subscriber sub_odom = n.subscribe("/odom", 1000, odom_call_back);
  // ros::Subscriber sub_move_vel = n.subscribe("/move_base/out_cmd_vel", 1000, call_back_move_base_vel);

  client = n.serviceClient<robot_control::BallPose>("/ball_pose_srv");


  cmdVel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  // action_client = MoveBaseClient("move_base", true);

  // while(!action_client.waitForServer(ros::Duration(5.0))){
  //   ROS_INFO("Waiting for the move_base action server to come up");
  // }

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
  	ROS_DEBUG_STREAM("Server: " << "a");
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
