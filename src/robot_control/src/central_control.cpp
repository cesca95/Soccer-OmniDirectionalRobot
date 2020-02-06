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
#include <geometry_msgs/Pose2D.h>

#define PI_Local 3.14159265



int ballAngle;
int goalAngle;

ros::Publisher cmdVel_pub;

// camera parameters
std::pair <int, int> imageInfo((640/2),(480/2));
float camera_base = 0.125;

// ************* Do not forget to set for the final work 
float goal_x = 0.0;
float goal_y = 2.0;
float goal_buff = 0.5;
float center_threshold = 0.05;

float ball_pose_x = 0;
float ball_pose_y = 0;
float ball_pose_s = false;


// updated from the odom callback 
float robot_x;
float robot_y;
float robot_theta;

float ballDist_global;


geometry_msgs::Twist navigation_cmd_vel;
bool navigation_status = false;

ros::ServiceClient client;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool sendActionStatus = false;
bool odom_update_semaphore = false;
int missionPhase = 1;

float saturateVelocity(float vel, bool lin_ang){
  // ROS_DEBUG_STREAM("start");
  // ROS_DEBUG_STREAM(vel);

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
  	if(vel > 0.5){
	    vel = 0.5;
	  }
	  if(vel < 0.02) vel = 0.02;
	  ROS_DEBUG_STREAM("end");
	  ROS_DEBUG_STREAM(vel);
	  return sign*vel;
  }

}

// void call_back_move_base_vel(const geometry_msgs::Twist::ConstPtr& msg){

// navigation_cmd_vel = msg;
// navigation_status = true;
// missionPhase = 2;
// }


move_base_msgs::MoveBaseGoal action_handler(geometry_msgs::Pose2D goalPose2D){
  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "odom";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = goalPose2D.x;
  goal.target_pose.pose.position.y = goalPose2D.y;

  tf2::Quaternion myQuaternion;
  
  myQuaternion.setRPY( 0, 0, goalPose2D.theta);
  myQuaternion.normalize();

  goal.target_pose.pose.orientation = tf2::toMsg(myQuaternion);

  return goal;
}

void odom_call_back(const nav_msgs::Odometry::ConstPtr& msg){

    if(odom_update_semaphore = true){
      robot_x = msg->pose.pose.position.x;
      robot_y = msg->pose.pose.position.y;
      tf2::Quaternion quat;
      tf2::fromMsg(msg->pose.pose.orientation, quat);
      double roll, pitch, yaw;
      tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
      robot_theta = yaw;
    }

    if(missionPhase == 4){
    // if are very close to the ball 
    // keep going straight and check if
    // pushed too much -> change phase 
    // keep pushing 
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
  ROS_DEBUG_STREAM("Mission: 0.0");

  float ballDist_global =  msg->DistBall; // in mmm
  ROS_DEBUG_STREAM("sendActionStatus: " << sendActionStatus);
  if(sendActionStatus == false){
    // ROS_DEBUG_STREAM("Mission: 0.1");
    if (msg->Ball  == false){
      //  find the ball: keep rotating 
      geometry_msgs::Twist robot_cmd_vel;
      robot_cmd_vel.angular.z  = 0.5;
      ROS_DEBUG_STREAM("Mission: 0, robot_cmd_vel: " << robot_cmd_vel);
      cmdVel_pub.publish(robot_cmd_vel);

    }
    else{
      ROS_DEBUG_STREAM("Mission: 1.0");
        // ball centring: center to the ball and find its position
      if(missionPhase == 1){
        ROS_DEBUG_STREAM("Mission: 1.01");
        geometry_msgs::Twist robot_cmd_vel;
      
        int xerror = ( (imageInfo.first) - msg->BallCenterX);
        ROS_DEBUG_STREAM("the error is: " << xerror);
        ROS_DEBUG_STREAM("Threshold is: "  << center_threshold*(imageInfo.first));
        //if ball is on left of center => + xerror => + rotation  
        //if ball is on right of center => - xerror => - rotation
          if(std::abs(xerror) < center_threshold*(imageInfo.first)){
            robot_cmd_vel.angular.z  = 0;
            ROS_DEBUG_STREAM("xerror is in range : "  << center_threshold*(imageInfo.first));
            ROS_DEBUG_STREAM("Mission: 1.1");

              robot_control::BallPose srv;
              srv.request.dist = ballDist_global;
              if (client.call(srv))
                {
                  ball_pose_x = srv.response.x;
                  ball_pose_y = srv.response.y;
                  ball_pose_s = true;
                  ROS_INFO("BallPose service request complete");
                  cmdVel_pub.publish(robot_cmd_vel);
                  missionPhase = 2;
                }
              else
                {
                  ROS_ERROR("BallPose service request failed");
                  missionPhase = 1;
                }
          }
          else{
            float vel = float(xerror)/(imageInfo.first);
            robot_cmd_vel.angular.z = saturateVelocity(vel,false);
            ROS_DEBUG_STREAM("xerror is very large : " << robot_cmd_vel);
            cmdVel_pub.publish(robot_cmd_vel);
            missionPhase = 1;
            ROS_DEBUG_STREAM("Mission: 1.2");
          }
      
     }

      if(missionPhase == 2){
        //get to the ball
        // the goal will be given to move_base
        sendActionStatus = true;
      }
    }
}
}


geometry_msgs::Pose2D behind_the_ball(){

  geometry_msgs::Pose2D out;

  out.theta = atan2(goal_y-ball_pose_y,goal_x - ball_pose_x);

  
  if(ball_pose_s){
      // set a point 20 cm  behind the ball
      out.x = ball_pose_x - cos(out.theta)*.50;
      out.y = ball_pose_y - sin(out.theta)*.50;
  }
  ROS_DEBUG_STREAM("behind_the_ball : " << out);
   ROS_DEBUG_STREAM("the_ball : " << ball_pose_x << " : " << ball_pose_y);

  return out;

  }


int main(int argc, char **argv)
{

  ros::init(argc, argv, "central_server");
  ros::NodeHandle n("~");

  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    ros::console::notifyLoggerLevelsChanged();

  ros::Subscriber sub_vision = n.subscribe("/robot_vision", 10, call_back_vision);
  ros::Subscriber sub_odom = n.subscribe("/odom", 100, odom_call_back);
  // ros::Subscriber sub_move_vel = n.subscribe("/move_base/out_cmd_vel", 1000, call_back_move_base_vel);

  client = n.serviceClient<robot_control::BallPose>("/ball_pose_srv");


  cmdVel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  MoveBaseClient action_client("move_base", true);
  
 // ROS_DEBUG_STREAM("1");

  while(!action_client.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ROS_DEBUG_STREAM("main  1");
    if(sendActionStatus == true){
      ROS_DEBUG_STREAM("main 2");

      if(missionPhase == 2){
        ROS_DEBUG_STREAM("main 3");

        geometry_msgs::Pose2D  goalPose2D =  behind_the_ball();
        move_base_msgs::MoveBaseGoal goal = action_handler(goalPose2D);

        ROS_DEBUG_STREAM("Sending goal");
        ROS_DEBUG_STREAM(goal);
        action_client.sendGoal(goal);
        action_client.waitForResult();


      for(int i = 0;  i < 3;  i++){
        if(action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
          ROS_INFO("Hooray, the base moved 1 meter forward");
          missionPhase = 4;
          break;
        }
        else{
          ROS_INFO("The base failed to move forward 1 meter for some reason");
          return false;
        }
      }
        sendActionStatus = false;
        missionPhase == 1;
        
      }

     

    //  for(int i = 0;  i < 3;  i++){
    //     if(action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    //       ROS_INFO("Hooray, the base moved 1 meter forward");
    //       missionPhase = 4;
    //       break;
    //     }
    //     else{
    //       ROS_INFO("The base failed to move forward 1 meter for some reason");
    //       return false;
    //     }
    //   }
    //   sendActionStatus = false;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
