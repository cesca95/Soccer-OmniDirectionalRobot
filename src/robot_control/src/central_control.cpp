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
std::pair <int, int> imageInfo((320/2),(240/2));
float camera_base = 0.125;

// ************* Do not forget to set for the final work 
float goal_x = 0.0;
float goal_y = 2.0;
float goal_buff = 0.9;
float center_threshold = 0.05;

float ball_pose_x = 0;
float ball_pose_y = 0;
float ball_pose_s = false;


// updated from the odom callback 
float robot_x;
float robot_y;
float robot_theta;

float ballDist_global;
bool ball_camera_status = false;

geometry_msgs::Twist navigation_cmd_vel;
bool navigation_status = false;

ros::ServiceClient client;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool sendActionStatus = false;
bool odom_update_semaphore = true; // default is true
int missionPhase = 1;
bool blindmode = false;

int  ballImage_X;
int  ballImage_Y;



float saturateVelocity(float vel, bool lin_ang){

  int sign = 1;
  if(vel < 0) sign = -1;
  
  vel = std::abs(vel);

  if(lin_ang == true){
  	// linear velocity saturation 
  	 if(vel > 0.5){
	    vel = 0.5;
	  }
	  if(vel < 0.02) vel = 0.02;
	  
  }
  else{
    // angular velocity saturation 
  	if(vel > 0.5){
	    vel = 0.5;
	  }
	  if(vel < 0.02) vel = 0.02;
  }
  return sign*vel;

}

// void call_back_move_base_vel(const geometry_msgs::Twist::ConstPtr& msg){

//   if(sendActionStatus == true){
//     cmdVel_pub.publish(msg);
//     ROS_DEBUG_STREAM("Mission: 4.2");
//   }
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

    // if(odom_update_semaphore = true){
      robot_x = msg->pose.pose.position.x;
      robot_y = msg->pose.pose.position.y;
      tf2::Quaternion quat;
      tf2::fromMsg(msg->pose.pose.orientation, quat);
      double roll, pitch, yaw;
      tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
      robot_theta = yaw;
    // }

    float x_sq = pow((robot_x - goal_x),2);
    float y_sq = pow((robot_y - goal_y),2);
    float robot_goal_dist = std::sqrt( x_sq + y_sq);

    float r_x = cos(robot_theta);
    float r_y = sin(robot_theta);

    float rg_x = (goal_x  - robot_x)/robot_goal_dist; // unit vector_x
    float rg_y = (goal_y - robot_y)/ robot_goal_dist; // unit vector_y 

    // float align_theta = acos(r_x*rg_x + r_y*rg_y); 
    float align_theta = asin(r_x*rg_y - r_y*rg_x); 

    if(missionPhase == 5){

      blindmode = true;
      if(robot_goal_dist > goal_buff){
          // ToDO Add check for the ball infront of camera in vision for all red
          geometry_msgs::Twist robot_cmd_vel;

          // ROS_DEBUG_STREAM("Mission: 5.1");

          int xerror = ( (imageInfo.first) - ballImage_X);

            // if(std::abs(xerror) > center_threshold*(imageInfo.first)){
               // stop centring
              robot_cmd_vel.linear.x = saturateVelocity(robot_goal_dist*0.1,true);

                
              robot_cmd_vel.angular.z  = saturateVelocity(align_theta,false);
              robot_cmd_vel.linear.y  = saturateVelocity(float(xerror)/(imageInfo.first), true);
              cmdVel_pub.publish(robot_cmd_vel);
              ROS_DEBUG_STREAM("stream value:" << robot_cmd_vel);
              ROS_DEBUG_STREAM("align_theta value:" << align_theta);

            // }
            // else{ // keep centering
            //     float vel = float(xerror)/(imageInfo.first);
            //     robot_cmd_vel.angular.z  = saturateVelocity(align_theta,false);
            //     robot_cmd_vel.linear.x = saturateVelocity(robot_goal_dist*0.1,true);
            //     cmdVel_pub.publish(robot_cmd_vel);
            //     ROS_DEBUG_STREAM("Mission: 2.2");
            // }


        }
        else{
          missionPhase = 6;
        }
    }
    if(missionPhase == 6){
          blindmode = true;
          // ROS_DEBUG_STREAM("Mission: 6 X:" << robot_x);
          // ROS_DEBUG_STREAM("Mission: 6 Y:" << robot_y);
          // ROS_DEBUG_STREAM("Mission: 6 Dist:" << robot_goal_dist);
          // if(robot_goal_dist < goal_buff){
        // kick
          geometry_msgs::Twist robot_cmd_vel;
          // float vel = 0.5;
          // robot_cmd_vel.linear.x = saturateVelocity(vel,true);
          robot_cmd_vel.linear.x = 0.5;
          cmdVel_pub.publish(robot_cmd_vel);

          ROS_DEBUG_STREAM(robot_cmd_vel.linear.x);
          ros::Duration(0.5).sleep();

          // 1 second push 
          robot_cmd_vel.linear.x = 0;
          cmdVel_pub.publish(robot_cmd_vel);
          missionPhase = 10;
          // blindmode = false;
          ROS_DEBUG_STREAM("Mission: 6");

      // }
    }


  }

void call_back_vision(const robot_control::RobotVision::ConstPtr& msg){
  // ROS_DEBUG_STREAM("Mission: 0.0");

  ballDist_global =  msg->DistBall;
  ball_camera_status = msg->Ball;
  ballImage_X = msg->BallCenterX;
  ballImage_Y = msg->BallCenterY;

  if(blindmode == true) return;

  // ever if the ball is not seen it is mission phase 1
  if(ball_camera_status == false) missionPhase =1;
  

  ROS_DEBUG_STREAM("Mission: phase: " << missionPhase);
  ROS_DEBUG_STREAM("sendActionStatus: " << sendActionStatus);
  ROS_DEBUG_STREAM("ball_camera_status: " << ball_camera_status);

  if(sendActionStatus == false){
    if(missionPhase == 1){
        if (ball_camera_status  == false){ // find the ball
          geometry_msgs::Twist robot_cmd_vel;
          robot_cmd_vel.angular.z  = 0.5;
          cmdVel_pub.publish(robot_cmd_vel);
          ROS_DEBUG_STREAM("Mission: 1.1");
        }else{
          missionPhase = 2; // ball  has been seen by the robot 
        }
    }
    else if(missionPhase == 2){ 

        geometry_msgs::Twist robot_cmd_vel;

        int xerror = ( (imageInfo.first) - msg->BallCenterX);

        if(std::abs(xerror) < center_threshold*(imageInfo.first)){
           // stop centring
            robot_cmd_vel.angular.z  = 0;
            cmdVel_pub.publish(robot_cmd_vel);
            missionPhase = 3;
            ROS_DEBUG_STREAM("Mission: 2.1");

        }
        else{ // keep centering
            float vel = float(xerror)/(imageInfo.first);
            robot_cmd_vel.angular.z = saturateVelocity(vel,false);
            cmdVel_pub.publish(robot_cmd_vel);
            ROS_DEBUG_STREAM("Mission: 2.2");
          }
      
     }
     else if (missionPhase == 3){
        robot_control::BallPose srv;
        srv.request.dist = ballDist_global;
        if (client.call(srv))
        {
          ball_pose_x = srv.response.x;
          ball_pose_y = srv.response.y;
          ball_pose_s = true;
          missionPhase = 4;
          ROS_DEBUG_STREAM("Mission: 3.1");
        }
      else
        {
          ROS_ERROR("BallPose service request failed");
        }
     }
     else if(missionPhase == 4){
        //get to the ball
        // the goal will be given to move_base
        ROS_DEBUG_STREAM("Mission: 4.1");
        sendActionStatus = true;
      }
    }
}



geometry_msgs::Pose2D behind_the_ball(){

  geometry_msgs::Pose2D out;

  out.theta = atan2(goal_y-ball_pose_y,goal_x - ball_pose_x);

  
  if(ball_pose_s){
      // set a point 20 cm  behind the ball
      out.x = ball_pose_x - cos(out.theta)*.40;
      out.y = ball_pose_y - sin(out.theta)*.40;
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
    if(sendActionStatus == true){

      if(missionPhase == 4){

        geometry_msgs::Pose2D  goalPose2D =  behind_the_ball();
        move_base_msgs::MoveBaseGoal goal = action_handler(goalPose2D);

        action_client.sendGoal(goal);
        action_client.waitForResult();

        if(action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
          ROS_INFO("Hooray, the base moved 1 meter forward");
          missionPhase = 5;
        }
        else{
          ROS_INFO("The base failed to move forward 1 meter for some reason");
        }

        sendActionStatus = false;  
      }

    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
