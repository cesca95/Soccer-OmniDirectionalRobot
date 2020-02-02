#include "ros/ros.h"
#include <ros/console.h>
#include <string> 

#include <robot_control/RobotVision.h>
#include <geometry_msgs/Twist.h>
#include <robot_control/BallPose.h>

#include <iostream> 
#include <utility> 
#include <cmath>

// Author: Astha Gupta

int missionPhase = 1; // mission phase 

int ballAngle;
int goalAngle;

ros::Publisher cmdVel_pub;

// camera parameters
std::pair <int, int> imageInfo((640/2),(480/2));
// imageInfo.first = 480;
// imageInfo.second = 640; 

float center_threshold = 0.05;

ros::ServiceClient client;

// 1) tracking where the goal and ball is (keep rotating to know )
// 2) get the ball at the center 
// 3) moving towards the ball 
// 3) pushing the ball 

// ocallback for RobotVision.msgs 
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

void call_back_vision(const robot_control::RobotVision::ConstPtr& msg){


  float ballDist =  msg->DistBall; // in mmm
  // if(missionPhase == 1){
  //   // rotate the robot 
  //   // save the ball angle 
  //   // compute all information required
  // }
  // if(missionPhase == 2){

  //   get the ball at the center 
  //   provide a reactive control for the robot 
  //   the cmd_vel should use angular to orient the ball at the center of image 
  //   use gain*(x-x') for setting cmd_vel an
	geometry_msgs::Twist robot_cmd_vel;
	// if(ballDist*(.001) > .5) {
		int xerror = ( (imageInfo.first) - msg->BallCenterX);
		//if ball is on left of center => + xerror => + rotation  
		//if ball is on right of center => - xerror => - rotation
		if(std::abs(xerror) < center_threshold*(imageInfo.first)){
			robot_cmd_vel.angular.z  = 0;
			ROS_DEBUG_STREAM("xerror is in range : " );
			// missionPhase = 2;

        robot_control::BallPose srv;
        srv.request.dist = ballDist;
        if (client.call(srv))
        {
          ROS_INFO("BallPose service request complete");
        }
        else
        {
          ROS_ERROR("BallPose service request failed");
        }

		}
		else{
			float vel = float(xerror)/(imageInfo.first);
			robot_cmd_vel.angular.z = saturateVelocity(vel,false);
			ROS_DEBUG_STREAM("xerror is very large : " );
		}
		cmdVel_pub.publish(robot_cmd_vel);
	  // }

  // }
  // if(missionPhase == 3){
  //     // if the bal is sufficiently at the center 
  // // stop the velocity command (set them to 00)-> for angular 
  // // move towards the ball (lin + angular) while keeping the ball at the center 
  // 	// 
  // }
	// if(){
	// 	// alignment objective 
	// }
  if(missionPhase == 3){
    // if are very close to the ball 
    // keep going straight and check if
    // pushed too much -> change phase 
    // keep pushing 
	  geometry_msgs::Twist robot_cmd_vel;
	  if(ballDist > .05) {
	    float vel = (ballDist - .05 );
	    robot_cmd_vel.linear.y = saturateVelocity(vel,true);

	    ROS_DEBUG_STREAM(robot_cmd_vel.linear.y);
	  }
	  cmdVel_pub.publish(robot_cmd_vel);

  }


}




int main(int argc, char **argv)
{

  ros::init(argc, argv, "central_server");
  ros::NodeHandle n("~");

  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    ros::console::notifyLoggerLevelsChanged();

  ros::Subscriber sub = n.subscribe("/robot_vision", 1000, call_back_vision);
  client = n.serviceClient<robot_control::BallPose>("/ball_pose_srv");

  cmdVel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

  // printMap();

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
  	ROS_DEBUG_STREAM("Server: " << "a");
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
