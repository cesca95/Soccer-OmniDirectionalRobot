#include "ros/ros.h"
#include <ros/console.h>
#include <string> 

#include <robot_control/RobotVision.h>
#include <geometry_msgs/Twist.h>

int missionPhase = 0; // mission phase 

int ballAngle;
int goalAngle;

ros::Publisher chatter_pub;

// 1) tracking where the goal and ball is (keep rotating to know )
// 2) get the ball at the center 
// 3) moving towards the ball 
// 3) pushing the ball 

// ocallback for RobotVision.msgs 
float saturateVelocity(float vel){
  ROS_DEBUG_STREAM("start");
  ROS_DEBUG_STREAM(vel);

  if(vel > 0.1){
    vel = 0.1;
  }
  if(vel < 0.02) vel = 0.02;
  ROS_DEBUG_STREAM("end");
  ROS_DEBUG_STREAM(vel);
  return vel;
}

void call_back_vision(const robot_control::RobotVision::ConstPtr& msg){

  geometry_msgs::Twist robot_cmd_vel;
  if(msg->DistBall*(.01) > .5) {
    float vel = (msg->DistBall*(.01) - .5 );
    robot_cmd_vel.linear.y = saturateVelocity(vel);

    ROS_DEBUG_STREAM(robot_cmd_vel.linear.y);
  }
  chatter_pub.publish(robot_cmd_vel);

  // if(missionPhase == 1){
  //   // rotate the robot 
  //   // save the ball angle 
  //   // compute all information required
  // }
  // if(missionPhase == 2){
  //   // get the ball at the center 
  //     // provide a reactive control for the robot 
  // // the cmd_vel should use angular to orient the ball at the center of image 
  // // use gain*(x-x') for setting cmd_vel ang
  // }
  // if(missionPhase == 3){
  //     // if the bal is sufficiently at the center 
  // // stop the velocity command (set them to 00)-> for angular 
  // // move towards the ball (lin + angular) while keeping the ball at the center 
  // }
  // if(missionPhase == 4){
  //   // if are very close to the ball 
  //   // keep going straight and check if
  //   // pushed too much -> change phase 
  //   // keep pushing 
  // }


}

void call_back(){



}

// void printMap(){
//   for (int i = 0; i < Utility::nRowMap; i++) 
//   { 
//     std::string temp;
//      for (int j = 0; j < Utility::nColMap; j++) 
//      { 
//        temp = temp + std::to_string(Utility::Map[i][j]) + " "; 
//      } 
//      // std::cout << endl; 
//      ROS_DEBUG_STREAM("Server: " << temp);
//   }
// }

// bool response_node(spawn_onmi::node::Request  &req,
//          spawn_onmi::node::Response &res)
// {

//   bool health = Utility::updateMap(req.cRow, req.cCol);

//   if(health == false){
//     ROS_ERROR_STREAM("Server: Something went wrong while updating the map");
//     return false;
//   }

//   // default values
//   res.nRow = req.cRow;
//   res.nCol = req.cCol;

//   res.isComplete = Utility::isComplete();
//   if(res.isComplete){
//       ROS_DEBUG_STREAM("Server: Exploration is complete!");
//   }

//   int resNextRow, resNextCol;
//   res.isNext = Utility::pickNext(req.cRow, req.cCol,resNextRow, resNextCol);

//   if(res.isNext){
//     res.nRow = resNextRow;
//     res.nCol = resNextCol;
//   }  
//   ROS_DEBUG_STREAM("Server: " << "b");
//   printMap();
//   return true;
// }

// bool response_initnode(spawn_onmi::initnode::Request  &req,
//          spawn_onmi::initnode::Response &res){

//   bool health = Utility::initMap(req.robotRow, req.robotCol);
//   res.clash = !(health);

//   ROS_DEBUG_STREAM("Server: " << "c");
//   printMap();

//   return true;

// }



int main(int argc, char **argv)
{

  ros::init(argc, argv, "central_server");
  ros::NodeHandle n("~");

  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    ros::console::notifyLoggerLevelsChanged();

  // std::string mapAbsPath;
  // int mapRow;
  // int mapCol;

  // n.getParam("map_row", mapRow);
  // n.getParam("map_col",mapCol);
  // n.getParam("map_abs_path",mapAbsPath);

  // bool cond1 = Utility::updateParams(mapRow,mapCol);
  // bool cond2 = Utility::getFileContent(mapAbsPath);

  // if(cond1 != true || cond2 != true){
  //   ROS_ERROR_STREAM("Server: Error while loading the map_abs_path file ");
  //   ROS_ERROR_STREAM("Server: Please check the size of map_row and map_col matches the input in map_abs_path file");
  //   return 0;

  // } 

  // ros::ServiceServer service = n.advertiseService("node_info", response_node);
  // ros::ServiceServer service_initnode = n.advertiseService("node_init", response_initnode);

  ros::Subscriber sub = n.subscribe("/robot_vision", 1000, call_back_vision);
  chatter_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

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
