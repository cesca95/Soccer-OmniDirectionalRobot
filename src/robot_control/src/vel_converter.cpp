#include "ros/ros.h"
#include <ros/console.h>
#include <string> 

#include <geometry_msgs/Twist.h>
#include <control_arduino/RobotCommand.h>


float radius=0.04;
float omegaFR;
float omegaFL;
float omegaBL;
float omegaBR;

float levelFR;
float levelFL;
float levelBR;
float levelBL;

float L1=107;
float L2=85;
float saturation=1.4;



ros::Publisher chatter_pub;
ros::Subscriber sub;

// ocallback for RobotVision.msgs 


float sign_omega(float omega,float level)
{
   if(omega<0)
    level=level*10+1;
  else
    level=level*10+0;

  return level;
}


float  sature ( float level)
{
    if(level>255)
    level=255;  
return level;
}


void call_back(const geometry_msgs::Twist::ConstPtr& msg){


    omegaFL=(1/radius)*((msg->linear.x)+(msg->linear.y)+(-(L1+L2))*(msg->angular.z));
    omegaFR=(1/radius)*((msg->linear.x)+(-(msg->linear.y))+(L1+L2)*(msg->angular.z));
    omegaBL=(1/radius)*((msg->linear.x)+(-(msg->linear.y))+(-(L1+L2))*(msg->angular.z));
    omegaBR=(1/radius)*((msg->linear.x)+(msg->linear.y)+(L1+L2)*(msg->angular.z));   
    
    levelFL=round(abs(omegaFL)*255/saturation);
    levelFL=sature(levelFL);

    levelFR=round(abs(omegaFR)*255/saturation);
    levelFR=sature(levelFR);

    levelBL=round(abs(omegaBL)*255/saturation);
    levelBL=sature(levelBL);
      
    levelBR=round(abs(omegaBR)*255/saturation);
    levelBR=sature(levelBR);
  

  
  
    levelFL = sign_omega(omegaFL,levelFL);
    levelFR = sign_omega(omegaFR,levelFR);
    levelBL = sign_omega(omegaBL,levelBL);
    levelBR = sign_omega(omegaBR,levelBR);
    
    control_arduino::RobotCommand velocity; 
    
    velocity.FR=10000+levelFR;
    velocity.BR=10000+levelBR;
    velocity.BL=10000+levelBL;
    velocity.FL=10000+levelFL;
    

    chatter_pub.publish(velocity);
    
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "vel_converter");
  ros::NodeHandle n("~");

  // if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
  //  ros::console::notifyLoggerLevelsChanged();


  sub = n.subscribe("/cmd_vel", 1000, call_back);
  chatter_pub = n.advertise<control_arduino::RobotCommand>("/robot_command", 1000);

  while (ros::ok())
  {
    ros::spin();
  }

  return 0;
}
