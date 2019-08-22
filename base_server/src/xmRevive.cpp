#include "xmRevive.h"
namespace base_server{
    xmRevive::xmRevive(int choice){
        this->choice=choice;
        ROS_ERROR("I don't want to give up!!!");
        if(choice==1)
            detect_obstacle=nh.subscribe("/xm_detect_response",1,&xmRevive::publish,this);
        else if(choice==2)
            detect_obstacle=nh.subscribe("/xm_detect_response",1,&xmRevive::xm_turn_back,this);
        xm_control=nh.advertise<geometry_msgs::Twist>("/mobile_base/mobile_base_controller/smooth_cmd_vel", 1);
        nh.param("detect_door",detect_door,true);
        if(detect_door==false){
            worker_thread_.start(&xmRevive::pubReq,*this);
        }
    }
    void xmRevive::pubReq(){
      ros::Publisher detect = nh.advertise<xm_msgs::xm_detect_request>("/xm_detect_request",1);
      xm_msgs::xm_detect_request msg;
      msg.need = true;
      detect.publish(msg);
    }

  void xmRevive::publish(const xm_msgs::xm_detect_response& response){     
    geometry_msgs::Twist cmd;
    if(response.direction[2]==false){//false就是没有障碍物
      cmd.linear.x = 0.25;
      xm_control.publish(cmd);
      ROS_ERROR("Move front!");
    }
    else if(response.direction[1]==false&&response.direction[0]==true){
      //假如右边没有障碍物左边有，就往右h后退一点
      cmd.angular.z = 0.25;
      cmd.linear.x = -0.1;
      xm_control.publish(cmd);
      ROS_ERROR("Move right!");
    }
    else if(response.direction[0]==false&&response.direction[1]==true){
      cmd.angular.z = -0.25;
      cmd.linear.x = -0.1;
      xm_control.publish(cmd);
      ROS_ERROR("Move left!");
    }
    else{
      cmd.linear.x = -0.25;
      xm_control.publish(cmd);
      ROS_ERROR("Move back!");
    }
    ROS_ERROR("$$$$XM_CONTROL MADE");
  }
  void xmRevive::xm_turn_back(const xm_msgs::xm_detect_response& response){
    geometry_msgs::Twist cmd;
    if(response.direction[0]==true||response.direction[1]==true||response.direction[2]==true){
      cmd.linear.x = -0.1;
      xm_control.publish(cmd);
      ROS_ERROR("xm Move back!");
    }
  }
}