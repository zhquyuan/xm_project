/*

author:xiaoyifu
description: controll the robot simple move
话说这个节点其实没必要写成插件，要是nodelet manager崩了，那可一下崩了好几个节点,懒得改了
*/

#ifndef DIRECTMOVE
#define DIRECTMOVE //如果宏名与类名一样会出现玄学错误,所以还是大写比较好

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/service_server.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <ecl/threads.hpp>
#include "xm_msgs/xm_Move.h"

namespace base_server{
    class directMove: public nodelet::Nodelet{
        public:
        directMove(){}
        virtual void onInit();
        private:
        inline double get_dis(double x, double y){
            return sqrt(x*x + y*y);
        }
        bool move_callback(xm_msgs::xm_Move::Request &req,xm_msgs::xm_Move::Response &res);
        void odom_callback(const nav_msgs::OdometryConstPtr &msg);
        //void sub();
        ros::Publisher  vel_pub;
        ros::Subscriber odom_sub;
        ros::ServiceServer move_server;
        geometry_msgs::Pose init_pose;
        geometry_msgs::Pose current_pose;
        geometry_msgs::Pose goal_pose;
       // ecl::Thread sub_thread_; 
    };
}


#endif