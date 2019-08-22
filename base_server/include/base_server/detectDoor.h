/*

author:xiaoyifu
description: detect the door's state
*/

#ifndef DETECTDOOR
#define DETECTDOOR

#include <ros/ros.h>
#include <std_msgs/Bool.h>
//#include <sensor_msgs/LaserScan.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ecl/threads.hpp>
#include "xm_msgs/xm_detect_request.h"
#include "xm_msgs/xm_detect_response.h"

namespace  base_server{
    class detectDoor: public nodelet::Nodelet{
        public:
        detectDoor(){}
        virtual void onInit();
        ~detectDoor()
        {
            pub_thread_.join();
        }
        private:
        //void pubDoorState(const sensor_msgs::LaserScanConstPtr &msg);
        void publish(const xm_msgs::xm_detect_response& response);
        void publishRequest();
        bool door_state;
        bool detect_door;//是否检测门开门关
        //ros::Subscriber laser_sub;
        ros::Publisher  door_pub;
        //ros::ServiceClient detectFront;  
        ros::Publisher detectFront;
        ros::Subscriber resp;
        ecl::Thread pub_thread_;   
        ros::NodeHandle ph;  
    };
}



#endif