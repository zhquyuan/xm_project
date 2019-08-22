#ifndef XMREVIVE
#define XMREVIVE
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "xm_msgs/xm_detect_request.h"
#include "xm_msgs/xm_detect_response.h"
#include <ecl/threads.hpp>

namespace base_server{
    class xmRevive{
        public:
        xmRevive(int choice);
        void xm_turn_back(const xm_msgs::xm_detect_response& response);
        void publish(const xm_msgs::xm_detect_response& response);
        void pubReq();
        private:
        ros::NodeHandle nh;
        ros::Publisher xm_control;
        bool detect_door;//如果为true，说明有另一个节点在发布请求
        ros::Subscriber detect_obstacle;
        ecl::Thread worker_thread_;
        int choice;
    };
}
#endif
