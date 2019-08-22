#ifndef DETECT_OBSTACLE
#define DETECT_OBSTACLE

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "xm_msgs/xm_detect_request.h"
#include "xm_msgs/xm_detect_response.h"
#include <ecl/threads.hpp>

namespace base_server{
    class detectObstacle: public nodelet::Nodelet{
        public:
        detectObstacle(){};
        virtual void onInit();
        private:
        void detectCallback(const xm_msgs::xm_detect_request& req);
        void detectFromLaser(const sensor_msgs::LaserScanConstPtr& msg);
        void spin();
        bool leftTag;
        bool rightTag;
        bool frontTag;
        ros::Subscriber hok;
        ros::Subscriber request;
        ros::Publisher response;
        //ros::ServiceServer sub;
        ros::NodeHandle nh;
        ecl::Thread worker_thread_;
    };
}
#endif