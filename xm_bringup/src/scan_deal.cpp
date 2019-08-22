#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <stdio.h>
#include <iostream>
#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>

#include<std_msgs/Bool.h>
#include<std_msgs/String.h>

#include<ros/service_server.h>
#include<std_srvs/Trigger.h>

#include<boost/shared_ptr.hpp>
#include<boost/make_shared.hpp>

#include <math.h> //fabs
#ifdef NAN
/* NAN is supported */
#endif
#ifdef INFINITY
/* INFINITY is supported */
#endif

namespace xm_bringup{
  class ScanDeal : public nodelet::Nodelet
  {
    public:
      ScanDeal(){}

    private:
      virtual void onInit()
      {
        ros::NodeHandle& private_nh = getPrivateNodeHandle();
        pub = private_nh.advertise<sensor_msgs::LaserScan>("/scan_deal", 10);
        sub = private_nh.subscribe("/scan", 10, &ScanDeal::callback, this);
      }

      void callback(const sensor_msgs::LaserScanConstPtr& input)
      {
        sensor_msgs::LaserScan dealScan;
        dealScan.ranges.resize(input->ranges.size());
        unsigned int n = input->ranges.size();
        dealScan.header.frame_id = input->header.frame_id;
        dealScan.header.stamp = input->header.stamp;
        dealScan.angle_min = input->angle_min;
        dealScan.angle_max = input->angle_max;
        dealScan.angle_increment = input->angle_increment;
        dealScan.time_increment = input->time_increment;
        dealScan.scan_time = input->scan_time;
        dealScan.range_min = input->range_min;
        dealScan.range_max = input->range_max;

        for (unsigned int i = 0; i < n; ++i){
          if(isinf(input->ranges[i])){
              dealScan.ranges[i] = 5.0;
          }else{
              dealScan.ranges[i] = input->ranges[i];
          }
        }
        pub.publish(dealScan);
      }

      ros::Publisher pub;
      ros::Subscriber sub;
  };

  PLUGINLIB_EXPORT_CLASS(xm_bringup::ScanDeal, nodelet::Nodelet);
  
}

