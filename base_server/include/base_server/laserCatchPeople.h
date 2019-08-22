#ifndef LASER_FOLLOW
#define LASER_FOLLOW
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
//#include <tf/tf.h>
#include <tf/transform_listener.h>


namespace base_server{
    typedef struct flag{//用来标识一个聚群
        uint8_t first;
        uint8_t num;
        double curve;
        double L;
    }flag;
    typedef struct scan{//存储数据点x,y坐标，以激光为参考坐标系
        double x;
        double y;
    }scan;
    typedef struct divs{//标识腿坐标
        double x;
        double y;
    }divs;
    typedef struct person{
        double x;
        double y;
    }person;
    class followPeople
    {
        public:
        followPeople(ros::NodeHandle &node_handle_);
        private:
        void scanCallback(const sensor_msgs::LaserScanConstPtr& input);
        void clustering(const sensor_msgs::LaserScanConstPtr& input,std::vector<flag>& cluster,std::vector<scan>& scans);
        void classify(const sensor_msgs::LaserScanConstPtr& input,std::vector<flag>& cluster,std::vector<scan>& scans);
        double radius;
        double minCurve;
        double Lmin;
        double Lmax;
        person* people;
        inline double get_radius(double a,double b,double c,double d){
            return sqrt(pow(a-c,2)+pow(b-d,2));
        }
    };
}

#endif