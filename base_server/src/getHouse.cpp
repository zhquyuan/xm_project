#ifndef DISTANCE
#define DISTANCE

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <fstream>
#include "xm_msgs/xm_house.h"

namespace base_server{
    typedef struct house{
        std::string name;
        geometry_msgs::Point leftFront;
        geometry_msgs::Point rightFront;
        geometry_msgs::Point leftBack;
        geometry_msgs::Point rightBack;
    }house;
    std::vector<house> houses;
    bool getHouse(xm_msgs::xm_house::Request &req,xm_msgs::xm_house::Response &res){
        if(req.req == true){
            tf::TransformListener listener;
            tf::StampedTransform transform;
            geometry_msgs::Point xm_place;
            try{
                listener.lookupTransform("/map", "/base_footpint",
                                ros::Time(0),transform);
            }
            catch (tf::TransformException &ex) {
                   ROS_ERROR("%s",ex.what());
                   ros::Duration(1.0).sleep();
                   return false;
            }
            xm_place.x=transform.getOrigin().x();xm_place.y=transform.getOrigin().y();
            
            for(int i=0;i<houses.size();i++){
                double xMin = (houses[i].leftFront.x < houses[i].leftBack.x) ? houses[i].leftFront.x:houses[i].leftBack.x;
                double xMax = (houses[i].rightFront.x < houses[i].rightBack.x) ? houses[i].rightFront.x:houses[i].rightBack.x;
                if(xm_place.x>xMin && xm_place.x<xMax){
                    double yMin = (houses[i].leftFront.y < houses[i].leftBack.y) ? houses[i].leftBack.y:houses[i].rightBack.y;
                    double yMax = (houses[i].rightFront.y < houses[i].rightBack.y) ? houses[i].rightFront.y:houses[i].rightBack.y;
                    if(xm_place.y>yMin && xm_place.y<yMax){
                        res.name = houses[i].name;
                        return true;
                    }
                }
            }
        }
        res.name = "";
        return true;
    }
    int main(int argc, char **argv){ 
        //ros::init(argc,argv,"base_server");
        ros::NodeHandle nh;
        std::fstream file_;        
        file_.open("/home/ye/code/xiao/src/base_server/config.txt", std::fstream::in);
        if (file_.is_open()){
            for (int i = 0; i < 4; i++){
                std::string temp;
                house tempHouse;
                houses.push_back(tempHouse);
                file_ >> temp >> houses[i].name >> houses[i].leftFront.x >> houses[i].leftFront.y >> houses[i].rightFront.x
                >> houses[i].rightFront.y >> houses[i].leftBack.x >> houses[i].leftBack.y
                >> houses[i].rightBack.x >> houses[i].rightBack.y;
                std::cout<< houses[i].name<<std::endl;
            }
            file_.close();
            ROS_INFO("FILE open and read successfully");
        }else{
            std::cerr << "config file can't be opened, check your file load " <<std::endl;
        
        }
        ros::ServiceServer houseName = nh.advertiseService("get_xm_house",&getHouse);
        return 0;
    }
}
int main(int argc, char **argv){
    return base_server::main(argc,argv);
}

#endif