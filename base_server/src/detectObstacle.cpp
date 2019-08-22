#include "detectObstacle.h"

namespace base_server{
    void detectObstacle::onInit(){
        nh=getPrivateNodeHandle();
        hok=nh.subscribe("/scan_deal",1,&detectObstacle::detectFromLaser,this);
        request=nh.subscribe("/xm_detect_request",1,&detectObstacle::detectCallback,this);
        response=nh.advertise<xm_msgs::xm_detect_response>("/xm_detect_response",1);
        worker_thread_.start(&detectObstacle::spin,*this);
        //ROS_ERROR("WOCNM");
        //sub=nh.advertiseService("/xm_detect",&detectObstacle::detectCallback,this);
        //ROS_ERROR("NO!!!");
    }
    void detectObstacle::spin(){
        ros::Rate r(1);
        while(ros::ok()){
            ros::spinOnce();
            r.sleep();
        }
    }
    void detectObstacle::detectCallback(const xm_msgs::xm_detect_request& req){
        if(req.need==true){
            xm_msgs::xm_detect_response resp;
            resp.direction.push_back(true);resp.direction.push_back(true);resp.direction.push_back(true);
            resp.direction[0] = leftTag;
            resp.direction[1] = rightTag;
            resp.direction[2] = frontTag;
            response.publish(resp);
        }
    }
    void detectObstacle::detectFromLaser(const sensor_msgs::LaserScanConstPtr& msg){
        leftTag=0;rightTag=0;frontTag=0;
        double scan_x = 0.0;
        double scan_y = 0.0;
        int leftCount = 0;int rightCount = 0;int frontCount = 0;
        for (unsigned int i = 0; i < msg->ranges.size(); ++i){
            float range = msg->ranges[i];
            float angle = msg->angle_min + i*msg->angle_increment;
            scan_x = range * cos(angle);//正前方为x
            scan_y = range * sin(angle);
            if (i<10 && scan_y>-0.25)
                leftCount++;                           //此处参数完全靠猜
            if(msg->ranges.size()-i<10 && scan_y<0.25)
                rightCount++;
            if(i>msg->ranges.size()/3 && i<2*msg->ranges.size()/3 && scan_x<0.4)
                frontCount++;
        }
        if(leftCount>=3)
            leftTag = true;
        if(rightCount>=3)
            rightTag = true;
        if(frontCount>=msg->ranges.size()/5)
            frontTag = true;
    }
}
PLUGINLIB_EXPORT_CLASS(base_server::detectObstacle, nodelet::Nodelet);
