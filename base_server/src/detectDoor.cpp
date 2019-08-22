#include "detectDoor.h"

namespace base_server{
    // void detectDoor::pubDoorState(const sensor_msgs::LaserScanConstPtr &msg){
    //     double scan_x = 0.0;
    //     double scan_y = 0.0;
    //     int count = 0;
    //     std_msgs::Bool message;
    //     for (unsigned int i = 0; i < msg->ranges.size(); ++i){
    //         float range = msg->ranges[i];
    //         float angle = msg->angle_min + i*msg->angle_increment;
    //         scan_x = range * cos(angle);//正前方为x
    //         scan_y = range * sin(angle);
    //         if ( (scan_y < 0.22 || scan_y > - 0.22)&& scan_x >1.5 && scan_y < 4.0 && scan_y >-4.0)
    //         {
    //             count++;
    //         }
    //     }
    // //ROS_INFO("%d",count);
    //     if (count > 30)
    //     {
    //         door_state = 1;
    //   //  ROS_INFO("the door is open:%d",door_state);
    //     }else{
    //         door_state = 0;
    //     }
    //     message.data = door_state;
    //     door_pub.publish(message);
    // }
    void detectDoor::publishRequest(){
        ros::Rate r(1.0);
        while(ros::ok()){
            xm_msgs::xm_detect_request msg;
            msg.need = true;
            detectFront.publish(msg);
            ros::spinOnce();
            r.sleep();
        }
    }
    void detectDoor::publish(const xm_msgs::xm_detect_response& response){
        //ROS_ERROR("JJ");
        //detectFront = ph.serviceClient<xm_msgs::xm_detect>("/xm_detect");
        // ros::Rate r(1);
        
        // while(ros::ok()){
           
        //     // xm_msgs::xm_detect srv;一个nodelet manager内的服务通信可能有问题
        //     // srv.request.need = true;
        //     // //xm_msgs::xm_detect::Response resp;
            
        //     // srv.response.direction.push_back(true);srv.response.direction.push_back(true);srv.response.direction.push_back(true);
        //     // //ROS_ERROR("NANI");
            
        //     // bool result = detectFront.call(srv);
        //     // if(result==true){
                std_msgs::Bool message;
                if(response.direction[2]==false&&response.direction[1]==false&&response.direction[0]==false)
                    door_state=1;//本来想只识别正前方，但如果机器人正对墙角，可能出现误识别，以为开门，估加强判断
                else
                    door_state=0;
                message.data = door_state;
                door_pub.publish(message);
        //         r.sleep();
        //     //}
        // }       
    }
    
    void detectDoor::onInit(){
        ph = getPrivateNodeHandle();
        door_state=false;
        ph.param("detect_door",detect_door,true);
        //ROS_ERROR("CNM");
        if(detect_door==true){
            //laser_sub = ph.subscribe("/scan_deal",100,&detectDoor::pubDoorState,this);   
            door_pub  = ph.advertise<std_msgs::Bool>("/DoorState",1);
            detectFront = ph.advertise<xm_msgs::xm_detect_request>("/xm_detect_request",1);
            resp = ph.subscribe("/xm_detect_response",1,&detectDoor::publish,this);
            //ROS_ERROR("WOC");
            pub_thread_.start(&detectDoor::publishRequest, *this);
        }  
     }
}

PLUGINLIB_EXPORT_CLASS(base_server::detectDoor, nodelet::Nodelet);