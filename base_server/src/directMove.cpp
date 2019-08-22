#include "directMove.h"

namespace base_server{
    void directMove::onInit(){
        ros::NodeHandle nh = getPrivateNodeHandle();
        odom_sub  = nh.subscribe("/mobile_base/mobile_base_controller/odom",100,&directMove::odom_callback,this);
       // sub_thread_.start(&directMove::sub,*this);
        move_server =nh.advertiseService("move",&directMove::move_callback,this);
        vel_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/mobile_base_controller/cmd_vel",100);
    }
    // void directMove::sub(){
    //     ros::Rate rate(100);
    //     while(ros::ok()){
    //         ros::spinOnce();
    //         ROS_WARN("he nmb");
    //         rate.sleep();
    //     }
    // }
    void directMove::odom_callback(const nav_msgs::OdometryConstPtr &msg){
        current_pose = (msg->pose).pose;
        //double yaw =tf::getYaw((msg->pose).pose.orientation);
        ROS_INFO("New position  x : %f  y",current_pose.position.x,current_pose.position.y);
    }
    bool directMove::move_callback(xm_msgs::xm_Move::Request &req,xm_msgs::xm_Move::Response &res)
{
    
    bool goal_reached = false;
    init_pose = current_pose;
    ros::Rate rate(100);
    std::vector<float> xm_pos;
    xm_pos.resize(3,0.0);
    xm_pos[0] = req.position.x;
    xm_pos[1] = req.position.y;
    xm_pos[2] = req.position.z;
    
    ROS_WARN("Move Cmd  %f , %f ,%f",xm_pos[0], xm_pos[1],xm_pos[2]);
    // because now xm is 2-wheel bot ,so I remove the y-axis move
    if(fabs(xm_pos[0]) > 0)
    {
        ros::Time time=ros::Time::now();
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.25 *(xm_pos[0] > 0 ? 1.0 : -1.0);
        while(!goal_reached  && ros::ok() )
        {
            //ros::spinOnce();
            vel_pub.publish(cmd) ;
            
            double dis=get_dis(init_pose.position.x-current_pose.position.x,init_pose.position.y-current_pose.position.y);
            // std::cout<<"DIS"<<dis<<std::endl;
            if(dis >fabs(xm_pos[0])) 
                goal_reached =true;//xiao移动距离大于目标移动距离就OK
            rate.sleep();
            if(ros::Time::now()-time>ros::Duration(10))
                break;
        }
        //if(fabs(xm_pos[1]) <= 0)
        res.arrived = true;
    }
   
    goal_reached =false;
    sleep(1.0);
  
    if(fabs(xm_pos[2]) > 0)
    {
        ros::Time time=ros::Time::now();
        geometry_msgs::Twist cmd;
        cmd.angular.z = 0.3*(xm_pos[2] > 0 ? 1.0 : -1.0);
        while(!goal_reached  && ros::ok() )
        {
            //ros::spinOnce();
            // have a bug, if you pub the speed to the cmd_vel,it may jj due to the slower-proccess
            vel_pub.publish(cmd);
            if( fabs(tf::getYaw(init_pose.orientation) - tf::getYaw(current_pose.orientation) ) > fabs(xm_pos[2]) ) 
                goal_reached =true;//xiao转角满足了就将goal_reached置为true
            if(ros::Time::now()-time>ros::Duration(10))
                break;
            // std::cout<<fabs(tf::getYaw(init_pose.orientation) - tf::getYaw(current_pose.orientation) )<<std::endl;
            rate.sleep();
        }
         res.arrived =true;
    }
    return true;
    }
}

PLUGINLIB_EXPORT_CLASS(base_server::directMove, nodelet::Nodelet);