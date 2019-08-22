/***********************************************************************************************************************
* Copyright (c) Hands Free Team. All rights reserved.
* FileName: base_simple_controller.cpp
* Contact:  QQ Exchange Group -- 521037187
* Version:  V2.0
*
* LICENSING TERMS:
* The Hands Free is licensed generally under a permissive 3-clause BSD license.
* Contributions are required to be made under the same license.
*
* History:
* <author>      <time>      <version>      <desc>
* luke liao       2015.10.1   V1.0           creat this file
* xiaoyifu                                   maintain
* Description: send the robot speed and publish odom directly through specific hardware interface
***********************************************************************************************************************/

#include <base_simple_controller/base_simple_controller.h>

#include <boost/assign.hpp>

#include <tf/transform_datatypes.h>

//肖：通过硬件接口来控制底盘

namespace base_simple_controller {
// 这里注意区分base_footprint 和base_link 的概念：
// base_footprint:是底盘的两个轮子的轴线的中点
// base_link:是底盘的几何形状的中心
// 关于这两个zuobiaox的关系可以在urdf里查看

BaseSimpleController::BaseSimpleController()
	: cmd_()
	, base_frame_id_("base_footprint")
	, cmd_vel_timeout_(0.5)
	, base_name_("mobile_base")//xiao:防止后面获取参数失败，所以此处要进行默认初始化
{}

bool BaseSimpleController::init(hardware_interface::BaseVelocityInterface* hw,
                                ros::NodeHandle& root_nh,
                                ros::NodeHandle &controller_nh)
{
	last_x_ = last_y_ = last_theta_ = 0.0;
	const std::string complete_ns = controller_nh.getNamespace();
	std::size_t id = complete_ns.find_last_of("/");
	name_ = complete_ns.substr(id + 1);

	double publish_rate = 20.0;
	controller_nh.getParam("publish_rate", publish_rate);
	publish_period_ = ros::Duration(1.0 / publish_rate);
	last_vel_get_ = ros::Time::now();

	controller_nh.getParam("cmd_vel_timeout", cmd_vel_timeout_);
	controller_nh.getParam("base_frame_id", base_frame_id_);
	controller_nh.getParam("base_name", base_name_);

	handle_ = hw->getHandle(base_name_);

	odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, "odom", 100));
	odom_pub_->msg_.header.frame_id = "odom";
	odom_pub_->msg_.child_frame_id = base_frame_id_;
	odom_pub_->msg_.pose.pose.position.z = 0;
	// TODO: fulfill the true parameters add the experience parameters
	odom_pub_->msg_.pose.covariance = boost::assign::list_of
	                                  (0.001) (0)  (0)  (0)  (0)  (0)
	                                  (0)  (0.001) (0)  (0)  (0)  (0)
	                                  (0)  (0)  (1000000.0) (0)  (0)  (0)
	                                  (0)  (0)  (0)  (1000000.0) (0)  (0)
	                                  (0)  (0)  (0)  (0)  (1000000.0) (0)
	                                  (0)  (0)  (0)  (0)  (0)  (1000.0);
	odom_pub_->msg_.twist.twist.linear.z  = 0;
	odom_pub_->msg_.twist.twist.angular.x = 0;
	odom_pub_->msg_.twist.twist.angular.y = 0;
	odom_pub_->msg_.twist.covariance = boost::assign::list_of//肖：这里难道不应该是twist.covariance
	                                  (0.001) (0)  (0)  (0)  (0)  (0)
	                                  (0)  (0.001) (0)  (0)  (0)  (0)
	                                  (0)  (0)  (1000000.0) (0)  (0)  (0)
	                                  (0)  (0)  (0)  (1000000.0) (0)  (0)
	                                  (0)  (0)  (0)  (0)  (1000000.0) (0)
	                                  (0)  (0)  (0)  (0)  (0)  (1000.0);
	//TODO: add the convarience matrix to the omni base
	tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(root_nh, "/tf", 100));//tf是全局topic
	tf_odom_pub_->msg_.transforms.resize(1);
	tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
	tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
	tf_odom_pub_->msg_.transforms[0].header.frame_id = "odom";
	//smooth_cmd_vel is the topic with the velocity-smooth,please donnot directly pub to this topic!!! 
	cmd_vel_sub_ = controller_nh.subscribe("smooth_cmd_vel", 1 , &BaseSimpleController::cmdvelCallBack, this);
	return true;
}

void BaseSimpleController::cmdvelCallBack(const geometry_msgs::Twist::ConstPtr &msg)
{
	if (isRunning())
	{
		command_buffer_.writeFromNonRT(*msg);//xiao复制消息到非实时缓冲区
		last_vel_get_ = ros::Time::now();
	} else
	{
		ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
	}
}

void BaseSimpleController::update(const ros::Time& time, const ros::Duration& period)
{//xiao来自下位机的里程计据此真正进入ros层面
	if (last_state_publish_time_ + publish_period_ < time)//hardware是每隔10ms更新一次，只有达到一定的时间间隔才来更新odom
	{
		last_state_publish_time_ += publish_period_;

		const geometry_msgs::Quaternion orientation(
		    tf::createQuaternionMsgFromYaw(handle_.getTheta()));
		if (odom_pub_->trylock())
		{
			odom_pub_->msg_.header.stamp = time;
			odom_pub_->msg_.pose.pose.position.x = handle_.getX();//因为cmd_handle是继承的state_handle，所以可以通过此handle来获取底盘反馈回来的实际位置值
			odom_pub_->msg_.pose.pose.position.y = handle_.getY();
			odom_pub_->msg_.pose.pose.orientation = orientation;
			odom_pub_->msg_.twist.twist.linear.x  = handle_.getXvel();
			odom_pub_->msg_.twist.twist.linear.y  = handle_.getYvel();
			odom_pub_->msg_.twist.twist.angular.z = handle_.getThetavel();
			odom_pub_->unlockAndPublish();

		}

		// Publish tf /odom frame
		if (tf_odom_pub_->trylock())
		{//xiao我怎么觉得这波操作不对啊
			geometry_msgs::TransformStamped& odom_frame = tf_odom_pub_->msg_.transforms[0];
			odom_frame.header.stamp = time;
			odom_frame.transform.translation.x = handle_.getX();
			odom_frame.transform.translation.y = handle_.getY();
			odom_frame.transform.rotation = orientation;
			tf_odom_pub_->unlockAndPublish();
		}
	}

	geometry_msgs::Twist temp;
	temp = *(command_buffer_.readFromRT());//xiao从实时缓冲区读取数据，中间包括非实时缓冲区到实时缓冲区的指针交换
	if ((time - last_vel_get_).toSec() > cmd_vel_timeout_)//时间间隔超过0.5s，底盘会强制停止,所以cmd_vel的发布频率要高于2hz
	{
		// stop the robot first
		handle_.setXcmd(0.0);
		handle_.setYcmd(0.0);
		handle_.setThetacmd(0.0);
	} else
	{
		handle_.setXcmd(temp.linear.x);
		handle_.setYcmd(temp.linear.y);
		handle_.setThetacmd(temp.angular.z);
		//ROS_ERROR("X:%f   y:%f   theta:%f",temp.linear.x,temp.linear.y,temp.angular.z);
	}
}

void BaseSimpleController::starting(const ros::Time& time)
{
	// stop the robot first
	handle_.setXcmd(0.0);
	handle_.setYcmd(0.0);
	handle_.setThetacmd(0.0);

	last_state_publish_time_ = time;
	state_ = RUNNING;
	ROS_ERROR("Started xm_robot base_simple_controller");          

}

void BaseSimpleController::stopping(const ros::Time& time)
{
	// stop the robot
	handle_.setXcmd(0.0);
	handle_.setYcmd(0.0);
	handle_.setThetacmd(0.0);
}

}
