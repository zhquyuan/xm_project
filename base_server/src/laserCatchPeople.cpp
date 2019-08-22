#include "laserCatchPeople.h"

namespace base_server{
    followPeople::followPeople(ros::NodeHandle &node_handle_){
        radius=0.5;//聚类范围
        minCurve=1.1;//最小弯曲度
        Lmin=0.06;//腿最小周长
        Lmax=0.2;//腿最大周长
        people=new person;
        people->x=0;people->y=0;
        ros::Subscriber sub_hokuyo=node_handle_.subscribe("/scan_deal",100,&followPeople::scanCallback,this);
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> scan_client("move_base",true);
        move_base_msgs::MoveBaseGoal goal;
        ros::Publisher stopPub = node_handle_.advertise<geometry_msgs::Twist>("/followcmd_vel",0);
        //此处话题不能使用相对名称，即"~cmd_vel"，应该是类的内部没有用ros::init初始化
        tf::TransformListener listener;
        ros::Rate loop_rate(1);
        while(ros::ok()){
            ros::spinOnce();
            //goal.target_pose.header.frame_id ="laser";
            //goal.target_pose.header.stamp = ros::Time::now();
            double dis=get_radius(people->x,people->y,0,0);
            if(dis>0.2&&dis<1){
                //以激光作为坐标系原点
                
                double yaw=atan2(people->y,people->x);
                tf::Quaternion qangle;
                qangle.setRPY(0, 0, yaw);
                
                geometry_msgs::QuaternionStamped qsangle;
                geometry_msgs::QuaternionStamped targetqsangle;
                geometry_msgs::PointStamped point;
                geometry_msgs::PointStamped targetpoint;

                qsangle.quaternion.x=qangle[0];
                qsangle.quaternion.y=qangle[1];
                qsangle.quaternion.z=qangle[2];
                qsangle.quaternion.w=qangle[3];
                qsangle.header.frame_id="laser";
                
                point.point.x=people->x;
                point.point.y=people->y;
                point.point.z=0;
                point.header.frame_id="laser";
                
                //tf::Transformer transform;
                try{
                    //listener.lookupTransform("/map", "/laser",
                      //         ros::Time(0), transform);
                    //transform.waitForTransform("map","laser",ros::Time::now(),ros::Duration(1.0));
                    point.header.stamp=ros::Time(0);
                    listener.transformPoint("/map",point,targetpoint);
                    qsangle.header.stamp=ros::Time(0);//如果用Time::now()会提示时间不对
                    listener.transformQuaternion("/map",qsangle,targetqsangle);
                    goal.target_pose.pose.orientation.x=targetqsangle.quaternion.x;
                    goal.target_pose.pose.orientation.y=targetqsangle.quaternion.y;
                    goal.target_pose.pose.orientation.z=targetqsangle.quaternion.z;
                    goal.target_pose.pose.orientation.w=targetqsangle.quaternion.w;

                    goal.target_pose.pose.position.x =targetpoint.point.x;
                    goal.target_pose.pose.position.y =targetpoint.point.y;
                    goal.target_pose.pose.position.z =targetpoint.point.z;
                    
                    goal.target_pose.header.frame_id ="map";
                    goal.target_pose.header.stamp = ros::Time::now();
                    scan_client.sendGoal(goal);
                    ROS_ERROR("I will catch you");
                    //scan_client.waitForResult();           
                }catch (tf::TransformException &ex) {
                    ROS_ERROR("%s",ex.what());
                    //ros::Duration(1.0).sleep();
                    loop_rate.sleep();
                    continue;
                }               
            }else{
                stopPub.publish(geometry_msgs::Twist());
                ROS_ERROR("It's too close");
            }
            loop_rate.sleep();
        }        
    }
    void followPeople::scanCallback(const sensor_msgs::LaserScanConstPtr& input){
        //ROS_ERROR("I have receive a laser data");
        std::vector<flag> cluster;
        std::vector<scan> scans;
        std::vector<divs> division;
        clustering(input,cluster,scans);
        classify(input,cluster,scans);
        
        {//计算出“腿”的中心 ros里for循环只能初始化一个，所以这里把迭代器移除，为了防止命名冲突加了个代码块
            std::vector<flag>::iterator it=cluster.begin();
            //mmp,for循环里竟然不能用逗号，垃圾Ubuntu
            for(uint8_t i=0;it<cluster.end();it++){
                if(it->curve>minCurve&&it->L>Lmin&&it->L<Lmax){
                    divs newdiv;
                    
                    for(uint8_t j=i;i<j+it->num;i++){
                        newdiv.x+=scans[i].x/it->num;
                        newdiv.y+=scans[i].y/it->num;
                    }
                    division.push_back(newdiv);
                }else i+=it->num;
            }
            //ROS_ERROR("caculated finish");
        }
        if(division.size()!=0){
            double dis=get_radius(division[0].x,division[0].y,0,0);
            //ROS_ERROR("hehe");
            uint8_t max=0;
            std::vector<divs>::iterator it=division.begin();
            it++;
        
            for(uint8_t i=0;it<division.end();it++,i++){
                //只采用检测到的最远点作为可信数据
                if(get_radius(it->x,it->y,0,0)>0){
                    max=i;
                    dis=get_radius(it->x,it->y,0,0);
                }
                //ROS_ERROR("HAHA");
            }
            if(get_radius(division[max].x,division[max].y,0,0)<5){
                //滤去距离太远的点
                people->x=division[max].x;people->y=division[max].y;
            }
            ROS_ERROR("callback finish");
        }
    }
    void followPeople::clustering(const sensor_msgs::LaserScanConstPtr& input, std::vector<flag>& cluster,std::vector<scan>& scans){
        //ROS_ERROR("I am ready to deal with laser data");
        flag firstFlag;
        firstFlag.first=0;firstFlag.num=1;
        scan firstScan;//用来存储每个激光数据点转换成x，y后的值
        double angle=input->angle_min;
        firstScan.x=(input->ranges[0])*cos(angle);firstScan.y=(input->ranges[0])*sin(angle);
        cluster.push_back(firstFlag);
        scans.push_back(firstScan);
        //ROS_ERROR("initialized successful");
        //std::vector<flag>::iterator it=cluster.begin();去他妈的迭代器，这里容器一直在增长，同时使用容易出事，可以试试改为用顺序容器迭代器适配器来添加元素 
        uint8_t j=0;
        for(uint8_t i=1;i<input->ranges.size()&&angle<input->angle_max;i++){
            //ROS_ERROR("I'm dealing with laser data");
            scan tempscan;
            angle+=input->angle_increment;
            tempscan.x=(input->ranges[i])*cos(angle);tempscan.y=(input->ranges[i])*sin(angle);
            scans.push_back(tempscan);//不用担心收回，此处拷贝了整个数据，而不是拷贝引用地址
            //ROS_ERROR("a new good data");
            if(get_radius(scans[i].x,scans[i].y,scans[i-cluster[j].num+1].x,scans[i-cluster[j].num+1].y)>radius){
                //ROS_ERROR("a new cluster");
                flag tempflag;
                tempflag.first=i;tempflag.num=1;
                cluster.push_back(tempflag);
                //it++;
                j++;
            }else //it->num++;
            cluster[j].num++;
        }
        ROS_WARN("I have dealt with laser data");
    }
    void followPeople::classify(const sensor_msgs::LaserScanConstPtr& input,std::vector<flag>& cluster,std::vector<scan>& scans){
        std::vector<flag>::iterator it=cluster.begin();
        for(uint8_t j=0;it<cluster.end();it++){
            double L=0.0;
            double D=0.0;
            for(uint8_t i=0;i<it->num-1;i++,j++){
                L+=get_radius(scans[j].x,scans[j].y,scans[j+1].x,scans[j+1].y);
            }
            D=get_radius(scans[j].x,scans[j].y,scans[j-it->num+1].x,scans[j-it->num+1].y);
            it->curve=L/D;
            it->L=L;
        }
        ROS_INFO("classify finish");
    } 
   
}
 int main(int argc,char **argv){
        ros::init(argc, argv, "laserCatchPeople");
        ros::NodeHandle node_handle_("laser");
        base_server::followPeople follow(node_handle_);
        ROS_ERROR("oh fuck");
        return 0;
    }
