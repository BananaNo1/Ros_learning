#include "ros/ros.h"
#include "std_msgs/String.h"
/*
    1。初始化Ros节点
    2.订阅话题
    3.循环等待话题信息，接收到消息后进入回调函数
    4.在回调函数中完成消息处理
*/
void chatterCallback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("I heard: [%s]",msg->data.c_str());
}


int main(int argc,char **argv){
    //初始化ROS节点
    ros::init(argc,argv,"listener");

    //创建节点句柄
    ros::NodeHandle n;

    
    //创建Subscriber  在Ros Master中注册  订阅 chatter话题 注册回调函数 chatterCallback
    ros::Subscriber sub = n.subscribe("chatter",1000,chatterCallback);

    //进入循环状态，尽快调用回调函数完成处理。在ros::ok 返回false时退出
    ros::spin();
    return 0;

}