#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"

// 1.初始化节点 2.向Ros Master注册节点信息，  3.按照一定频率循环发布消息


int main(int argc,char **argv){

    //Ros init
    //前两个参数树命令行或launch文件输入的参数，可以用来完成命名重映射等功能;
    //第三个参数定义了publisher节点的名称，而且该名称在运行的Ros中必须是独一无二的
    //，不允许同时存在相同名称的两个节点。
    ros::init(argc,argv,"talker");

    //创建一个节点句柄，方便对节点资源的使用和管理
    ros::NodeHandle n;

    //在Ros Master端注册一个Publisher，并告诉系统Publisher节点将会发布以chatter为话题的String类型消息。
    //第二个参数表示消息发布队列的大小，当发布消息的实际速度较慢时，Publisher会将消息存储到一定的空间队列中;
    //如果消息数量超过队列大小时，ROS会自动删除队列中最早入对的消息。
    ros::Publisher chatter_pub =
     n.advertise<std_msgs::String>("chatter",1000);

    //设置循环的频率
    ros::Rate loop_rate(10);

    //进入节点的主循环，在节点未发生异常的情况下将一直在循环中进行，一旦发生异常，ros：：ok就会返回false，跳出循环。
    int count=0;
    while(ros::ok()){
        
        //初始化消息
        // 该消息类型只有一个成员，即data，用来存储字符串数据。
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hello world" <<count;
        msg.data =ss.str();

        //发布封装完的消息 msg，消息发布后，Master会查找订阅该话题的节点，并且帮助两个节点建立连接，完成消息的传输。
        //Info 类似于 printf/cout 打印日志信息
        ROS_INFO("%s",msg.data.c_str());
        chatter_pub.publish(msg);

        //用来处理节点订阅话题的回调函数
        //为了保证功能无误，建议所有节点都默认加入该函数。
        ros::spinOnce();

        //一个周期的工作完成，可以让节点休息一段时间 
        loop_rate.sleep();
        ++count;
    }
    return 0;
}
