#include<ros/ros.h>
#include <geometry_msgs/Twist.h>
int main(int argc,char** argv)//argv没有[]
{
 // ROS节点初始化
  ros::init(argc, argv, "talker");
 // 创建节点句柄
  ros::NodeHandle n;

  geometry_msgs::Twist Pose;

  // 创建一个Publisher，发布名为chatter的topic，消息类型为std_msgs::String
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	// 设置循环的频率
  ros::Rate loop_rate(17);
  float count = 0;

 while (ros::ok())
  {
Pose.linear.x=count;
Pose.linear.y=0;
Pose.angular.z=0;
	// 发布消息
    ROS_INFO("%f", Pose.linear.x);//Bu yong ".c_str",数字符号需要用%f
    //ROS_INFO("%f", Pose.linear.x);//Bu yong ".c_str",数字符号需要用%f
    chatter_pub.publish(Pose);

	// 循环等待回调函数
    ros::spinOnce();
	
	// 按照循环频率延时
    loop_rate.sleep();
    count=0.2;//9-1 count+=1
  }
 return 0;
}

