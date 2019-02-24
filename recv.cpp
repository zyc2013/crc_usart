#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cstdio>
#include <math.h>
#include "usart.h"
#include <thread>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <sys/types.h>
#include <ros/ros.h>
#include <fcntl.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
using namespace std;
using namespace cv;

class Class_Usart usart;
vector<int> data;
void serial_thread();//串口函数
std::mutex serial_mutex;//某个锁

double secs=0,secs_last;
geometry_msgs::Twist mcu_msgs;
geometry_msgs::Twist uwb_msgs;
/*
void serial_thread()
{
//extern int fd;
int err;   
//定义接受数据的数组，分别是头帧，x方向速度，y方向速度，旋转角速度

ros::NodeHandle nh; 
ros::Publisher chatter_pub = nh.advertise<geometry_msgs::Twist>("raw_vel", 1000); 
ros::Publisher uwb_pub = nh.advertise<geometry_msgs::Twist>("uwb_msg", 1000);  
          //返回调用函数的状态      
while (ros::ok()) 
	{
		//if(send_new)
		//{
		serial_mutex.lock();
		//std::cout<<"send_new"<<send_new<<std::endl;
			if (usart.Usart_Recv(Data)==0)
	{
		if(Data[0]==254)
		{
			printf("%f,%f,%f,%f;\n",Data[0],Data[1],Data[2],Data[3]);
			mcu_msgs.linear.x=Data[1];//Class_Usart::a
			mcu_msgs.linear.y=Data[2];
			mcu_msgs.angular.z=Data[3];
			chatter_pub.publish(mcu_msgs);
		}
	}
		//sleep(0.01);
		serial_mutex.unlock();
       		//std::this_thread::sleep_for(std::chrono::milliseconds(10));  
		
	//}              
		//UART0_Close(fd);   
	} 
}
*/
/*void sendCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  // 将接收到的消息打印出来
 // ROS_INFO("I heard: [%f]", msg->linear.x);
int x=msg->linear.x*100;
int y=msg->linear.y*100;
int z=msg->angular.z*100;
//if(secs==0)
//{
//	secs =ros::Time::now().toSec();
 // ROS_INFO("I heard: [%f]", secs);
	usart.Usart_Send(0xfe,x,y,z,1,1);
}
*/
int main(int argc,char** argv)
{
	ros::init(argc,argv,"r_serial");
	ros::NodeHandle n;
	volatile int Data[5]={0};
  	// 创建一个Publisher，发布名为chatter的topic，消息类型为geometry_msgs::Twist
	usart.Usart_Init();
	//std::thread thread1(serial_thread);
	//thread1.detach();
	//ros::Subscriber sub = n.subscribe("chatter", 1000,sendCallback);//位置？
	ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("raw_pos", 1000);                       //返回调用函数的状态      
	ros::Publisher uwb_pub = n.advertise<geometry_msgs::Twist>("uwb_msg", 1000);                       //返回调用函数的状态      

	ros::Rate rate(180);
	while(ros::ok())
	{
		//if(send_new)
		//{
		//serial_mutex.lock();
		//std::cout<<"send_new"<<send_new<<std::endl;
			if (usart.Usart_Recv(Data)==0)
	{
		if(Data[0]==254)//0xfe
		{
			
			mcu_msgs.linear.x=Data[1]/1000.0;//Class_Usart::a
			mcu_msgs.linear.y=Data[2]/1000.0;
			mcu_msgs.angular.z=-Data[3]/100.0;
			mcu_msgs.angular.x=Data[4]/100.0;
			chatter_pub.publish(mcu_msgs);
			ROS_INFO("%f,%f,%f;\n",mcu_msgs.linear.x,mcu_msgs.linear.y,mcu_msgs.angular.z);
		}
if(Data[0]==255)//0xff
		{
			
			mcu_msgs.linear.x=Data[1]/100.0;//Class_Usart::a
			mcu_msgs.linear.y=Data[2]/100.0;
			mcu_msgs.angular.z=Data[3]/100.0;
			mcu_msgs.angular.x=Data[4]/100.0;
			//chatter_pub.publish(mcu_msgs);
			ROS_INFO("%f,%f,%f;\n",mcu_msgs.linear.x,mcu_msgs.linear.y,mcu_msgs.angular.z);
		}

			if(Data[0]==170)//0xaa
		{
                        //printf("%d,%d,%d,%d,%d;\n",Data[0],Data[1],Data[2],Data[3],Data[4]);
			uwb_msgs.linear.x=Data[1]/1000.0;
			uwb_msgs.linear.y=Data[2]/1000.0;
			uwb_msgs.angular.z=Data[3]/100.0;
			uwb_pub.publish(uwb_msgs);
			ROS_INFO("%f,%f,%f;\n",uwb_msgs.linear.x,uwb_msgs.linear.y,uwb_msgs.angular.z);
			
		}


	}
	ros::spinOnce();
	rate.sleep();
		//serial_mutex.unlock();
       		//std::this_thread::sleep_for(std::chrono::milliseconds(10));  
		
	//}              
		//UART0_Close(fd);   
}
return 0;
}

