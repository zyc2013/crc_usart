#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int16.h>
#include <math.h>
std::vector<double> angle;
geometry_msgs::Twist pose;
geometry_msgs::Twist cmd_pose;
double laser;
int adjust_sign=0;
double degree=0;
double ans[5]={0,0,0,0,0};
double diff_i_angle=0;
double out_v_angle=0;
double diff_i_y=0;
double out_v_y=0;
double diff_i_x=0;
double out_v_x=0;
double offset_front=0;
double slide[4]={0,0,0,0};
double cos_angle[4]={0,0,0,0};
double cal_angle[4]={0,0,0,0};
double diff_y=0;
double diff_x=0;
double offset_front_x=0.1;
double offset_back_x=0.22;
double limit(double x,int a,int b)
{
if (x>a)
return a;
else if(x<b)
return b;
else
return x;
}
void sendCallback(const sensor_msgs::LaserScan& msg)//right
{
int count = msg.scan_time / msg.time_increment;
for(int i = 0; i < count; i++)
 {
      if(degree>-90.1&&degree<-89.9)
{
       // ROS_INFO("right=: [%f, %f]", degree, msg.ranges[i]);
angle[0]=msg.ranges[i];
//pose.linear.y=msg.ranges[i];
}
if(degree>-85.1&&degree<-84.9)
{
       // ROS_INFO("right=: [%f, %f]", degree, msg.ranges[i]);
angle[1]=msg.ranges[i];
}
if(degree>-80.1&&degree<-79.9)
{
       //ROS_INFO("right=: [%f, %f]", degree, msg.ranges[i]);
angle[2]=msg.ranges[i];
}
if(degree>-75.1&&degree<-74.9)
{
       //ROS_INFO("right=: [%f, %f]", degree, msg.ranges[i]);
angle[3]=msg.ranges[i];
}
if(degree>-70.1&&degree<-69.9)
{
       //ROS_INFO("right=: [%f, %f]", degree, msg.ranges[i]);
angle[4]=msg.ranges[i];
}
   }
 slide[0]=sqrt(angle[0]*angle[0]+angle[2]*angle[2]-2*angle[0]*angle[2]*cos(0.174));
   cos_angle[0]=(angle[0]*angle[0]+slide[0]*slide[0]-angle[2]*angle[2])/(2*angle[0]*slide[0]);
   cal_angle[0]=(cos_angle[0]>0.99?0.99:(cos_angle[0]<-0.99)?-0,99:acos(cos_angle[0]));
ans[1]=cal_angle[0]+1.57;//cal_angle-90+180
 slide[1]=sqrt(angle[0]*angle[0]+angle[1]*angle[1]-2*angle[0]*angle[1]*cos(0.087));
  cos_angle[1]=(angle[0]*angle[0]+slide[1]*slide[1]-angle[1]*angle[1])/(2*angle[0]*slide[1]);
  cal_angle[1]=(cos_angle[1]>0.99?0.99:(cos_angle[1]<-0.99)?-0,99:acos(cos_angle[1]));
ans[2]=cal_angle[1]+1.57;//cal_angle-90+180
 slide[2]=sqrt(angle[0]*angle[0]+angle[2]*angle[2]-2*angle[0]*angle[2]*cos(0.262));
  cos_angle[2]=(angle[0]*angle[0]+slide[2]*slide[2]-angle[2]*angle[2])/(2*angle[0]*slide[2]);
  cal_angle[2]=(cos_angle[2]>0.99?0.99:(cos_angle[2]<-0.99)?-0,99:acos(cos_angle[2]));
ans[3]=cal_angle[2]+1.57;//cal_angle-90+180
slide[3]=sqrt(angle[0]*angle[0]+angle[3]*angle[3]-2*angle[0]*angle[3]*cos(0.349));
  cos_angle[3]=(angle[0]*angle[0]+slide[3]*slide[3]-angle[3]*angle[3])/(2*angle[0]*slide[3]);
  cal_angle[3]=(cos_angle[3]>0.99?0.99:(cos_angle[3]<-0.99)?-0,99:acos(cos_angle[3]));
ans[4]=cal_angle[3]+1.57;//cal_angle-90+180
ans[0]=(ans[1]+ans[2]+ans[3]+ans[4])/4;
diff_y=(angle[0]+offset_front_x*tan(3.1416-ans[0]))*cos(3.1416-ans[0])-60;//距底面
diff_x=(laser+offset_back_x)*cos(3.1416-ans[0])-50;//距侧面
}
void posCallback(const geometry_msgs::Twist &pos)
{
laser=pos.angular.x;//测距信息
//pose.linear.y=pos.linear.y;
pose.angular.z=pos.angular.z;//角速度信息
}
void adjust_Callback(const std_msgs::Int16 &sign)
{
adjust_sign=sign.data;
}
int main(int argc,char** argv)
{
ros::init(argc,argv,"serial");
	ros::NodeHandle n;
angle.push_back(0);
angle.push_back(0);
angle.push_back(0);
  ros::Subscriber sub_scan = n.subscribe("scan",1000,sendCallback);//位置？
  ros::Publisher pub_vel=n.advertise<geometry_msgs::Twist>("cmd_vel",1);
ros::Subscriber laser_scan = n.subscribe("uwb_msg",1000,posCallback);
ros::Subscriber adjust = n.subscribe("adjust",1000,adjust_Callback);
ros::Rate rate(100);
while(ros::ok())
{
if (adjust_sign)
{
//调整pid
double diff_angle=ans[0]-3.1416;
diff_i_angle+=diff_angle*0.01;
diff_i_angle=limit(diff_i_angle,1,-1);
out_v_angle=diff_angle*3+diff_i_angle*10;
cmd_pose.angular.z=limit(out_v_angle,3.0,-3.0);
//double diff_y=angle[0]*cos(ans[0])-60;
diff_i_y+=diff_y*0.01;
diff_i_y=limit(diff_i_y,1,-1);
out_v_y=diff_y*3+diff_i_y*10;
cmd_pose.linear.y=limit(out_v_y,3.0,-3.0);
//double diff_x=laser-50;
diff_i_x+=diff_x*0.01;
diff_i_x=limit(diff_i_x,1,-1);
out_v_x=diff_x*3+diff_i_x*10;
cmd_pose.linear.x=limit(out_v_x,3.0,-3.0);
if(abs(diff_x)<5&&abs(diff_y)<5&&abs(cmd_pose.linear.x)<0.5&&abs(cmd_pose.linear.y)<0.5&&adjust_sign==1)//保证发布补弹信息只有1次
{
cmd_pose.angular.x=1;
adjust_sign=2;//为2后不再发布补弹真
}

pub_vel.publish(cmd_pose);
cmd_pose.angular.x=0;//补弹假
if(adjust_sign==2)
adjust_sign=0;//每次发布补弹后再发布一次，还原补弹标志位
}
ros::spinOnce();
rate.sleep();
}
return 0;
}

