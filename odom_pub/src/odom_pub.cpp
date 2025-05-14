#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Twist.h>

#define R 0.035 //robot redius
#define N 855.0
#define L 0.46 //distance between 2 wheel
double Dc=0.0;
double dc=0.0;
double RtickOld=0.0; // previews right read
double RtickNew=0.0; // new right read
double LtickOld=0.0; // previews left read
double LtickNew=0.0; // new left read
  double x = 0.0;
  double y = 0.0;
  double th = 0.0;
  
  double vx = 0.0;
  double vy = 0.0;
  double vth = 0.0;
    double vl,vr,v1,v2;

  


void encoder_cb(const geometry_msgs::Point32& msg){

RtickNew=(double)msg.x;//right encoder msg
LtickNew=(double)msg.y;
Dc=((2*3.14*R*((RtickNew-RtickOld)/N))+((2*3.14*R*((LtickNew-LtickOld)/N))))/2;
x = x + Dc*cos(th);
y = y + Dc*sin(th);

th= th + (((2*3.14*R*(RtickNew-RtickOld)/N)-((2*3.14*R*(LtickNew-LtickOld)/N)))/L);
RtickOld=RtickNew;
LtickOld=LtickNew;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/encoder", 1000, encoder_cb);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;
	
  

   ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  double dts = (current_time - last_time).toSec();
  vr = ((RtickNew-RtickOld)/dts)*(3.14/180);
  vr = ((LtickNew-LtickOld)/dts)*(3.14/180);
  v1= vr*R;
  v2=vl*R;

 double vx = (v1+v2)/2;
  double vy = 0.0;
  double vth = (v1-v2)/L;
  

  ros::Rate r(1.0);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    //current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    //double dt = (current_time - last_time).toSec();
    //---odom calculation------
    

    double delta_x = (vx * cos(th) - vy * sin(th)) * dts;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dts;
    double delta_th = vth * dts;

    x =x + delta_x;
    y =y + delta_y;
    th =th + delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "robot_footprint";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "robot_footprint";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}




