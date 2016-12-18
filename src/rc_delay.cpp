#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float32.h" // nur Testzweck
#include <vector>

std::vector<sensor_msgs::Joy::Ptr> queue;
ros::Duration delay_time(0.01); 

void callback_set_delay_time( const std_msgs::Float32::Ptr& msg ){
   ROS_INFO( "neue Delay Time: %f", msg->data);
   delay_time = ros::Duration( msg->data );   
}

void callback_rc_signal( const sensor_msgs::Joy::Ptr& msg )
{  
  msg->header.stamp += delay_time;
  queue.push_back(msg);
  ROS_INFO("Queue Size: %lu\n", queue.size() );
}

int main( int argc, char **argv )
{
  ros::init(argc, argv, "rc_delay");

  ros::NodeHandle nh;

  ros::Subscriber sub_Signal = nh.subscribe( "joy", 10, callback_rc_signal);
  ros::Subscriber sub_Delay_time = nh.subscribe( "/set_Delay_Time", 100, callback_set_delay_time );
  ros::Publisher pub = nh.advertise<sensor_msgs::Joy>("rc_signal_delayed", 1000);

  ros::Rate loop_rate(100);

  while(ros::ok())
  { 

    ros::Time now = ros::Time::now();
    while( queue.size() > 0 && queue.front()->header.stamp < now )
    {
       pub.publish(queue.front());
       queue.erase(queue.begin());
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin();
  return 0;
}
