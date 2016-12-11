#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_srvs/Empty.h" // nur Testzweck
#include <vector>

std::vector<sensor_msgs::Joy::Ptr> queue;
ros::Duration delay_time(3.0); 

bool set_delay_time(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp){
  //delay_time
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
  ros::ServiceServer service = nh.advertiseService("set_Delay_Time", set_delay_time);
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
