#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include <vector>

std::vector<sensor_msgs::Joy::Ptr> queue;
ros::Duration delay_time(5.0); 

void callback_rc_signal( const sensor_msgs::Joy::Ptr& msg )
{  
  msg->header.stamp += delay_time;
  queue.push_back(msg);
  ROS_INFO("Queue Size: %lu\n", queue.size() );
}

int main( int argc, char **argv )
{
  ros::init(argc, argv, "rc_delay");

  //ros::Duration delay_time(5.0); // 5 Sekunden (fuer Test)

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe( "joy", 10, callback_rc_signal);
  ros::Publisher pub = n.advertise<sensor_msgs::Joy>("rc_signal_delayed", 1000);

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
