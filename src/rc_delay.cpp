#include "ros/ros.h"

#include <dynamic_reconfigure/server.h>
#include <quadrotor_rc_delay/DelayTimeConfig.h>

#include "sensor_msgs/Joy.h"
#include "std_msgs/Float32.h" // nur Testzweck
#include <vector>

std::vector<sensor_msgs::Joy::Ptr> queue;
ros::Duration delay_time(0.01); 

/*
void callback_set_delay_time( const std_msgs::Float32::Ptr& msg ){
   ROS_INFO( "neue Delay Time: %f", msg->data);
   delay_time = ros::Duration( msg->data );   
}
*/

void callbackSetDelayTime(quadrotor_rc_delay::DelayTimeConfig &config, uint32_t level) {	
	delay_time = ros::Duration( config.delay_time );  
	ROS_INFO( "neue Delay Time: %f", config.delay_time );	
}

void callbackRCSignal( const sensor_msgs::Joy::Ptr& msg )
{  
	msg->header.stamp += delay_time;
	queue.push_back(msg);
	ROS_INFO("Queue Size: %lu\n", queue.size() );
}

int main( int argc, char **argv )
{
	ros::init(argc, argv, "rc_delay");

	ros::NodeHandle nh;

	ros::Subscriber sub_Signal = nh.subscribe( "joy", 10, callbackRCSignal);

	dynamic_reconfigure::Server<quadrotor_rc_delay::DelayTimeConfig> server;
	dynamic_reconfigure::Server<quadrotor_rc_delay::DelayTimeConfig>::CallbackType f;
	f = boost::bind(&callbackSetDelayTime, _1, _2);
	server.setCallback(f);

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
