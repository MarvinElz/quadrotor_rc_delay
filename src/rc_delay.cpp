#include "ros/ros.h"

#include <dynamic_reconfigure/server.h>
#include <quadrotor_rc_delay/DelayTimeConfig.h>

#include "sensor_msgs/Joy.h"
#include "std_msgs/Float32.h" // nur Testzweck
#include <vector>

// #define LOGGING
#ifdef LOGGING
	#include <fstream>
	using namespace std;
	ofstream logFile;
  ros::Time startTime;
  double aktuell = 0;
  double verzoegert = 0;
#endif

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
	
  #ifdef LOGGING
	 aktuell = msg->axes[4] * 3.0;
  #endif

	queue.push_back(msg);
	ROS_INFO("Queue Size: %lu\n", queue.size() );
}

int main( int argc, char **argv )
{
	ros::init(argc, argv, "rc_delay");

	ros::NodeHandle nh;

  #ifdef LOGGING
	startTime = ros::Time::now();
	char filePathName[] = "/home/robo/Desktop/logRC.txt";
	logFile.open(filePathName); 
	if(!logFile.is_open()){
			ROS_ERROR("Logfile: '%s' konnte nicht geöffnet werden. Beende.", filePathName);
			return 0;
	}
	logFile << " Time , 	aktuell  ,  verzoegert" << std::endl; 
	#endif

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
            #ifdef LOGGING
						  verzoegert = queue.front()->axes[4] * 3.0;
            #endif
       			pub.publish(queue.front());
       			queue.erase(queue.begin());
    		}
				#ifdef LOGGING
					logFile << (now - startTime).toSec() << " , " <<aktuell << " , " << verzoegert << std::endl; 
				#endif
    		ros::spinOnce();
    		loop_rate.sleep();
  	}

  	ros::spin();
  	return 0;
}
