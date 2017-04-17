#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_listener.h"

//Twist that is to be published on /edumip/cmd
static ros::Publisher edumip_twist;
/*
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg) 
{

	//convert joystick msg to twist
	twist.linear.x = -joy_msg->axes[1]; //joystick axis 1
	twist.angular.z = joy_msg->axes[0]; 
	
}
*/

void timerCallback(const ros::TimerEvent&)
{

	static tf::TransformListener tf_listener;

	try {

		// CONFIRM IF THESE NAMES ARE CORRECT
		tf::StampedTransform transform;

		tf_listener.lookupTransform("/edumip_body", "/goal", ros::Time(0), transform);
		geometry_msgs::Twist twist;

		twist.linear.y = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));
		twist.angular.z = 8 * atan2(transform.getOrigin().y(), transform.getOrigin().x());
	
		edumip_twist.publish(twist);

	} catch (tf::TransformException &ex) {
		ROS_ERROR("%s", ex.what());
		ros::Duration(1.0).sleep();
	}

}


int main(int argc, char** argv) 
{
	ros::init(argc, argv, "goal_twist_listener");
	ros::NodeHandle n;

	//edumip twist publisher
	edumip_twist = n.advertise<geometry_msgs::Twist>("/edumip/cmd", 100);

	//create a subscriber
	//ros::Subscriber sub_joy = n.subscribe("/joy", 100, joyCallback);


	//create a timer callback at 10 hz
	ros::Timer timer = n.createTimer(ros::Duration(0.1), timerCallback);

	ros::spin();

	return 0;

}