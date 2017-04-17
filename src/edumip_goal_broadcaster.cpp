#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose2D.h>

/** Subscribes from a topic and broadcasts the TF frame for it */

void poseCallback(const geometry_msgs::Pose2DConstPtr& msg) {
	//Transform broadcaster, which broadcasts to Transform listener
	static tf::TransformBroadcaster br;
	tf::Transform transform;

	transform.setOrigin(tf::Vector3(msg->x, msg->y, 0.0));

	tf::Quaternion q;
	//q.setRPY(0, 0, msg->theta);
	q.setRPY(0, 0, 0);
	transform.setRotation(q);

	//sends a stamped transform with the world frame and the turtle frame
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "goal"));

}

int main(int argc, char** argv) 
{

	ros::init(argc, argv, "edumip_goal_broadcaster");
	ros::NodeHandle n;

	//subscribes to the topic where the first turtle's poses are being published
	ros::Subscriber goal_subscriber = n.subscribe("/edumip/goal_commands", 10, &poseCallback);

	ros::spin();
	return 0;

};