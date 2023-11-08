#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <string.h>

ros::Publisher pose_pub;

void poseCallback(const geometry_msgs::TransformStamped::ConstPtr& msg) {

    geometry_msgs::PoseStamped poseMavros;

    poseMavros.header.frame_id = "base_link";
    poseMavros.header.stamp = ros::Time::now();

    poseMavros.pose.position.x = msg->transform.translation.x;
    poseMavros.pose.position.y = msg->transform.translation.y;
    poseMavros.pose.position.z = msg->transform.translation.z;
    poseMavros.pose.orientation = msg->transform.rotation;


  	pose_pub.publish(poseMavros);
	return;
}

int main(int argc, char *argv[])
{
		ros::init(argc, argv, "mocap");
		ROS_INFO("mocap to mavros node initialized");

		ros::NodeHandle nh;

		std::string drone_name, external_vision_type;
		int mocap_frequency;

		// subscribe to qualisys/drone_name
		nh.getParam("drone_name",drone_name);

		nh.getParam("mocap_frequency",mocap_frequency);


		ROS_INFO("getting mocap for drone : %s",drone_name.data());
		ros::Subscriber pose_sub = nh.subscribe("/vicon/"+drone_name, 1, &poseCallback);


		// Publish either vision or mocap topics
		nh.getParam("external_vision_type",external_vision_type);

		if(external_vision_type=="vision")
		{
			pose_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 1);
		}
		else if(external_vision_type=="mocap")
		{
			pose_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/mocap/pose", 1);
		}
		else
		{
			ROS_ERROR("External vision type must be either vision or mocap");
		}


    ros::Rate rate(mocap_frequency);
    while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}


	return 0;
}
