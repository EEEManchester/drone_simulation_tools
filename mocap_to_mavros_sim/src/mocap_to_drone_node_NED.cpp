#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <string.h>
#include <Eigen/Dense>

ros::Publisher pose_pub;

// define rotation matrix representing rotation of END w.r.t NED
Eigen::Matrix3d trans_ENU_2_NED = Eigen::Matrix3d::Zero();


// getPositionInNED
// input: position vector form mocap in ENU
// output: position vector to mavros in NED

Eigen::Vector3d getPositionInNED(const geometry_msgs::Vector3  position_ENU)
{

		Eigen::Vector3d p_NED =  Eigen::Vector3d::Zero();

		Eigen::Vector3d p_ENU(double(position_ENU.x), double(position_ENU.y), double(position_ENU.z));

		
		p_NED = trans_ENU_2_NED * p_ENU;
		
		return p_NED;
}

Eigen::Quaterniond getQuaterionInNED(const geometry_msgs::Quaternion quaterion_ENU)
{
		// get quaterion in Eigen
		Eigen::Quaterniond q_ENU(double(quaterion_ENU.w), double(quaterion_ENU.x), double(quaterion_ENU.y), double(quaterion_ENU.z)); 

		// normalise
		q_ENU.normalize();

		// to rotation matrix 
		Eigen::Matrix3d R_NED=q_ENU.toRotationMatrix();

		Eigen::Quaterniond q_NED(R_NED);

		return q_NED;
}

void poseCallback(const geometry_msgs::TransformStamped::ConstPtr& msg) {

    geometry_msgs::PoseStamped poseMavros;

    poseMavros.header.frame_id = "base_link";
    poseMavros.header.stamp = ros::Time::now();

	// transfer pose infor from ENU to NED frame
	// position
	Eigen::Vector3d p_NED =  getPositionInNED(msg->transform.translation);

    poseMavros.pose.position.x = float(p_NED(0));
	poseMavros.pose.position.y = float(p_NED(1));
	poseMavros.pose.position.z = float(p_NED(2));

	// attitude 
	Eigen::Quaterniond q_NED = getQuaterionInNED(msg->transform.rotation);
    poseMavros.pose.orientation.x = float(q_NED.x());
	poseMavros.pose.orientation.y = float(q_NED.y());
	poseMavros.pose.orientation.z = float(q_NED.z());
	poseMavros.pose.orientation.w = float(q_NED.w());

  	pose_pub.publish(poseMavros);
	return;
}

int main(int argc, char *argv[])
{
		Eigen::Vector3d vx(0,1,0);
		Eigen::Vector3d vy(1,0,0);
		Eigen::Vector3d vz(0,0,-1);
		trans_ENU_2_NED.col(0) = vx;
		trans_ENU_2_NED.col(1) = vy;
		trans_ENU_2_NED.col(2) = vz;


		ros::init(argc, argv, "mocap");
		ROS_INFO("mocap to mavros node initialized");

		ros::NodeHandle nh;

		std::string drone_name, external_vision_type;

		// subscribe to qualisys/drone_name
		nh.getParam("drone_name",drone_name);
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


    ros::Rate rate(50);
    while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}


	return 0;
}
