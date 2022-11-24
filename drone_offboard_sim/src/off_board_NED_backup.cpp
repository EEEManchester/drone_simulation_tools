/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <Eigen/Dense>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
    //ROS_INFO("state message received");
}

// define rotation matrix representing rotation of NED w.r.t ENU
Eigen::Matrix3d trans_ENU_2_NED = Eigen::Matrix3d::Zero();


Eigen::Vector3d getPositionInNED(const Eigen::Vector3d position_ENU)
{

		Eigen::Vector3d p_NED =  Eigen::Vector3d::Zero();

		Eigen::Vector3d p_ENU(double(position_ENU(0)), double(position_ENU(1)), double(position_ENU(2)));

		
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


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    // create global node handle for publishing and subscribing
    ros::NodeHandle nh;
    //create local node handle for private parameters
    ros::NodeHandle nh_loc("~");

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    Eigen::Vector3d vx(0,1,0);
	Eigen::Vector3d vy(1,0,0);
	Eigen::Vector3d vz(0,0,-1);
	trans_ENU_2_NED.col(0) = vx;
	trans_ENU_2_NED.col(1) = vy;
	trans_ENU_2_NED.col(2) = vz;



    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50.0);

    // wait for FCU connection
    ROS_INFO("before connection");
    while(ros::ok() && !current_state.connected){
        ROS_INFO("waiting for connnection");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("after connection");

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    float take_off_height;
    nh_loc.getParam("take_off_height",take_off_height);
    ROS_INFO_STREAM("take_off_height "<< take_off_height);


    Eigen::Vector3d position_ref_ENU(0,0,double(take_off_height));
    Eigen::Vector3d position_ref_NED;

    position_ref_NED = getPositionInNED(position_ref_ENU);

    pose.pose.position.x = position_ref_NED(0);
    pose.pose.position.y = position_ref_NED(1);
    pose.pose.position.z = position_ref_NED(2);
    

    pose.header.seq=1;
    pose.header.frame_id = 1;

    //send a few setpoints before starting
    for(int i = 400; ros::ok() && i > 0; --i){
        ROS_INFO_STREAM("publish setpoints:"<< float(400-i)<<"/"<<400);
        local_pos_pub.publish(pose);
        pose.header.seq++;
        pose.header.stamp = ros::Time::now();
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("finish sending setpoints");
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ROS_INFO("step into while loop");
    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        ROS_INFO_STREAM_THROTTLE(2.5, "take off height (absolute) " << take_off_height);
        ROS_INFO_STREAM_THROTTLE(2.5, "reference position (absolute) " << position_ref_NED);

        pose.header.seq++;
        pose.header.stamp =ros::Time::now();
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
