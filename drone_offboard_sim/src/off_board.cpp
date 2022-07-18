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

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
    //ROS_INFO("state message received");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

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
    nh.getParam("take_off_height",take_off_height);

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = take_off_height;
    

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
        pose.header.seq++;
        pose.header.stamp =ros::Time::now();
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
