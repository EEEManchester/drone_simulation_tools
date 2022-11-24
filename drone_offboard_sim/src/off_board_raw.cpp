/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <Eigen/Dense>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
    //ROS_INFO("state message received");
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
    ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/local", 10);
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

    mavros_msgs::PositionTarget position_ref;
    position_ref.header.stamp = ros::Time::now();

    position_ref.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    // mavros_msgs::PositionTarget::FRAME_LOCAL_NED
    // only control position
    position_ref.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY |
                  mavros_msgs::PositionTarget::IGNORE_VZ | mavros_msgs::PositionTarget::IGNORE_AFX |
                  mavros_msgs::PositionTarget::FORCE |
                  mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
                  mavros_msgs::PositionTarget::IGNORE_YAW | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;


    float take_off_height;
    nh_loc.getParam("take_off_height",take_off_height);
    ROS_INFO_STREAM("take_off_height "<< take_off_height);


    position_ref.position.x = 0;
    position_ref.position.y = 0;
    position_ref.position.z = take_off_height;


    position_ref.header.seq=1;
    position_ref.header.frame_id = 1;

    //send a few setpoints before starting
    for(int i = 400; ros::ok() && i > 0; --i){
        ROS_INFO_STREAM("publish setpoints:"<< float(400-i)<<"/"<<400);
        local_pos_pub.publish(position_ref);
        position_ref.header.seq++;
        position_ref.header.stamp = ros::Time::now();
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
        ROS_INFO_STREAM_THROTTLE(2, "take off height (absolute) " << take_off_height);
        ROS_INFO_STREAM_THROTTLE(2, "reference position (absolute) " << position_ref.position);

        position_ref.header.seq++;
        position_ref.header.stamp =ros::Time::now();
        local_pos_pub.publish(position_ref);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
