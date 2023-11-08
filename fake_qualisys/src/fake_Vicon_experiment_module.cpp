
#include "ros/ros.h"

#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/TimeReference.h"
#include "gazebo_msgs/LinkStates.h"

#include "std_msgs/String.h"
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>

#include "geometry_msgs/TransformStamped.h"

geometry_msgs::TransformStamped drone; // global Vicon message variable


// ----------- MAIN FUNCTION --------
int main(int argc, char **argv)
{
    // Start ROS
    ros::init(argc, argv, "fake_vicon_experiment");

    // create global node handle for publishing and subscribing
    ros::NodeHandle nh;
    // create local node handle for private parameters
    ros::NodeHandle nh_loc("~");

    // set loop rate of vicon
    int rate;

    double drone_post_x; 
    double drone_post_y; 
    double drone_post_z; 
    double drone_att_x; 
    double drone_att_y; 
    double drone_att_z;
    double drone_att_w; 

    nh_loc.param("mocap_rate",rate,200);
    nh_loc.param("drone_position_x",drone_post_x,0.0);
    nh_loc.param("drone_position_y",drone_post_y,0.0);
    nh_loc.param("drone_position_z",drone_post_z,0.0);
    nh_loc.param("drone_attitude_x",drone_att_x,0.0);
    nh_loc.param("drone_attitude_y",drone_att_y,0.0);
    nh_loc.param("drone_attitude_z",drone_att_z,0.0);
    nh_loc.param("drone_attitude_w",drone_att_w,1.0);  
     
    ros::Rate loop_rate(rate);
    ROS_INFO("Mocap spinning at %d Hz", rate);

    // get name for topic in form /vicon/"name"
    std::string name;
    nh_loc.getParam("drone_name",name); ///<


    //name = "drone1";

    // warn the user that the messages are faked
    ROS_WARN("FAKING Vicon MESSAGES FOR: %s", name.data());

    //	ros::Publisher publisher for fake vicon message
    ros::Publisher mocap_pub = nh.advertise<geometry_msgs::TransformStamped>("vicon/"+name,1);


    // set immutable sections of vicon message header
    drone.child_frame_id = name;
    drone.header.stamp = ros::Time::now();
    drone.transform.translation.x = drone_post_x;
    drone.transform.translation.y = drone_post_y;
    drone.transform.translation.z = drone_post_z;
    drone.transform.rotation.w = drone_att_w;
    drone.transform.rotation.x = drone_att_x;
    drone.transform.rotation.y = drone_att_y;
    drone.transform.rotation.z = drone_att_z;
    drone.header.seq = 1;

    while (ros::ok())
    {
        //ROS_WARN_THROTTLE(15,"CAUTION: %s MOCAP TOPICS ARE FAKED",name.data() );
        drone.header.stamp = ros::Time::now();
        drone.header.seq++;

        mocap_pub.publish(drone);

        // do gazebo callback
        ros::spinOnce();

        // sleep, to simulate qualisys processing delay
        loop_rate.sleep();


    }

    return 0;
}
