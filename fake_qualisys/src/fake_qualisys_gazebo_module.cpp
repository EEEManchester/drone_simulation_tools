#include "ros/ros.h"

#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/TimeReference.h"
#include "gazebo_msgs/LinkStates.h"

#include "std_msgs/String.h"
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>

#include "qualisys/Subject.h"

qualisys::Subject drone; // global qualisys message variable
std::string link_name; // gloabal link_name variable
bool new_topic_flag = false;

void callback(const gazebo_msgs::LinkStates::ConstPtr& msg)
{
  gazebo_msgs::LinkStates Gmsg = *msg;

  for(int i=0;i<Gmsg.name.size();i++)
  {
    if(!Gmsg.name[i].compare(link_name))
    {
      drone.header.stamp=ros::Time::now();

      drone.position.x = Gmsg.pose[i].position.x;
      drone.position.y = Gmsg.pose[i].position.y;
      drone.position.z = Gmsg.pose[i].position.z;

      drone.orientation.w = Gmsg.pose[i].orientation.w;
      drone.orientation.x = Gmsg.pose[i].orientation.x;
      drone.orientation.y = Gmsg.pose[i].orientation.y;
      drone.orientation.z = Gmsg.pose[i].orientation.z;

      new_topic_flag = true;
      return;
    }
  }
}

// ----------- MAIN FUNCTION --------
int main(int argc, char **argv)
{
    // Start ROS
    ros::init(argc, argv, "fake_qualisys_gazebo");

    // create global node handle for publishing and subscribing
    ros::NodeHandle nh;
    // create local node handle for private parameters
    ros::NodeHandle nh_loc("~");

    // set loop rate of qualisys
    int rate;
    nh_loc.param("mocap_rate",rate,200);
    ros::Rate loop_rate(rate);
    ROS_INFO("Mocap spinning at %d Hz", rate);

    // get name for topic in form /qualisys/"name"
    std::string name;
    nh_loc.getParam("drone_name",name); ///<

    nh_loc.getParam("link_name",link_name); ///< \param Gazebo LinkState frame name

    //name = "drone1";

    // warn the user that the messages are faked
    ROS_WARN("FAKING QUALISYS MESSAGES FOR: %s", name.data());

    //	ros::Publisher publisher for fake qualisys message
    ros::Publisher mocap_pub = nh.advertise<qualisys::Subject>("qualisys/"+name,1);

    // subscriber for gazebo message
    ros::Subscriber sub = nh.subscribe("gazebo/link_states",1, callback);

    // set immutable sections of qualisys message header
    drone.name = name;
    drone.header.stamp = ros::Time::now();

    while (ros::ok())
    {
        //ROS_WARN_THROTTLE(15,"CAUTION: %s MOCAP TOPICS ARE FAKED",name.data() );

        // do gazebo callback
        ros::spinOnce();

        // sleep, to simulate qualisys processing delay
        loop_rate.sleep();

        // publish previous mocap position and repeat
        if(new_topic_flag=true)
        {
          mocap_pub.publish(drone);
          new_topic_flag = false;
        }
    }

    return 0;
}
