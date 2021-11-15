/*
Author: Tri Nguyen
 
This ROS nodelet will subscribe to a topic (/ros_in) to receive a message and then 
republish that message to another topic (/ros_out). 
 
ROS Version: ROS Melodic
*/
 
// Add the necessary includes
#include "nodelet_tester/message_station.h"
#include <pluginlib/class_list_macros.h>

namespace nodelet_ns
{    
    MessageStation::MessageStation()
    {
        ROS_INFO("MessageStation Constructor");
    } 

    MessageStation::~MessageStation()
    {
        ROS_INFO("MessageStation Destructor");
    } 
    void MessageStation::onInit()
    {
        // Init the NodeHandle
        nh = getPrivateNodeHandle();
        ROS_INFO("Initializing nodelet...");

        // Create a publisher topic
        pub = nh.advertise<std_msgs::String>("ros_out",10); 
             
        // Create a subscriber topic
        sub = nh.subscribe("ros_in",10, &MessageStation::callback, this); 
    }

    void MessageStation::callback(const std_msgs::String::ConstPtr& input)
    { 
        std_msgs::String output;
        output.data = input->data;        
        ROS_INFO("msg data = %s",output.data.c_str());
        pub.publish(output);        
    }  
    // Export the MessageStation class as a plugin using the PLUGINLIB_EXPORT_CLASS macro. 
    // We don’t need to link our node executable against our nodelet library in CMakeLists.
    PLUGINLIB_EXPORT_CLASS(nodelet_ns::MessageStation, nodelet::Nodelet);
} //End of nodelet_ns namespace



 