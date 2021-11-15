/*
Author: Tri Nguyen
 
This ROS nodelet will subscribe to a topic (/ros_in) to receive a message and then 
republish that message to another topic (/ros_out). 

ROS Version: ROS Melodic
*/

#ifndef MY_NODELET_MESSAGESTATION_H
#define MY_NODELET_MESSAGESTATION_H
// Add the necessary includes
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/String.h>
#include <stdio.h>
 
namespace nodelet_ns
{
    // This class inherits from nodelet::Nodelet
    class MessageStation : public nodelet::Nodelet
    {     
        public:
            MessageStation(); //Constructor    
            ~MessageStation();          
 
        private:
            virtual void onInit();
            ros::NodeHandle nh;
            ros::Publisher pub;
            ros::Subscriber sub;
            
            // Display messages from /ros_in topic to the terminal window.
            // Publish to /ros_out topic
            void callback(const std_msgs::String::ConstPtr& input);
    };   
} //End of nodelet_ns namespace
#endif // MY_NODELET_MESSAGESTATION_H