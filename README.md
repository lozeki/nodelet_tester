# What is a nodelet?
A nodelet is a special type of ROS node that enables us to run multiple nodes in a single process on a single computer. These nodes are actually separate threads that can communicate with each other directly without going through the traditional ROS communication network middleman. 
## Build a nodelet
This ROS nodelet will subscribe to a topic (/ros_in). It will then receive a message via that topic and then republish that message to another topic (/ros_out).
## Step by step

> Create a new package called nodelet_tester inside a catkin workspace (catkin_ws) with some dependencies

cd ~/catkin_ws/src

catkin_create_pkg nodelet_tester nodelet roscpp std_msgs

> Create a header file for the nodelet in include/nodelet_tester folder

touch nodelet_tester/include/nodelet_tester/message_station.h

```
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
```
> Create a ccp file for the nodelet in src folder

touch nodelet_tester/src/message_station.cpp

```
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
```
> With the plugin library, we don’t need to link our node executable against our nodelet library in CMakeLists. However, we need to describe it in an xml file

touch nodelet_plugins.xml

```
<library path="lib/libnodelet_tester_lib">
  <class name="nodelet_tester_lib/MessageStation" 
         type="nodelet_ns::MessageStation"  
         base_class_type="nodelet::Nodelet">
    <description>
        This nodelet receives messages and publishes them.
    </description>
  </class>
</library>
```
> Explain: nodelet_tester_lib is the name of the executable of the node that we want to use (We will declare it in Cmakelist.txt)

>Make sure you add this plugin between the <export></export> tags of the package xml file.
```
<export>
<nodelet plugin="${prefix}/nodelet_plugins.xml" />
</export>
```
> Now let add the executable into CMakeLists.txt.

```
catkin_package(
  INCLUDE_DIRS include
)
include_directories(
 include 
 ${catkin_INCLUDE_DIRS}
)
## Declare a C++ library
 add_library(nodelet_tester_lib
   src/message_station.cpp
 )
## Link libraries
target_link_libraries(nodelet_tester_lib ${catkin_LIBRARIES})
```
> Create the launch for the project

touch launch/nodelet_tester.launch

```
<launch>
  <node pkg="nodelet"
        type="nodelet"
        name="standalone_nodelet_manager" 
        args="manager"
        output="screen"
  /> 
  <node pkg="nodelet"
        type="nodelet"
        name="MessageStation"
        args="load nodelet_tester_lib/MessageStation standalone_nodelet_manager"
        output="screen"
  />
</launch>
```
## Test the project

> Now, let build the project with catkin_make

cd ~/catkin_ws/

catkin_make

> Launch the nodelet

roslaunch nodelet_tester nodelet_tester.launch

> The nodelet will repulish the message from /MassageStation/ros_in to /MassageStation/ros_out topic

> In another terminal, check out the list of active nodes.

rosnode list

> Display the topic /MassageStation/ros_out in another terminal

rostopic echo /MessageStation/ros_out 

> Manual pulish a message to /MassageStation/ros_in topic

rostopic pub /MessageStation/ros_in std_msgs/String "Hi Tri Nguyen"

> We can check the workflow in the previous terminals or use graph to dislay the connection

rqt_graph

### remap (Condition: you must have another package with a pulisher)
> The <remap> tag: Remapping allows you to "trick" a ROS node so that when it thinks it is subscribing to or publishing to /some_topic it is actually subscribing to or publishing to /some_other_topic, for instance. 

    `<remap from="/different_topic" to="/needed_topic"/>`

> We can use <remap> tag to let the project receive the input data from a different topic, ex: "/talker_msg" (from gtest_example package in this case), so we don't need to manual input it.
> Change the launch file:

```
<launch>
  <node pkg="nodelet"
        type="nodelet"
        name="standalone_nodelet_manager" 
        args="manager"
        output="screen"
  />
 
  <node pkg="nodelet"
        type="nodelet"
        name="MessageStation"
        args="load nodelet_tester_lib/MessageStation standalone_nodelet_manager"
        output="screen"        
  >
  <remap from="/MessageStation/ros_in" to="/talker_msg"/>
  </node>
</launch>
```
// Pulish the message "Hello world" with the command:

rosrun gtest_example rostalker