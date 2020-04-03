#include "ros/ros.h"
#include "swc_msgs/Control.h"
#include "swc_msgs/Waypoints.h"

ros::Publisher g_control_pub;

void controlTimerCallback(const ros::TimerEvent& timer_event) {
    // Create a new message with speed 1 (m/s) and turn angle 15 (degrees CW)
    swc_msgs::Control controlMsg;
    controlMsg.speed = 1;
    controlMsg.turn_angle = 15;

    // Publish the message to /sim/control so the simulator receives it
    g_control_pub.publish(controlMsg);
}

int main(int argc, char **argv)
{
    // Initalize our node in ROS
    ros::init(argc, argv, "cpp_robot_control_node");
    ros::NodeHandle node;

    // Create a Publisher that we can use to publish messages to the /sim/control topic
    g_control_pub = node.advertise<swc_msgs::Control>(node.resolveName("/sim/control"), 1);

    // Create and wait for Waypoints service
    ros::ServiceClient waypoint_service = node.serviceClient<swc_msgs::Waypoints>("/sim/waypoints");
    waypoint_service.waitForExistence();

    // Request waypoints and display them
    swc_msgs::Waypoints waypoints_msg;
    waypoint_service.call(waypoints_msg);

    std::cout << "Found the following waypoints:" << std::endl;
    for (int i=0; i<waypoints_msg.response.waypoints.size(); i++) {
        std::cout << waypoints_msg.response.waypoints[i] << std::endl;
    }

    // Create a timer that calls timer_callback() with a period of 0.1 (10 Hz)
    ros::Timer control_timer = node.createTimer(ros::Duration(0.1), &controlTimerCallback, false);

    // Let ROS take control of this thread until a ROS wants to kill
    ros::spin();

    return 0;
}