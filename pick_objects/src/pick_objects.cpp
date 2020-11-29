#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void sendGoal(float pos_x, float pos_y, int type) {

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // Wait 5 sec for move_base action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    // set up the frame parameters
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach
    goal.target_pose.pose.position.x = pos_x;
    goal.target_pose.pose.position.y = pos_y;
    goal.target_pose.pose.orientation.w = 1.0;

    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        switch (type) {
        case 0:
            ROS_INFO("Hooray, the item was picked up");
            // Wait 5 seconds to simulate object pick up
            ros::Duration(5.0).sleep();
            break;
        case 1:
            ROS_INFO("Hooray, the item was dropped off");
            // Wait 5 seconds to simulate object pick up
            ros::Duration(5.0).sleep();
            break;
        }
    else
        ROS_INFO("The base failed to move");
}

int main(int argc, char** argv){
    // Initialize the pick_objects node
    ros::init(argc, argv, "pick_objects");

    // send pick up goal
    sendGoal(1.0, 1.0, 0);

    // send drop off goal
    sendGoal(1.0, 3.0, 1);

    return 0;
}