#include <franka_control/ErrorRecoveryAction.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

int main(int argc, char** argv)
{
//Das alles hier lebensnotwendig - ANFANG
  ros::init(argc, argv, "own_experiment");
  ros::NodeHandle node_handle;

  ros::Publisher error_recovery = node_handle.advertise<franka_control::ErrorRecoveryActionGoal>("/franka_control/error_recovery/goal", 1);
  if(error_recovery){
      ROS_INFO("Error reset valid");
      ROS_INFO("%s", error_recovery.getTopic());
  }
  franka_control::ErrorRecoveryActionGoal errorGoal;
  error_recovery.publish(errorGoal);

    // actionlib::SimpleActionClient<franka_control::ErrorRecoveryAction> error_recovery("franka_control/error_recovery/goal", true);

    // error_recovery.waitForServer();
    // ROS_INFO("WaitForServer complete");
    // franka_control::ErrorRecoveryGoal errorGoal;
    // error_recovery.sendGoal(errorGoal);
}

