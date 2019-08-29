// Es handelt sich hierbei um ein Beispielprogramm. Dieses ist in keiner Art und Weise auch nur ansatzweise optimiert.
// Es sollten hier Funktionen inkludiert werden, die den Workflow erleichtern und somit dem Code an Komplexität nehmen

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <franka/exception.h>
#include <std_msgs/String.h>
#include <franka_control/ErrorRecoveryAction.h>

// Aus dem MoveIt!-Tutorium Pick & Place kopiert - ANFANG
void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL open_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.04;
  posture.points[0].positions[1] = 0.04;
  // END_SUB_TUTORIAL
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL closed_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.00;
  posture.points[0].positions[1] = 0.00;
  // END_SUB_TUTORIAL
}
// Aus dem MoveIt!-Tutorium Pick & Place kopiert -ENDE

int main(int argc, char** argv)
{
  //Das alles hier lebensnotwendig - ANFANG
  ros::init(argc, argv, "own_experiment");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP = "panda_arm";

  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Create a vector of grasps to be attempted, currently only creating single grasp.
  // This is essentially useful when using a grasp generator to generate and test multiple grasps.
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);
  
  // Noch nicht ganz klar, ob einer von Beiden reicht oder warum, weshalb & wieso ^^
  grasps[0].grasp_pose.header.frame_id = "panda_link0";
  grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "Z_FirstTry Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  
  // Das alles lebensnotwendig - ENDE

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // ERSTE POTENZIELLE FUNKTION - ANFANG

    // actionlib::SimpleActionClient<franka_control::ErrorRecoveryAction> recovery_action("franka_control/error_recovery/goal", true);
    // recovery_action.waitForServer();
    // franka_control::ErrorRecoveryActionGoal goalError;

  try{


  ros::Publisher error_recovery = node_handle.advertise<franka_control::ErrorRecoveryActionGoal>("/franka_control/error_recovery/goal", 3, false);
  franka_control::ErrorRecoveryActionGoal errorGoal;
  error_recovery.publish(errorGoal);
  sleep(3);

  // Planning to a joint-space goal
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Let's set a joint space goal and move towards it.  This will replace the
  // pose target we set above.
  //
  // To start, we'll create an pointer that references the current robot's state.
  // RobotState is the object that contains all the current position/velocity/acceleration data.
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  //
  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  // Es wird uns hier die Möglichkeit gegeben, alle Joints einzeln umzuändern. Dies wäre in der Applikation dann so zu verwenden,
  // dass die Joints durchiteriert werden, es wird geplant und dann kann die Bewegung ausgeführt werden. Wenn dies in einer Schleife
  // passiert, hört der Roboter zwar non-stop zu, aber dafür ist er immer einsatzbereit.
  joint_group_positions[0] = 0;  // radians
  joint_group_positions[1] = -0.7853;
  joint_group_positions[2] = 0;
  joint_group_positions[3] = -2.3561;
  joint_group_positions[4] = 0;
  joint_group_positions[5] = 1.5707;
  joint_group_positions[6] = 0.7853;

  move_group.setJointValueTarget(joint_group_positions);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (joint space goal) %s", success ? "" : "FAILED");

  // Cartesian motions are frequently needed to be slower for actions such as approach and retreat
  // grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
  // of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
  move_group.setMaxVelocityScalingFactor(0.1);

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  
  // Moving to a pose goal
  // ^^^^^^^^^^^^^^^^^^^^^
  //
  // Moving to a pose goal is similar to the step above
  // except we now use the move() function. Note that
  // the pose goal we had set earlier is still active
  // and so the robot will try to move to that goal. We will
  // not use that function in this tutorial since it is
  // a blocking function and requires a controller to be active
  // and report success on execution of a trajectory.

  move_group.move();

  current_state = move_group.getCurrentState();
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Es können die Werte via Pi-Values oder in "reine" Radiant angegeben werden. Durch den Verlusst von M_PI/M_PI_2/M_PI_4 etc.
  // ist die Argumentation somit obsolet. Eine Möglichkeit wäre diese Pi-Values als Konstanten einzufügen...
  // Radianten sind übrigens nur bis zum vierten bzw. fünften Nachkommawert speicherbar. Somit eine überschaubare Anzah.
  // Im weiteren Verlauf werden nur mehr Radianten verwendet. (Faulheit?)
  joint_group_positions[0] = 0;
  joint_group_positions[1] = (-25 * 3.1415926535897932384624433832795 /180);
  joint_group_positions[2] = 0;
  joint_group_positions[3] = (-140 * 3.1415926535897932384624433832795 /180);
  joint_group_positions[4] = 0;
  joint_group_positions[5] = (115 * 3.1415926535897932384624433832795 / 180);
  joint_group_positions[6] = ( 3.1415926535897932384624433832795 / 4);

  move_group.setJointValueTarget(joint_group_positions);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

  move_group.setMaxVelocityScalingFactor(0.1);

  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  move_group.move();

// Hier soll der Gripper aufmachen, ein Stückchen nach unten gehen und Gripper zu machen
  openGripper(grasps[0].pre_grasp_posture);

  ros::WallDuration(1.0).sleep();

  closedGripper(grasps[0].grasp_posture);

// Neue Bewegung!

  current_state = move_group.getCurrentState();
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  joint_group_positions[0] = 0;
  joint_group_positions[1] = -0.7853;
  joint_group_positions[2] = 0;
  joint_group_positions[3] = -2.3561;
  joint_group_positions[4] = 0;
  joint_group_positions[5] = 1.5707;
  joint_group_positions[6] = 0.7853;


  move_group.setJointValueTarget(joint_group_positions);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (joint space goal) %s", success ? "" : "FAILED");

  move_group.setMaxVelocityScalingFactor(0.1);

  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  move_group.move();

  }catch(franka::Exception& ex){
    ROS_INFO("Exception caught: %s", ex.what());
    // recovery_action.sendGoal();
    // ros::Publisher error_recovery = node_handle.advertise<std_msgs::String>("/franka_control/error_recovery/goal", 5);
    // std_msgs::String msg;
    // error_recovery.publish(msg);
    ROS_INFO("RecoveryAction sent");
  }

  // Auch wieder notwendig!!!

  ros::shutdown();
  return 0;
}
  
