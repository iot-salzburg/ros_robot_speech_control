// Es handelt sich hierbei um ein Beispielprogramm. Dieses ist in keiner Art und Weise auch nur ansatzweise optimiert.
// Es sollten hier Funktionen inkludiert werden, die den Workflow erleichtern und somit dem Code an Komplexität nehmen

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <control_msgs/GripperCommandAction.h>
#include <franka_gripper/franka_gripper.h>

namespace rvt = rviz_visual_tools;

void moveFunction(std::vector<double> joint_group_positions, const robot_state::JointModelGroup* joint_model_group,
moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose){
  moveit::core::RobotStatePtr current_state = move_group->getCurrentState();
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  move_group->setJointValueTarget(joint_group_positions);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (joint space goal) %s", success ? "" : "FAILED");

  move_group->setMaxVelocityScalingFactor(speed);

  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  
  move_group->move();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool moveToInitialPosition(const robot_state::JointModelGroup* joint_model_group,
moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose){

    std::vector<double> joint_group_positions =
              {-0.000207,            // Joint 1
               -0.785368,            // Joint 2
               -0.000246,            // Joint 3
               -2.356503,           // Joint 4
               +0.000946,           // Joint 5
               +1.570938,            // Joint 6
               +0.784972             // Joint 7  
               };

    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool moveToPrinter(const robot_state::JointModelGroup* joint_model_group,
moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose){
     
    // Fängt an das Objekt vom Drucker zu heben
   
    // Erste Position
std::vector<double> joint_group_positions = 
              {-2.136935,            // Joint 1
                -0.829897,            // Joint 2
                +2.508779,            // Joint 3
                -1.023524,           // Joint 4
                +0.956515,           // Joint 5
                +0.849816,            // Joint 6
                +1.410623             // Joint 7  
                };

    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

    // Position direkt beim Drucker
     joint_group_positions = 
              {-2.021821,            // Joint 1
                -1.397909,            // Joint 2
                +2.402339,            // Joint 3
                -1.073298,           // Joint 4
                +1.213987,           // Joint 5
                +1.125178,            // Joint 6
                +1.118048             // Joint 7  
                };
 
    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool moveFromPrinter(const robot_state::JointModelGroup* joint_model_group,
moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose){
     
    // Fängt an das Objekt vom Drucker zu heben
   
    // Erste Position
std::vector<double> joint_group_positions = 
              {-2.136935,            // Joint 1
                -0.829897,            // Joint 2
                +2.508779,            // Joint 3
                -1.023524,           // Joint 4
                +0.956515,           // Joint 5
                +0.849816,            // Joint 6
                +1.410623             // Joint 7  
                };
   
       moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);
   return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool moveToOutput(const robot_state::JointModelGroup* joint_model_group,
moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose){
   
    // Erste Position
std::vector<double> joint_group_positions = 
              {-1.675980,            // Joint 1
                -0.067966 ,            // Joint 2
                +0.801821,            // Joint 3
                -1.808331,           // Joint 4
                -0.241695,           // Joint 5
                +2.056401,            // Joint 6
                -1.637750              // Joint 7  
                };
    
    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

// Zweite Position
     joint_group_positions = 
              {-1.521378,            // Joint 1
                -0.678895,            // Joint 2
                +0.988494,            // Joint 3
                -2.171208,           // Joint 4
                +0.300746,           // Joint 5
                +1.933613,            // Joint 6
                -1.462331              // Joint 7  
                };
    
    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

// Position über dem Förderband
     joint_group_positions = 
              {-2.600428,            // Joint 1
                -0.694723,            // Joint 2
                +2.300770,            // Joint 3
                -2.332446,           // Joint 4
                -1.497476,           // Joint 5
                +2.798764,           // Joint 6
                +0.365014             // Joint 7  
                };

    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);
   return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool moveFromOutput(const robot_state::JointModelGroup* joint_model_group,
moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose){
   
    // Erste Position
   std::vector<double>  joint_group_positions = 
              {-1.521378,            // Joint 1
                -0.678895,            // Joint 2
                +0.988494,            // Joint 3
                -2.171208,           // Joint 4
                +0.300746,           // Joint 5
                +1.933613,            // Joint 6
                -1.462331              // Joint 7  
                };
    
    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);
return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool moveToStorage(const robot_state::JointModelGroup* joint_model_group,
moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose){ // Passt

// Hier ne Position um vor dem Regal zu "Schweben"
std::vector<double> joint_group_positions = 
              {-0.099726,            // Joint 1
                +1.237739,            // Joint 2
                -1.972287,            // Joint 3
                -2.136524,           // Joint 4
                +0.355537,           // Joint 5
                +2.225301,            // Joint 6
                -1.403869             // Joint 7  
                };
  
    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Alle 9 Funktionen, die für das Finden des richtigen Lagerplatzes gedacht sind, fahren folgendes Muster:
// *Von Regal schwebend zu Platz schwebend
// *Von Platz schwebend zu ablegen/aufnehmen
// *Von ablegen/aufnehmen zu Platz schwebend

bool findPlaceOne(const robot_state::JointModelGroup* joint_model_group,
moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose){


 std::vector<double> joint_group_positions = 
              {-0.516120,            // Joint 1
                +0.820823,            // Joint 2
                -1.837487,            // Joint 3
                -1.347663,           // Joint 4
                +0.079926,           // Joint 5
                +1.910257,            // Joint 6
                -1.434026             // Joint 7  
                };
 
    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

               joint_group_positions = 
              {-0.632862,            // Joint 1
                +0.793258,            // Joint 2
                -1.771385,            // Joint 3
                -1.542701,           // Joint 4
                +0.047447,           // Joint 5
                +2.234886,            // Joint 6
                -1.452819             // Joint 7  
                };
 
    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

return true;
}

bool leavePlaceOne(const robot_state::JointModelGroup* joint_model_group,
moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose){

 std::vector<double> joint_group_positions = 
              {-0.516120,            // Joint 1
                +0.820823,            // Joint 2
                -1.837487,            // Joint 3
                -1.347663,           // Joint 4
                +0.079926,           // Joint 5
                +1.910257,            // Joint 6
                -1.434026             // Joint 7  
                };
 
    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);
return true;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool findPlaceTwo(const robot_state::JointModelGroup* joint_model_group,
moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose){

 std::vector<double> joint_group_positions = 
              {-0.256274,            // Joint 1
                +0.859337,            // Joint 2
                -2.170525,            // Joint 3
                -1.627315,           // Joint 4
                +0.280060,           // Joint 5
                +1.833001,            // Joint 6
                -1.646919             // Joint 7  
                };
  
    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

     joint_group_positions = 
              {-0.337015,            // Joint 1
                +0.925032,            // Joint 2
                -2.101915,            // Joint 3
                -1.935825,           // Joint 4
                +0.227259,           // Joint 5
                +2.343030,            // Joint 6
                -1.684558            // Joint 7  
                };

    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

return true;
}

bool leavePlaceTwo(const robot_state::JointModelGroup* joint_model_group,
moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose){

 std::vector<double> joint_group_positions = 
              {-0.256274,            // Joint 1
                +0.859337,            // Joint 2
                -2.170525,            // Joint 3
                -1.627315,           // Joint 4
                +0.280060,           // Joint 5
                +1.833001,            // Joint 6
                -1.646919             // Joint 7  
                };
 
    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

return true;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool findPlaceThree(const robot_state::JointModelGroup* joint_model_group,
moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose){

 std::vector<double> joint_group_positions = 
              {-0.000185,            // Joint 1
                +0.976473,            // Joint 2
                -2.466721,            // Joint 3
                -1.934626,           // Joint 4
                +0.272974,           // Joint 5
                +1.871381,            // Joint 6
                -1.827909             // Joint 7  
                };
  
    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

     joint_group_positions = 
              {-0.135728,            // Joint 1
                +1.045247,            // Joint 2
                -2.334923,            // Joint 3
                -2.235124,           // Joint 4
                +0.262659,           // Joint 5
                +2.419076,            // Joint 6
                -1.839854            // Joint 7  
                };
  
    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

return true;
}

void leavePlaceThree(const robot_state::JointModelGroup* joint_model_group,
moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose){

 std::vector<double> joint_group_positions = 
              {-0.000185,            // Joint 1
                +0.976473,            // Joint 2
                -2.466721,            // Joint 3
                -1.934626,           // Joint 4
                +0.272974,           // Joint 5
                +1.871381,            // Joint 6
                -1.827909             // Joint 7  
                };
 
    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool findPlaceSeven(const robot_state::JointModelGroup* joint_model_group,
moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose){

 std::vector<double> joint_group_positions = 
              {-0.428316,            // Joint 1
                +1.122033,            // Joint 2
                -1.603507,            // Joint 3
                -1.884648,           // Joint 4
                +0.362244,           // Joint 5
                +2.221434,            // Joint 6
                -1.200215             // Joint 7  
                };
 
    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

     joint_group_positions = 
              {-0.663267,            // Joint 1
                +1.130119,            // Joint 2
                -1.470890,            // Joint 3
                -1.948712,           // Joint 4
                +0.512178,           // Joint 5
                +2.588224,            // Joint 6
                -1.428654             // Joint 7  
                };
 
    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

return true;
}

bool leavePlaceSeven(const robot_state::JointModelGroup* joint_model_group,
moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose){

 std::vector<double> joint_group_positions = 
              {-0.428316,            // Joint 1
                +1.122033,            // Joint 2
                -1.603507,            // Joint 3
                -1.884648,           // Joint 4
                +0.362244,           // Joint 5
                +2.221434,            // Joint 6
                -1.200215             // Joint 7  
                };
 
    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool findPlaceEight(const robot_state::JointModelGroup* joint_model_group,
moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose){

 std::vector<double> joint_group_positions = 
              {-0.145144,            // Joint 1
                +1.120037,            // Joint 2
                -1.758764,            // Joint 3
                -2.318438,           // Joint 4
                +0.466797,           // Joint 5
                +2.419395,            // Joint 6
                -1.436929             // Joint 7  
                };

    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

     joint_group_positions = 
              {-0.391746,            // Joint 1
                +1.030484,            // Joint 2
                -1.582915,            // Joint 3
                -2.403232,           // Joint 4
                +0.616726,           // Joint 5
                +2.841888,            // Joint 6
                -1.605796             // Joint 7  
                };
 
    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

return true;
}

bool leavePlaceEight(const robot_state::JointModelGroup* joint_model_group,
moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose){

 std::vector<double> joint_group_positions = 
              {-0.145144,            // Joint 1
                +1.120037,            // Joint 2
                -1.758764,            // Joint 3
                -2.318438,           // Joint 4
                +0.466797,           // Joint 5
                +2.419395,            // Joint 6
                -1.436929             // Joint 7  
                };
 
    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

return true;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool findPlaceNine(const robot_state::JointModelGroup* joint_model_group,
moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose){

 std::vector<double> joint_group_positions = 
              {+0.222048,            // Joint 1
                +1.211415,            // Joint 2
                -2.023364,            // Joint 3
                -2.721475,           // Joint 4
                +0.524792,           // Joint 5
                +2.463421,            // Joint 6
                -1.656938             // Joint 7  
                };
 
    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

     joint_group_positions = 
              {-0.174449,            // Joint 1
                +1.002473,            // Joint 2
                -1.725353,            // Joint 3
                -2.705326,           // Joint 4
                +0.615969,           // Joint 5
                +2.954883,            // Joint 6
                -1.709142             // Joint 7  
                };
 
    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

return true;
}

void leavePlaceNine(const robot_state::JointModelGroup* joint_model_group,
moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose){

 std::vector<double> joint_group_positions = 
              {+0.222048,            // Joint 1
                +1.211415,            // Joint 2
                -2.023364,            // Joint 3
                -2.721475,           // Joint 4
                +0.524792,           // Joint 5
                +2.463421,            // Joint 6
                -1.656938             // Joint 7  
                };
 
    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool findPlaceFour(const robot_state::JointModelGroup* joint_model_group,
moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose){

 std::vector<double> joint_group_positions = 
              {-0.427535,            // Joint 1
                +0.979701,            // Joint 2
                -1.802566,            // Joint 3
                -1.729782,           // Joint 4
                +0.396360,           // Joint 5
                +2.114495,            // Joint 6
                -1.510005             // Joint 7  
                };
 
    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

     joint_group_positions = 
              {-0.581477,            // Joint 1
                +1.065810,            // Joint 2
                -1.718917,            // Joint 3
                -1.883813,           // Joint 4
                +0.380246,           // Joint 5
                +2.482093,            // Joint 6
                -1.476558             // Joint 7  
                };

    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);


return true;
}

void leavePlaceFour(const robot_state::JointModelGroup* joint_model_group,
moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose){

 std::vector<double> joint_group_positions = 
              {-0.427535,            // Joint 1
                +0.979701,            // Joint 2
                -1.802566,            // Joint 3
                -1.729782,           // Joint 4
                +0.396360,           // Joint 5
                +2.114495,            // Joint 6
                -1.510005             // Joint 7  
                };
 
    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool findPlaceFive(const robot_state::JointModelGroup* joint_model_group,
moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose){

 std::vector<double> joint_group_positions = 
              {-0.064589,            // Joint 1
                +1.006084,            // Joint 2
                -2.079785,            // Joint 3
                -2.105988,           // Joint 4
                +0.302475,           // Joint 5
                +2.155265,            // Joint 6
                -1.555202             // Joint 7  
                };
 
    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

     joint_group_positions = 
              {-0.295086,            // Joint 1
                +0.921615,            // Joint 2
                -1.920725,            // Joint 3
                -2.197950,           // Joint 4
                +0.302841,           // Joint 5
                +2.526494,            // Joint 6
                -1.580266             // Joint 7  
                };
   
    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

return true;
}

void leavePlaceFive(const robot_state::JointModelGroup* joint_model_group,
moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose){

 std::vector<double> joint_group_positions = 
              {-0.064589,            // Joint 1
                +1.006084,            // Joint 2
                -2.079785,            // Joint 3
                -2.105988,           // Joint 4
                +0.302475,           // Joint 5
                +2.155265,            // Joint 6
                -1.555202             // Joint 7  
                };
 
    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool findPlaceSix(const robot_state::JointModelGroup* joint_model_group,
moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose){

 std::vector<double> joint_group_positions = 
              {+0.045320,            // Joint 1
                +1.277773,            // Joint 2
                -2.265168,            // Joint 3
                -2.379450,           // Joint 4
                +0.353567,           // Joint 5
                +2.044280,            // Joint 6
                -1.691176             // Joint 7  
                };

    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

     joint_group_positions = 
              {-0.034831,            // Joint 1
                +1.023587,            // Joint 2
                -2.185950,            // Joint 3
                -2.526037,           // Joint 4
                +0.361866,           // Joint 5
                +2.577958,            // Joint 6
                -1.790957             // Joint 7  
                };
  
    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

return true;
}

void leavePlaceSix(const robot_state::JointModelGroup* joint_model_group,
moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose){

 std::vector<double> joint_group_positions = 
              {+0.045320,            // Joint 1
                +1.277773,            // Joint 2
                -2.265168,            // Joint 3
                -2.379450,           // Joint 4
                +0.353567,           // Joint 5
                +2.044280,            // Joint 6
                -1.691176             // Joint 7  
                };
 
    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool findRightSpot(const robot_state::JointModelGroup* joint_model_group,
moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose, int place){


// Hier die Abfrage
// wenn 1 - 3: oberes Regal
//    auf Position des jeweiligen Plates fahren (davor schweben)
//    auf Ablageposition des Plates fahren und Gripper öffnen
// wenn 4 - 6: keine Änderung
//    auf Position des jeweiligen Plates fahren (davor schweben)
//    auf Ablageposition des Plates fahren und Gripper öffnen
// wenn 7 - 9: unteres Regal fahren
//    auf Position des jeweiligen Plates fahren (davor schweben)
//    auf Ablageposition des Plates fahren und Gripper öffnenbool notCorrectPosition = true;
   

if (place < 4) {
// Hier Position für erstes Fach
    std::vector<double> joint_group_positions = 
              {-0.202298,            // Joint 1
                +1.118617,            // Joint 2
                -2.215788,            // Joint 3
                -1.792656,           // Joint 4
                +0.262851,           // Joint 5
                +1.940611,            // Joint 6
                -1.581900             // Joint 7  
                };
 
    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);


  if(place == 1) {
    findPlaceOne(joint_model_group, move_group, visual_tools, speed, text_pose);
  }
  else if(place == 2) {
  // Hier Position für zweiten Platz
    findPlaceTwo(joint_model_group, move_group, visual_tools, speed, text_pose);
  }
  else{
  // Hier Position für dritten Platz
    findPlaceThree(joint_model_group, move_group, visual_tools, speed, text_pose);
  }
}
else if (place > 6) {
// Hier Position für drittes Fach

    std::vector<double> joint_group_positions = 
              {-0.087449,            // Joint 1
                +1.312051,            // Joint 2
                -1.758231,            // Joint 3
                -2.331742,           // Joint 4
                +0.478561,           // Joint 5
                +2.410676,            // Joint 6
                -1.306896             // Joint 7  
                }; 

    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

  if(place == 7) {
    findPlaceSeven(joint_model_group, move_group, visual_tools, speed, text_pose);  

  }
  else if(place == 8) {
    findPlaceEight(joint_model_group, move_group, visual_tools, speed, text_pose);

  }
  else{
    findPlaceNine(joint_model_group, move_group, visual_tools, speed, text_pose);

  }
}
else {

  if(place == 4) {
  findPlaceFour(joint_model_group, move_group, visual_tools, speed, text_pose);
  }
  else if(place == 5) {
  findPlaceFive(joint_model_group, move_group, visual_tools, speed, text_pose);
  }
  else{
  findPlaceSix(joint_model_group, move_group, visual_tools, speed, text_pose);
  }
}
return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool leaveRightSpot(const robot_state::JointModelGroup* joint_model_group,
moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose, int place){

if (place < 4) {
  if(place == 1) {
    leavePlaceOne(joint_model_group, move_group, visual_tools, speed, text_pose);
  }
  else if(place == 2) {
    leavePlaceTwo(joint_model_group, move_group, visual_tools, speed, text_pose);
  }
  else{
    leavePlaceThree(joint_model_group, move_group, visual_tools, speed, text_pose);
  }
}
else if (place > 6) {
  if(place == 7) {
    leavePlaceSeven(joint_model_group, move_group, visual_tools, speed, text_pose);  
  }
  else if(place == 8) {
    leavePlaceEight(joint_model_group, move_group, visual_tools, speed, text_pose);
  }
  else{
    leavePlaceNine(joint_model_group, move_group, visual_tools, speed, text_pose);
  }
}
else {

  if(place == 4) {
  leavePlaceFour(joint_model_group, move_group, visual_tools, speed, text_pose);
  }
  else if(place == 5) {
  leavePlaceFive(joint_model_group, move_group, visual_tools, speed, text_pose);
  }
  else{
  leavePlaceSix(joint_model_group, move_group, visual_tools, speed, text_pose);
  }
}
return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool moveFromStorage(const robot_state::JointModelGroup* joint_model_group,
moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose, int place){

if (place < 4) {
// Hier Position für erstes Fach
    std::vector<double> joint_group_positions =
              {-0.202298,            // Joint 1
                +1.118617,            // Joint 2
                -2.215788,            // Joint 3
                -1.792656,           // Joint 4
                +0.262851,           // Joint 5
                +1.940611,            // Joint 6
                -1.581900             // Joint 7  
                };

    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

}
else if (place > 6) {
// Hier Position für drittes Fach
    std::vector<double> joint_group_positions =
              {-0.087449,            // Joint 1
                +1.312051,            // Joint 2
                -1.758231,            // Joint 3
                -2.331742,           // Joint 4
                +0.478561,           // Joint 5
                +2.410676,            // Joint 6
                -1.306896             // Joint 7  
                };     

    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

}

// In Schwebeposition zurück fahren
    std::vector<double> joint_group_positions = 
              {-0.099726,            // Joint 1
                +1.237739,            // Joint 2
                -1.972287,            // Joint 3
                -2.136524,           // Joint 4
                +0.355537,           // Joint 5
                +2.225301,            // Joint 6
                -1.403869             // Joint 7  
                };

    moveFunction(joint_group_positions, joint_model_group, move_group, visual_tools, speed, text_pose);

return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool getBlockFromPrinterToOutput(const robot_state::JointModelGroup* joint_model_group,
moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose){

  moveToPrinter(joint_model_group, move_group, visual_tools, speed, text_pose);

  moveFromPrinter(joint_model_group, move_group, visual_tools, speed, text_pose);
  moveToOutput(joint_model_group, move_group, visual_tools, speed, text_pose);

  moveFromOutput(joint_model_group, move_group, visual_tools, speed, text_pose);
  moveToInitialPosition(joint_model_group, move_group, visual_tools, speed, text_pose);

  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool getBlockFromPrinterToStorage(const robot_state::JointModelGroup* joint_model_group,
moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose, int place){
    
  moveToPrinter(joint_model_group, move_group, visual_tools, speed, text_pose);
// Hier vielleicht noch Texte, was beschreiben was passiert?

  moveFromPrinter(joint_model_group, move_group, visual_tools, speed, text_pose);
  moveToStorage(joint_model_group, move_group, visual_tools, speed, text_pose);
  findRightSpot(joint_model_group, move_group, visual_tools, speed, text_pose, place);

  leaveRightSpot(joint_model_group, move_group, visual_tools, speed, text_pose, place);    
  moveFromStorage(joint_model_group, move_group, visual_tools, speed, text_pose, place);

  moveToInitialPosition(joint_model_group, move_group, visual_tools, speed, text_pose);

  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool getBlockFromStorageToOutput(const robot_state::JointModelGroup* joint_model_group,
moveit::planning_interface::MoveGroupInterface* move_group, moveit_visual_tools::MoveItVisualTools visual_tools, float speed,   Eigen::Affine3d text_pose, int place){
   
  moveToStorage(joint_model_group, move_group, visual_tools, speed, text_pose);
  findRightSpot(joint_model_group, move_group, visual_tools, speed, text_pose, place);

  leaveRightSpot(joint_model_group, move_group, visual_tools, speed, text_pose, place);
  moveFromStorage(joint_model_group, move_group, visual_tools, speed, text_pose, place);
  moveToOutput(joint_model_group, move_group, visual_tools, speed, text_pose);

  moveFromOutput(joint_model_group, move_group, visual_tools, speed, text_pose);
  moveToInitialPosition(joint_model_group, move_group, visual_tools, speed, text_pose);

  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  //Das alles hier lebensnotwendig - ANFANG: #PfotenWeg, #WeheDir, #IToldYouSo
  ros::init(argc, argv, "Stretching");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "panda_arm";

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();

  visual_tools.loadRemoteControl();

  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "Z_FirstTry Demo", rvt::WHITE, rvt::XLARGE);

  visual_tools.trigger();

  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  
  // Das alles lebensnotwendig - ENDE
// Ab hier werden die einzelnen Funktionen aufgerufen

double speed = 0.14;

getBlockFromPrinterToOutput(joint_model_group, &move_group, visual_tools, speed, text_pose);

for(int i=1; i<10; i++){
  getBlockFromPrinterToStorage(joint_model_group, &move_group, visual_tools, speed, text_pose, i);
}
for(int i=1; i<10; i++){
  getBlockFromStorageToOutput(joint_model_group, &move_group, visual_tools, speed, text_pose, i);
}
  // Auch wieder notwendig!!!

  ros::shutdown();
  return 0;
}
  
