Die zwei wichtigsten Folder sind MultilingualROSSpeechControl sowie niks_experiments. In Multilingual... befinden sich die zwei Programme STTNode.py und TTSNode.py. Im niks_experiments Ordner (unter src) StretchingSpeech.cpp. Es befinden sich hier noch mehrere andere Files, was daran liegt, dass dies nur eine Kopie von niks_experiments ist.

roslaunch franka_control franka_control.launch robot_ip:=192.168.13.1
roslaunch panda_moveit_config panda_moveit.launch
roslaunch panda_moveit_config moveit_rviz.launch


cd libfranka/rad_ws
source devel/setup.bash
rosrun niks_experiments StretchingSpeech




cd libfranka/rad_ws/src/MultilingualROSSpeechControl/
python TTSNode.py

cd libfranka/rad_ws/src/MultilingualROSSpeechControl/
python STTNode.py
