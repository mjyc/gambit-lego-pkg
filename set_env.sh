# use below command to set environmental variables
# . ./set_env.sh

alias set_rosmasteruri_chessbot="export ROS_MASTER_URI=http://chess-laptop.dyn.cs.washington.edu:11311;set_ros_ip;set_ros_hostname"
alias set_rosmasteruri_chessarmbox="export ROS_MASTER_URI=http://chess-armbox.dyn.cs.washington.edu:11311;set_ros_ip;set_ros_hostname"

ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/ros-repos/gambit-lego-pkg/
