# use below command to set environmental variables
# . ./set_env.sh
source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}~/Projects/HIL/codes/ros/stacks/ros-pkg-nsl/trunk/gambit/gambit_gazebo_driver/models/:
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}~/Projects/HIL/codes/ros/stacks/ros-pkg-nsl/trunk/gambit/gambit_gazebo_driver/lib: