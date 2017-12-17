roslaunch turtlebot_gazebo turtlebot_world.launch &
roslaunch turtlebot_navigation amcl_demo.launch map_file:=$(pwd)/turtlebotmap3.yaml &
roslaunch turtlebot_rviz_launchers view_navigation.launch
echo "preparando para fechar todos os roslaunchs"
kill $(pgrep roslaunch)
echo "fim..."
