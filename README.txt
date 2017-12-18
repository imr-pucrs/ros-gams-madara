
INTRO:

  This directory has been created by the gpc.pl script and contains
  a custom simulation, GAMS controller, algorithms and platforms.

HOW TO:

  EDIT YOUR ALGORITHMS:
  
    Open square_patrol.cpp|h with your favorite programming environment / editor.
    Each method in your algorithm should be non-blocking or the call will
    block the controller. It is in your best interest to poll information from
    the environment and knowledge base, rather than blocking on an operating
    system call.

  EDIT YOUR PLATFORMS:
  
    Open MoveBasePlatform.cpp|h with your favorite programming environment / editor.
    Each method in your platform should be non-blocking or the call will
    block the controller. It is in your best interest to poll information from
    the environment and knowledge base, rather than blocking on an operating
    system call.
    
  INSTALL:
  
    Note: this turotiral works only on Ubuntu 16.04. To install GAMS/MADARA use the following commands on the terminal:
    
        export GAMS_ROOT=$HOME/gams
        export CORES=4
        git clone -b master --single-branch https://github.com/jredmondson/gams $GAMS_ROOT
        
  COMPILE ON LINUX:
  
        $GAMS_ROOT/scripts/linux/base_build.sh prereqs ace madara gams ros
        
  RUN THE SIMULATION:
  
        cd $HOME
        git clone https://github.com/marceloparavisi/turtlebot_gams.git
        cd $HOME/turtlebot_gams.git
            

TESTING PATROL
--------------

Each command below must be run in separate terminals:

TERMINAL 1
----------

roscore 


TERMINAL 2
----------

roslaunch turtlebot_gazebo turtlebot_world.launch


TERMINAL 3
----------

roslaunch turtlebot_navigation amcl_demo.launch map_file:=$PROJECT_HOME/turtlebot_gams/turtlebotmap3.yaml


TERMINAL 4
----------

roslaunch turtlebot_rviz_launchers view_navigation.launch 


TERMINAL 5
----------

cd $HOME/turtlebot_gams
source setup.sh
./action run
