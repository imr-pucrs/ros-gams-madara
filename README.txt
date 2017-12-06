
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
  
    Open turtlebot_platform.cpp|h with your favorite programming environment / editor.
    Each method in your platform should be non-blocking or the call will
    block the controller. It is in your best interest to poll information from
    the environment and knowledge base, rather than blocking on an operating
    system call.

  COMPILE ON LINUX:
  
    mwc.pl -type gnuace workspace.mwc
    make vrep=1
    
  COMPILE ON WINDOWS:
  
    mwc.pl -type vc12 workspace.mwc 
    
    <Open Visual Studio and compile project>. Note that you should compile the
    solution in the same settings you used for ACE, MADARA, and GAMS. For most
    users, this is probably Release mode for x64. However, whatever you use
    make sure that you have compiled ACE, MADARA, and GAMS in those settings.
    
  RUN THE SIMULATION:
    open VREP simulator
    perl sim/run.pl 
    

TESTING PATROL
--------------

roscore 
roslaunch turtlebot_gazebo turtlebot_world.launch
roslaunch turtlebot_navigation amcl_demo.launch map_file:=/home/paravisi/turtlebotmap2.yaml
roslaunch turtlebot_rviz_launchers view_navigation.launch 
./action run
