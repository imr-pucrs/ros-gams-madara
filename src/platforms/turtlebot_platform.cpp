
#include "madara/knowledge/containers/NativeDoubleVector.h"
#include "turtlebot_platform.h"

gams::pose::CartesianFrame  platforms::turtlebot_platform::cartesian_frame;   
gams::pose::GPSFrame  platforms::turtlebot_platform::gps_frame;         
        
 
// factory class for creating a turtlebot_platform 
gams::platforms::BasePlatform *
platforms::turtlebot_platformFactory::create (
        const madara::knowledge::KnowledgeMap & args,
        madara::knowledge::KnowledgeBase * knowledge,
        gams::variables::Sensors * sensors,
        gams::variables::Platforms * platforms,
        gams::variables::Self * self)
{
  return new turtlebot_platform (knowledge, sensors, self);
}
        
// Constructor
platforms::turtlebot_platform::turtlebot_platform (
  madara::knowledge::KnowledgeBase * knowledge,
  gams::variables::Sensors * sensors,
  gams::variables::Self * self)
: //gams::platforms::BasePlatform (knowledge, sensors, self),
  gams::platforms::RosBase (knowledge, sensors, self),
  ros_namespace_("turtlebot"), 
  node_handle_ ("turtlebot"),
  move_client_ (std::string ("/move_base"), true) 
{
  // as an example of what to do here, create a coverage sensor
  if (knowledge && sensors)
  {
    // set the data plane for the threader
    //threader_.set_data_plane (*knowledge);
  
    // create a coverage sensor
    gams::variables::Sensors::iterator it = sensors->find ("coverage");
    if (it == sensors->end ()) // create coverage sensor
    {
      // get origin
      gams::pose::Position origin (gams::pose::gps_frame());
      madara::knowledge::containers::NativeDoubleArray origin_container;
      origin_container.set_name ("sensor.coverage.origin", *knowledge, 3);
      origin.from_container (origin_container);

      // establish sensor
      gams::variables::Sensor* coverage_sensor =
        new gams::variables::Sensor ("coverage", knowledge, 2.5, origin);
      (*sensors)["coverage"] = coverage_sensor;
    }
    (*sensors_)["coverage"] = (*sensors)["coverage"];
    status_.init_vars (*knowledge, get_id ());
    
    // create threads
    // end create threads
    
    
    /**
    * the following should be set when movement is available in your
    * platform. If on construction, movement should be possible, then
    * feel free to keep this uncommented. Otherwise, set it somewhere else
    * in analyze or somewhere else when appropriate to enable movement.
    * If you never enable movement_available, movement based algorithms are
    * unlikely to ever move with your platform.
    **/
    status_.movement_available = 1;
  }
  else
  {
	madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "platforms::turtlebot_platform::turtlebot_platform" 
      " knowledge=%p, sensors=%p\n",
      knowledge, sensors);
  }
}


// Destructor
platforms::turtlebot_platform::~turtlebot_platform ()
{
  //threader_.terminate ();
  //threader_.wait ();
}


// Polls the sensor environment for useful information. Required.
int platforms::turtlebot_platform::sense (void)
{
  return gams::platforms::PLATFORM_OK;
}


// Analyzes platform information. Required.
int
platforms::turtlebot_platform::analyze (void)
{
  return gams::platforms::PLATFORM_OK;
}


// Gets the name of the platform. Required.
std::string
platforms::turtlebot_platform::get_name () const
{
  return "turtlebot_platform";
}


// Gets the unique identifier of the platform.
std::string
platforms::turtlebot_platform::get_id () const
{
  return "turtlebot_platform";
}


// Gets the position accuracy in meters. Optional.
double
platforms::turtlebot_platform::get_accuracy (void) const
{
  // will depend on your localization capabilities for robotics
  return 0.0;
}

// Gets Location of platform, within its parent frame. Optional.
gams::pose::Position
platforms::turtlebot_platform::get_location () const
{
  gams::pose::Position result;
  
  return result;
}


// Gets Rotation of platform, within its parent frame. Optional.
gams::pose::Orientation
platforms::turtlebot_platform::get_orientation () const
{
  gams::pose::Orientation result;
  
  return result;
}


// Gets sensor radius. Optional.
double
platforms::turtlebot_platform::get_min_sensor_range () const
{
  // should be in square meters
  return 0.0;
}

// Gets move speed. Optional.
double
platforms::turtlebot_platform::get_move_speed () const
{
  // should be in meters/s
  return 0.0;
}

// Instructs the agent to return home. Optional.
int
platforms::turtlebot_platform::home (void)
{
  /**
   * Movement functions work in a polling manner. In a real implementation,
   * we would want to check a finite state machine, external thread or
   * platform status to determine what to return. For now, we will simply
   * return that we are in the process of moving to the final pose.
   **/
  return gams::platforms::PLATFORM_IN_PROGRESS;
}


// Instructs the agent to land. Optional.
int
platforms::turtlebot_platform::land (void)
{
  /**
   * Movement functions work in a polling manner. In a real implementation,
   * we would want to check a finite state machine, external thread or
   * platform status to determine what to return. For now, we will simply
   * return that we are in the process of moving to the final pose.
   **/
  return gams::platforms::PLATFORM_IN_PROGRESS;
}


// Moves the platform to a location. Optional.
int
platforms::turtlebot_platform::move (
  const gams::pose::Position & location,
  double epsilon)
{
  /**
   * Movement functions work in a polling manner. In a real implementation,
   * we would want to check a finite state machine, external thread or
   * platform status to determine what to return. For now, we will simply
   * return that we are in the process of moving to the final pose.
   **/
  // generate message
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,"\n\n ------------- RECEIVED REQUEST TO MOVE!!!\n\n");
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "/map";
  goal.target_pose.header.stamp = ros::Time::now ();
  goal.target_pose.pose.position.x = location.x ();
  goal.target_pose.pose.position.y = location.y ();
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;
  
  // send the goal
  ROS_INFO("Sending goal");
  move_client_.sendGoal(goal);//*/


  return gams::platforms::PLATFORM_MOVING;
}


// Rotates the platform to match a given angle. Optional.
int
platforms::turtlebot_platform::rotate (
  const gams::pose::Orientation & target,
  double epsilon)
{
  /**
   * Movement functions work in a polling manner. In a real implementation,
   * we would want to check a finite state machine, external thread or
   * platform status to determine what to return. For now, we will simply
   * return that we are in the process of moving to the final pose.
   **/
  return gams::platforms::PLATFORM_MOVING;
}


// Moves the platform to a pose (location and rotation). Optional.
int
platforms::turtlebot_platform::pose (const gams::pose::Pose & target,
  double loc_epsilon, double rot_epsilon)
{
  /**
   * Movement functions work in a polling manner. In a real implementation,
   * we would want to check a finite state machine, external thread or
   * platform status to determine what to return. For now, we will simply
   * return that we are in the process of moving to the final pose.
   **/
  return gams::platforms::PLATFORM_MOVING;
}

// Pauses movement, keeps source and dest at current values. Optional.
void
platforms::turtlebot_platform::pause_move (void)
{
}


// Set move speed. Optional.
void
platforms::turtlebot_platform::set_move_speed (const double& speed)
{
}


// Stops movement, resetting source and dest to current location. Optional.
void
platforms::turtlebot_platform::stop_move (void)
{
}

// Instructs the agent to take off. Optional.
int
platforms::turtlebot_platform::takeoff (void)
{
  return gams::platforms::PLATFORM_OK;
}

const gams::pose::ReferenceFrame &
platforms::turtlebot_platform::get_frame (void) const
{
  return gps_frame;
}
