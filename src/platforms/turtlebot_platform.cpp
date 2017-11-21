
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
  firstMoveSent = false;
  move_client_.waitForServer();

  updateServiceClientMoveBase = node_handle_.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/DWAPlannerROS/set_parameters");
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
  madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_MAJOR,"\n\n ------------- turtlebot_platform::analyze::status_: %d\n\n", status_);
  if (move_client_.isServerConnected() == false)
	return gams::platforms::UNKNOWN;
  if (!firstMoveSent)
	return gams::platforms::MOVEMENT_AVAILABLE;

  if ((move_client_.getState().toString().compare("ACTIVE")==0) )//|| (move_client_.getState().toString().compare("PENDING")==0))
  {
        madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_MAJOR,"\n\n ------------- Still trying to move!!! Please wait...%s\n\n", move_client_.getState().toString().c_str());
	cleanAllStatus();
	status_.moving = 1;
	return gams::platforms::MOVING;
  }
  else   if (move_client_.getState().toString().compare("PENDING")==0)
  {
	madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_MAJOR,"\n\n ------------- PLATFORM PENDING!!!...\n\n");
	cleanAllStatus();
        status_.waiting = 1;
	return gams::platforms::WAITING;
  }
  else   if (move_client_.getState().toString().compare("SUCCEEDED")==0)
  {
	madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_MAJOR,"\n\n ------------- PLATFORM ARRIVED!!!...\n\n");
	cleanAllStatus();
        status_.movement_available = 1;
	return gams::platforms::MOVEMENT_AVAILABLE;

  }
  else   if (move_client_.getState().toString().compare("ABORTED")==0)
  {
	madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_MAJOR,"\n\n ------------- PLATFORM ABORTED!!!...\n\n");
	cleanAllStatus();
        status_.failed = 1;
	return gams::platforms::FAILED;
	
  }
  else   if (move_client_.getState().toString().compare("REJECTED")==0)
  {
	madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_MAJOR,"\n\n ------------- PLATFORM REJECTED THE GOAL!!!...\n\n");
	cleanAllStatus();
        status_.reduced_movement = 1;
	return gams::platforms::REDUCED_MOVEMENT_AVAILABLE;

  }
  else   if (move_client_.getState().toString().compare("RECALLED")==0)
  {
	madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_MAJOR,"\n\n ------------- PLATFORM RECALLED THE GOAL!!!...\n\n");
	cleanAllStatus();
        status_.ok = 1;
	return gams::platforms::OK;

  }
  else   if (move_client_.getState().toString().compare("PREEMPTED")==0)
  {
	madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_MAJOR,"\n\n ------------- PLATFORM PREEMPTED THE GOAL!!!...\n\n");
	cleanAllStatus();
        status_.ok = 1;
	return gams::platforms::OK;

  }
  else   if (move_client_.getState().toString().compare("RECALLING")==0)
  {
	madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_MAJOR,"\n\n ------------- PLATFORM RECALLING !!!...\n\n");
	cleanAllStatus();
        status_.waiting = 1;
	return gams::platforms::WAITING;
  }
  else   if (move_client_.getState().toString().compare("PREEMPTING")==0)
  {
	madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_MAJOR,"\n\n ------------- PLATFORM PREEMPTING !!!...\n\n");
	cleanAllStatus();
        status_.waiting = 1;
	return gams::platforms::WAITING;
  }
  madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_MAJOR,"\n\n ------------- moveclient %s...\n\n", move_client_.getState().toString().c_str());

  return gams::platforms::UNKNOWN;
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
   * platform status_ to determine what to return. For now, we will simply
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
   * platform status_ to determine what to return. For now, we will simply
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
   * platform status_ to determine what to return. For now, we will simply
   * return that we are in the process of moving to the final pose.
   **/
  // generate message
	firstMoveSent = true;
	madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_MAJOR,"\n\n ------------- RECEIVED REQUEST TO MOVE!!!\n\n");
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now ();
	goal.target_pose.pose.position.x = location.x ();
	goal.target_pose.pose.position.y = location.y ();
	goal.target_pose.pose.position.z = 0;
	/*double dist = math::sqrt(location.x()*location.x()+location.y()*location.y()+location.z()*location.z());
	double cossenoTheta = OrientationVector.x()/dist;
	
	goal.target_pose.pose.orientation.x = quat.x();
	goal.target_pose.pose.orientation.y = quat.y();
	goal.target_pose.pose.orientation.z = quat.z();
	goal.target_pose.pose.orientation.w = math::sin(Theta)*quat.w()+cossenoTheta;*/
	goal.target_pose.pose.orientation.x = 0;
	goal.target_pose.pose.orientation.y = 0;
	goal.target_pose.pose.orientation.z = 0;
	goal.target_pose.pose.orientation.w = 1;
	
	lastWaypoint=location;
	



	// send the goal
	move_client_.sendGoal(goal);

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
   * platform status_ to determine what to return. For now, we will simply
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
   * platform status_ to determine what to return. For now, we will simply
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

	dynamic_reconfigure::Reconfigure srv;
	dynamic_reconfigure::DoubleParameter double_param;
	double_param.name = "max_vel_x";
	double_param.value = speed;
	srv.request.config.doubles.push_back(double_param);

	if (updateServiceClientMoveBase.call(srv))
	{
		madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_MAJOR,"\n\n ---- SUCCESS: speed changed %f!!!\n\n", speed);	
	}
	else
		madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_MAJOR,"\n\n ---- ERROR: speed NOT changed!!!\n\n");

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


void platforms::turtlebot_platform::cleanAllStatus()
{
	status_.communication_available=0;
	status_.deadlocked=0;
	status_.failed=0;
	status_.gps_spoofed=0;
	status_.movement_available=0;
	status_.moving=0;
	status_.rotating=0;
	status_.ok=0;
	status_.paused_moving=0;
	status_.paused_rotating=0;
	status_.reduced_sensing=0;
	status_.reduced_movement=0;
	status_.sensors_available=0;
	status_.waiting=0;
}

bool platforms::turtlebot_platform::isUnknownStatus()
{
	int sum = (*status_.communication_available)+
			(*status_.deadlocked)+
			(*status_.failed)+
			(*status_.gps_spoofed)+
			(*status_.movement_available)+
			(*status_.moving)+
			(*status_.rotating)+
			(*status_.ok)+
			(*status_.paused_moving)+
			(*status_.paused_rotating)+
			(*status_.reduced_sensing)+
			(*status_.reduced_movement)+
			(*status_.sensors_available)+
			(*status_.waiting);
/*ROS_INFO(" communication_available: %d",*status_.communication_available);
ROS_INFO(" deadlocked: %d",*status_.deadlocked);
ROS_INFO(" failed: %d",*status_.failed);
ROS_INFO(" gps: %d",*status_.gps_spoofed);
ROS_INFO(" movement_available: %d",*status_.movement_available);
ROS_INFO(" moving: %d",*status_.moving);
ROS_INFO(" rotating: %d",*status_.rotating);
ROS_INFO(" ok: %d",*status_.ok);
ROS_INFO(" paused_moving: %d",*status_.paused_moving);
ROS_INFO(" paused_rotating: %d",*status_.paused_rotating);
ROS_INFO(" reduced_sensing: %d",*status_.reduced_sensing);
ROS_INFO(" reduced_movement: %d",*status_.reduced_movement);
ROS_INFO(" sensors_available: %d",*status_.sensors_available);//*/
ROS_INFO(" waiting: %d",*status_.waiting);
	//ROS_INFO(" UNKNOWN: %d", sum);
	if (sum>0)
		return false;
	//ROS_INFO("UNKNOWN ------------------------------------------------_");
	return true;
}
