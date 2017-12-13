
#include "madara/knowledge/containers/NativeDoubleVector.h"
#include "turtleGenericFrame.h"
#include "threads/TopicListener.h"

gams::pose::CartesianFrame  platforms::turtleGenericFrame::cartesian_frame;
gams::pose::GPSFrame  platforms::turtleGenericFrame::gps_frame;

        
 
// factory class for creating a turtleGenericFrame 
gams::platforms::BasePlatform *
platforms::turtleGenericFrameFactory::create (
        const madara::knowledge::KnowledgeMap & args,
        madara::knowledge::KnowledgeBase * knowledge,
        gams::variables::Sensors * sensors,
        gams::variables::Platforms * platforms,
        gams::variables::Self * self)
{
  return new turtleGenericFrame (knowledge, sensors, self);
}
        
// Constructor
platforms::turtleGenericFrame::turtleGenericFrame (
  madara::knowledge::KnowledgeBase * knowledge,
  gams::variables::Sensors * sensors,
  gams::variables::Self * self)
: gams::platforms::RosBase (knowledge, sensors, self),
  ros_namespace_(knowledge->get (".ros_namespace").to_string ()),
  node_handle_ (ros_namespace_)
{
	frame_ = new gams::pose::CartesianFrame(gams::pose::Pose(0,0,0));
  // as an example of what to do here, create a coverage sensor
  if (knowledge && sensors)
  {
    // set the data plane for the threader
    threader_.set_data_plane (*knowledge);
  
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
    status_.init_vars (*knowledge, self->prefix.to_string());
    //self_->agent.gams_debug_level=0;
    //self_->agent.madara_debug_level=0;
    
    // create threads
    threader_.run(1.0, "TopicListener2", topic_listener_ = new threads::TopicListener2(node_handle_, self_, status_));
    threader_.run(1.0, "TopicPublisher", topic_publisher_ = new threads::TopicPublisher(node_handle_, self_));
    // end create threads
    
    goalId_.set_name(".goalId_", *knowledge);
    goalId_ = 0;
    frameType_.set_name(".frameType", *knowledge);


    self_->init_vars(*knowledge, self_->prefix.to_string());

    knowledge->print();


    updateServiceClientMoveBase = node_handle_.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/DWAPlannerROS/set_parameters");

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
}


// Destructor
platforms::turtleGenericFrame::~turtleGenericFrame ()
{
  threader_.terminate ();
  threader_.wait ();
}


// Polls the sensor environment for useful information. Required.
int platforms::turtleGenericFrame::sense (void)
{

	std::cerr<<"\n =================== location: "<<self_->agent.location.to_record(0).to_string()<<", "<<self_->agent.location.to_record(1).to_string()<<") ";
  return gams::platforms::PLATFORM_OK;
}


// Analyzes platform information. Required.
int
platforms::turtleGenericFrame::analyze (void)
{
	//knowledge_->print();
	if (*status_.paused_moving)
		return gams::platforms::WAITING;

	std::cerr<<"\n "
			<<"  communication_available: "<<status_.communication_available.to_string()<<" - "
	<<" deadlocked : "<<status_.deadlocked.to_string()<<" - "
	<<" failed : "<<status_.failed.to_string()<<" - "
	<<" gps_spoofed : "<<status_.gps_spoofed.to_string()<<" - "
	<<" movement_available : "<<status_.movement_available.to_string()<<" - "
	<<" moving : "<<status_.moving.to_string()<<" - "
	<<" rotating : "<<status_.rotating.to_string()<<" - "
		<<" ok : "<<status_.ok.to_string()<<" - "
		<<"  paused_moving: "<<status_.paused_moving.to_string()<<" - "
		<<"  paused_rotating: "<<status_.paused_rotating.to_string()<<" - "
		<<"  reduced_sensing: "<<status_.reduced_sensing.to_string()<<" - "
		<<"  reduced_movement: "<<status_.reduced_movement.to_string()<<" - "
		<<"  sensors_available: "<<status_.sensors_available.to_string()<<" - "
		<<"  waiting: "<<status_.waiting.to_string()<<" !!!! ";
	if (status_.moving==1)
	{
		madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_ALWAYS,"\n ------------- Still trying to move!!! Please wait %d...\n", goalId_.to_integer());
		return gams::platforms::MOVING;
	}
	else if (status_.waiting == 1)
	{
		madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_ALWAYS,"\n ------------- PLATFORM PENDING!!!...\n");
		return gams::platforms::WAITING;
	}
	else if (status_.movement_available == 1)
	{
		madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_ALWAYS,"\n ------------- PLATFORM ARRIVED!!!...\n");

		return gams::platforms::MOVEMENT_AVAILABLE;
	}
	else if (status_.failed == 1)
	{
		madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_ALWAYS,"\n ------------- PLATFORM ABORTED!!!...\n");

		return gams::platforms::FAILED;

	}
	else if (status_.reduced_movement == 1)
	{
		madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_ALWAYS,"\n ------------- PLATFORM REJECTED THE GOAL!!!...\n");
		return gams::platforms::REDUCED_MOVEMENT_AVAILABLE;

	}
	else if (status_.ok==1)
	{
		madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_ALWAYS,"\n ------------- PLATFORM RECALLED THE GOAL!!!...\n");
		return gams::platforms::OK;

	}
	else if (status_.reduced_sensing==1)
	{
		madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_ALWAYS,"\n ------------- PLATFORM RECALLED THE GOAL!!!...\n");
		return gams::platforms::REDUCED_SENSING_AVAILABLE;

	}


	return gams::platforms::UNKNOWN;

}


// Gets the name of the platform. Required.
std::string
platforms::turtleGenericFrame::get_name () const
{
  return "turtleGenericFrame";
}


// Gets the unique identifier of the platform.
std::string
platforms::turtleGenericFrame::get_id () const
{
  return self_->prefix.to_string();
}


// Gets the position accuracy in meters. Optional.
double
platforms::turtleGenericFrame::get_accuracy (void) const
{
	std::string rosParameter ="/move_base/DWAPlannerROS/xy_goal_tolerance";
	double accuracy;
	if (node_handle_.getParam(rosParameter.c_str(), accuracy))
		return accuracy*2.5; // should be this
	return 0.5;

}

// Gets Location of platform, within its parent frame. Optional.
gams::pose::Position
platforms::turtleGenericFrame::get_location () const
{
  gams::pose::Position result;
  result.from_container(self_->agent.location);
  if (frameType_=="gpsTocartesian")
  {
	  gams::utility::GPSPosition gpsP = gams::utility::GPSPosition::to_gps_position (result,
	  			gams::utility::GPSPosition(knowledge_->get (".initial_lat").to_double (), knowledge_->get (".initial_lon").to_double (), 0));
	  result = gams::pose::Position(gpsP.x, gpsP.y, gpsP.z);
  }
  return result;
}


// Gets Rotation of platform, within its parent frame. Optional.
gams::pose::Orientation
platforms::turtleGenericFrame::get_orientation () const
{
  gams::pose::Orientation result;
  result.from_container(self_->agent.orientation);
  return result;
}


// Gets sensor radius. Optional.
double
platforms::turtleGenericFrame::get_min_sensor_range () const
{
  // should be in square meters
  return 0.0;
}

// Gets move speed. Optional.
double
platforms::turtleGenericFrame::get_move_speed () const
{
	// should be in meters/s
	double speed;
	node_handle_.getParam("/move_base/DWAPlannerROS/max_vel_x", speed);
	return speed;
}

// Instructs the agent to return home. Optional.
int
platforms::turtleGenericFrame::home (void)
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
platforms::turtleGenericFrame::land (void)
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
platforms::turtleGenericFrame::move (
  const gams::pose::Position & location,
  double epsilon)
{
  /**
   * Movement functions work in a polling manner. In a real implementation,
   * we would want to check a finite state machine, external thread or
   * platform status to determine what to return. For now, we will simply
   * return that we are in the process of moving to the final pose.
   **/
	std::cerr<<"\n ------------------- move(move): ("<<location.latitude()<<", "<<location.longitude()<<") gps_frame: ("<<this->gps_frame.origin().x()<<", "<<this->gps_frame.origin().y()<<", "<<this->gps_frame.origin().z()<<") ";
	location.to_container(self_->agent.dest);

	std::vector<double> stdOrientation;
	stdOrientation.push_back(0.0);
	stdOrientation.push_back(0.0);
	stdOrientation.push_back(0.0);
	stdOrientation.push_back(1.0);
	self_->agent.orientation.set(stdOrientation);
	topic_publisher_->move();

    return gams::platforms::PLATFORM_MOVING;
}


// Rotates the platform to match a given angle. Optional.
int
platforms::turtleGenericFrame::rotate (
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
platforms::turtleGenericFrame::pose (const gams::pose::Pose & target,
  double loc_epsilon, double rot_epsilon)
{
  /**
   * Movement functions work in a polling manner. In a real implementation,
   * we would want to check a finite state machine, external thread or
   * platform status to determine what to return. For now, we will simply
   * return that we are in the process of moving to the final pose.
   **/

	double dist =target.distance_to(
			gams::pose::Pose(self_->agent.location.to_record(0).to_double(), self_->agent.location.to_record(1).to_double(), self_->agent.location.to_record(2).to_double())) ;

	if (dist< this->get_accuracy())
	{
		std::cerr<<"\n PLATFORM_ARRIVED";
		std::cerr<<"\n PLATFORM_ARRIVED";
		std::cerr<<"\n PLATFORM_ARRIVED";
			return gams::platforms::PLATFORM_ARRIVED;
	}
	gams::pose::Quaternion q(gams::pose::OrientationVector(target.rx(), target.ry(), target.rz() ));

	std::vector<double> stdOrientation;
	stdOrientation.push_back(q.x());
	stdOrientation.push_back(q.y());
	stdOrientation.push_back(q.z());
	stdOrientation.push_back(q.w());

	target.to_container(self_->agent.dest);
	self_->agent.orientation.set(stdOrientation);
	topic_publisher_->move();

	return gams::platforms::PLATFORM_MOVING;
}

// Pauses movement, keeps source and dest at current values. Optional.
void
platforms::turtleGenericFrame::pause_move (void)
{
	topic_publisher_->cancel();
}


// Set move speed. Optional.
void
platforms::turtleGenericFrame::set_move_speed (const double& speed)
{
	dynamic_reconfigure::Reconfigure srv;
	dynamic_reconfigure::DoubleParameter double_param;
	double_param.name = "max_vel_x";
	double_param.value = speed;
	srv.request.config.doubles.push_back(double_param);

	if (updateServiceClientMoveBase.call(srv))
	{
		madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_MAJOR,"\n ---- SUCCESS: speed changed %f!!!\n", speed);
	}
	else
		madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_MAJOR,"\n ---- ERROR: speed NOT changed!!!\n");
}


// Stops movement, resetting source and dest to current location. Optional.
void
platforms::turtleGenericFrame::stop_move (void)
{
	topic_publisher_->cancel();
}

// Instructs the agent to take off. Optional.
int
platforms::turtleGenericFrame::takeoff (void)
{
  return gams::platforms::PLATFORM_OK;
}

const gams::pose::ReferenceFrame &
platforms::turtleGenericFrame::get_frame (void) const
{
//	return gams::pose::GPSFrame(gams::pose::Pose(knowledge_->get (".initial_lat").to_double (), knowledge_->get (".initial_lon").to_double (), knowledge_->get (".initial_alt").to_double ()), gps_frame.EARTH_RADIUS);
	//return gps_frame;
	return *frame_;
}
