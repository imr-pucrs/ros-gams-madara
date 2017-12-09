
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
    status_.init_vars (*knowledge, get_id ());
    
    // create threads
    threader_.run(1.0, "TopicListener2", new threads::TopicListener2(node_handle_, self_));
    threader_.run(1.0, "TopicPublisher", topic_publisher_ = new threads::TopicPublisher(node_handle_, self_));
    // end create threads
    
    self_->init_vars(*knowledge, get_id());
    

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
  return gams::platforms::PLATFORM_OK;
}


// Analyzes platform information. Required.
int
platforms::turtleGenericFrame::analyze (void)
{
  return gams::platforms::PLATFORM_OK;
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
  return "turtleGenericFrame";
}


// Gets the position accuracy in meters. Optional.
double
platforms::turtleGenericFrame::get_accuracy (void) const
{
	std::string rosParameter ="/move_base/DWAPlannerROS/xy_goal_tolerance";
	double accuracy;
	if (node_handle_.getParam(rosParameter.c_str(), accuracy))
		return accuracy; // should be this
	return 0.0;
}

// Gets Location of platform, within its parent frame. Optional.
gams::pose::Position
platforms::turtleGenericFrame::get_location () const
{
  gams::pose::Position result;
  result.from_container(self_->agent.location);
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
	std::cerr<<"\n ------------------- move: ("<<location.x()<<", "<<location.y()<<") gps_frame: ("<<this->gps_frame.origin().x()<<", "<<this->gps_frame.origin().y()<<", "<<this->gps_frame.origin().z()<<") ";
	gams::utility::GPSPosition gpsP(location.y(), location.x());
	gams::utility::Position  p = gpsP.to_position(gams::utility::GPSPosition(-30.060700, -51.173249, 0));
	std::cerr<<"\n ------------------- p: ("<<p.x<<", "<<p.y<<")";
	gpsP = gams::utility::GPSPosition::to_gps_position (gams::utility::Position(10,0,0),
			gams::utility::GPSPosition(-30.060700, -51.173249, 0));
	std::cerr<<"\n ------------------- gpsP: ("<<gpsP.x<<", "<<gpsP.y<<")";
	p = gpsP.to_position(gams::utility::GPSPosition(-30.060700, -51.173249, 0));
	std::cerr<<"\n ------------------- p2: ("<<p.x<<", "<<p.y<<")";



	std::vector<double> stdOrientation;
	stdOrientation.push_back(0.0);
	stdOrientation.push_back(0.0);
	stdOrientation.push_back(0.0);
	stdOrientation.push_back(1.0);
	self_->agent.orientation.set(stdOrientation);
	location.to_container(self_->agent.dest);
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
	std::cerr<<"\n ------------------- pose: ("<<target.x()<<", "<<target.y()<<") ";
	gams::pose::Position location(target.x(),target.y(),target.z() );
	location.to_container(self_->agent.dest);
	gams::pose::Orientation orientation(target.rx(), target.ry(), target.rz());
	orientation.to_container(self_->agent.orientation);
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
  return gps_frame;
}
