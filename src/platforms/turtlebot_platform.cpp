
#include "madara/knowledge/containers/NativeDoubleVector.h"
#include "turtlebot_platform.h"





// factory class for creating a turtlebot_platform 
gams::platforms::BasePlatform *
platforms::turtlebot_platformFactory::create (
		const madara::knowledge::KnowledgeMap & args,
		madara::knowledge::KnowledgeBase * knowledge,
		gams::variables::Sensors * sensors,
		gams::variables::Platforms * platforms,
		gams::variables::Self * self)
{

    knowledge->set(".ros_namespace", "myplatform");
    knowledge->set(".ros_node", "turtlebot_platform_node");
	return new turtlebot_platform (knowledge, sensors, self);
}

// Constructor
platforms::turtlebot_platform::turtlebot_platform (
		madara::knowledge::KnowledgeBase * knowledge,
		gams::variables::Sensors * sensors,
		gams::variables::Self * self)
:   gams::platforms::RosBase (knowledge, sensors, self),
  	  ros_namespace_(knowledge->get (".ros_namespace").to_string ()),
	  node_handle_ (ros_namespace_),
	  move_client_ (std::string ("/move_base"), true)
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
			//gams::pose::Position origin (gams::pose::gps_frame());
			gams::pose::Position origin (*frame_);
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
		//self_->agent.init_vars(*knowledge, get_id());
		//self_->init_vars(*knowledge, get_id());
		self_->agent.location.set_name(".location", *knowledge);

		moveSpeed_.set_name(".moveSpeed", *knowledge);
		min_sensor_range_.set_name(".min_sensor_range", *knowledge);
		max_sensor_range_.set_name(".max_sensor_range", *knowledge);
		//location_.set_name(".location", *knowledge);
		orientation_.set_name(".orientation", *knowledge);

		// create threads
		threader_.run (35.0, "TopicListener", new platforms::threads::TopicListener(node_handle_));
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
	ros::init (remap, std::string("turtlebot_platform"));


	updateServiceClientMoveBase = node_handle_.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/DWAPlannerROS/set_parameters");


	firstMoveSent = false;
	std::cerr<<"\n waiting for move base server";
	move_client_.waitForServer();
	std::cerr<<"...Done\n";

	min_sensor_range_ = 0.0;
	max_sensor_range_ = 0.0;
	moveSpeed_ = 0.0;


}


// Destructor
platforms::turtlebot_platform::~turtlebot_platform ()
{
	threader_.terminate ();
	threader_.wait ();
}


// Polls the sensor environment for useful information. Required.
int platforms::turtlebot_platform::sense (void)
{
	return gams::platforms::PLATFORM_OK;
}


// Analyzes platform information. Required.
int platforms::turtlebot_platform::analyze (void)
{
        std::cerr<<" turtlebot platform analyze executed!\n";
	//knowledge_->print();
	if (move_client_.isServerConnected() == false)
		return gams::platforms::UNKNOWN;
	if (!firstMoveSent)
		return gams::platforms::MOVEMENT_AVAILABLE;

	if (*status_.paused_moving)
		return gams::platforms::WAITING;
	//std::cerr<<" status_"<<*status_.paused_moving;
	std::cerr<<"\n min_sensor_range_: "<<min_sensor_range_.to_double();
	//std::cerr<<"\n location: "<<location_.to_record();
	std::cerr<<"\n location: "<<self_->agent.location.to_record(0)<<", "<<self_->agent.location.to_record(1);

	if ((move_client_.getState().toString().compare("ACTIVE")==0) )//|| (move_client_.getState().toString().compare("PENDING")==0))
	{
		madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_ALWAYS,"\n ------------- Still trying to move!!! Please wait...%s\n", move_client_.getState().toString().c_str());
		cleanAllStatus();
		status_.moving = 1;
		return gams::platforms::MOVING;
	}
	else   if (move_client_.getState().toString().compare("PENDING")==0)
	{
		madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_ALWAYS,"\n ------------- PLATFORM PENDING!!!...\n");
		cleanAllStatus();
		status_.waiting = 1;
		return gams::platforms::WAITING;
	}
	else   if (move_client_.getState().toString().compare("SUCCEEDED")==0)
	{
		madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_ALWAYS,"\n ------------- PLATFORM ARRIVED!!!...\n");
		cleanAllStatus();
		status_.movement_available = 1;
		return gams::platforms::MOVEMENT_AVAILABLE;

	}
	else   if (move_client_.getState().toString().compare("ABORTED")==0)
	{
		madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_ALWAYS,"\n ------------- PLATFORM ABORTED!!!...\n");
		cleanAllStatus();
		status_.failed = 1;
		return gams::platforms::FAILED;

	}
	else   if (move_client_.getState().toString().compare("REJECTED")==0)
	{
		madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_ALWAYS,"\n ------------- PLATFORM REJECTED THE GOAL!!!...\n");
		cleanAllStatus();
		status_.reduced_movement = 1;
		return gams::platforms::REDUCED_MOVEMENT_AVAILABLE;

	}
	else   if (move_client_.getState().toString().compare("RECALLED")==0)
	{
		madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_ALWAYS,"\n ------------- PLATFORM RECALLED THE GOAL!!!...\n");
		cleanAllStatus();
		status_.ok = 1;
		status_.movement_available = 1;
		return gams::platforms::OK;

	}
	else   if (move_client_.getState().toString().compare("PREEMPTED")==0)
	{
		madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_ALWAYS,"\n ------------- PLATFORM PREEMPTED THE GOAL!!!...\n");
		cleanAllStatus();
		status_.ok = 1;
		status_.movement_available = 1;
		return gams::platforms::OK;

	}
	else   if (move_client_.getState().toString().compare("RECALLING")==0)
	{
		madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_ALWAYS,"\n ------------- PLATFORM RECALLING !!!...\n");
		cleanAllStatus();
		status_.waiting = 1;
		return gams::platforms::WAITING;
	}
	else   if (move_client_.getState().toString().compare("PREEMPTING")==0)
	{
		madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_ALWAYS,"\n ------------- PLATFORM PREEMPTING !!!...\n");
		cleanAllStatus();
		status_.waiting = 1;
		return gams::platforms::WAITING;
	}
	madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_ALWAYS,"\n ------------- moveclient %s...\n", move_client_.getState().toString().c_str());

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
	std::string rosParameter ="/move_base/DWAPlannerROS/xy_goal_tolerance";
	double accuracy;
	if (node_handle_.getParam(rosParameter.c_str(), accuracy))
		return accuracy; // should be this
// 		return accuracy+0.2; // it is a bug into move_base??
	return 0.0;
}

// Gets Location of platform, within its parent frame. Optional.
gams::pose::Position
platforms::turtlebot_platform::get_location () const
{
	//gams::pose::Position returnValue(location_.to_record(0).to_double(), location_.to_record(1).to_double(), location_.to_record(2).to_double());
	gams::pose::Position returnValue(self_->agent.location.to_record(0).to_double(), self_->agent.location.to_record(1).to_double(), self_->agent.location.to_record(2).to_double());
	return returnValue;
}


// Gets Rotation of platform, within its parent frame. Optional.
gams::pose::Orientation
platforms::turtlebot_platform::get_orientation () const
{
	gams::pose::Orientation returnValue(orientation_.to_record(0).to_double(), orientation_.to_record(1).to_double(), orientation_.to_record(2).to_double());
	return returnValue;
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
	return this->moveSpeed_.to_double();
}

// Instructs the agent to return home. Optional.
int
platforms::turtlebot_platform::home (void)
{
	return this->move(home_, 0.1);
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
	return gams::platforms::PLATFORM_OK;
}


// Moves the platform to a location. Optional.
int platforms::turtlebot_platform::move (
		const gams::pose::Position & location,
		double epsilon)
{
	// generate message
	firstMoveSent = true;
        std::cerr<<"received request to move! ("<<location.x()<<", "<<location.y()<<" )\n";
	madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_ALWAYS,"\n ------------- RECEIVED REQUEST TO MOVE!!! (%f, %f)\n", location.x(), location.y());
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now ();
	goal.target_pose.pose.position.x = location.x ();
	goal.target_pose.pose.position.y = location.y ();
	goal.target_pose.pose.position.z = 0;

	gams::pose::Orientation orientation(location.x()-targetLocation_.x(), location.y()-targetLocation_.y(), location.z()-targetLocation_.z());
	double dist = sqrt(orientation.rx()*orientation.rx()+orientation.ry()*orientation.ry()+orientation.rz()*orientation.rz());
	if (dist < 0.1)
	{
		orientation =gams::pose::Orientation(location.x(), location.y(), location.z());
		dist = sqrt(orientation.rx()*orientation.rx()+orientation.ry()*orientation.ry()+orientation.rz()*orientation.rz());
	}
	double cossTheta = orientation.rx()/dist;
	double theta = acos(cossTheta);
	if (orientation.ry() <0)
		theta = (-1)*theta;

	goal.target_pose.pose.orientation.x = 0;
	goal.target_pose.pose.orientation.y = 0;
	goal.target_pose.pose.orientation.z = sin(theta/2.0);
	goal.target_pose.pose.orientation.w = cos(theta/2.0);

	targetLocation_ = location;

	// send the goal
	move_client_.sendGoal(goal);

	return gams::platforms::PLATFORM_MOVING;

}


// Rotates the platform to match a given angle. Optional.
int platforms::turtlebot_platform::rotate (
		const gams::pose::Orientation & target,
		double epsilon)
{
	return gams::platforms::PLATFORM_MOVING;
}


// Moves the platform to a pose (location and rotation). Optional.
int platforms::turtlebot_platform::pose (const gams::pose::Pose & target,
		double loc_epsilon, double rot_epsilon)
{

	if (firstMoveSent)
	{
		double x = target.x()-targetLocation_.x();
		double y = target.y()-targetLocation_.y();
		double z = target.z()-targetLocation_.z();
		double dist = sqrt(x*x+y*y+z*z);
		if (dist < loc_epsilon)
		{
			cleanAllStatus();
			status_.movement_available = 1;
			return gams::platforms::PLATFORM_ARRIVED;
		}
	}
	firstMoveSent = true;
	madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_MAJOR,"\n ------------- RECEIVED REQUEST TO MOVE!!! (%f, %f)\n", target.x(), target.y());
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now ();
	goal.target_pose.pose.position.x = target.x ();
	goal.target_pose.pose.position.y = target.y ();
	goal.target_pose.pose.position.z = target.z();

	gams::pose::Quaternion q(target.rx(),target.ry(), target.rz());
	goal.target_pose.pose.orientation.x = q.x();
	goal.target_pose.pose.orientation.y = q.y();
	goal.target_pose.pose.orientation.z = q.z();
	goal.target_pose.pose.orientation.w = q.w();
	targetLocation_=target;

	// send the goal
	move_client_.sendGoal(goal);

	return gams::platforms::PLATFORM_MOVING;
}

// Pauses movement, keeps source and dest at current values. Optional.
void platforms::turtlebot_platform::pause_move (void)
{
	if (*status_.moving)
	{
		move_client_.cancelAllGoals();
		cleanAllStatus();
		status_.paused_moving = 1;

	}
	else if (*status_.paused_moving)
	{
		move(targetLocation_);
		cleanAllStatus();
		status_.moving = 1;
	}
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
		madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_MAJOR,"\n ---- SUCCESS: speed changed %f!!!\n", speed);
	}
	else
		madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_MAJOR,"\n ---- ERROR: speed NOT changed!!!\n");

}


// Stops movement, resetting source and dest to current location. Optional.
void
platforms::turtlebot_platform::stop_move (void)
{
	move_client_.cancelAllGoals();
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

	//return gams::pose::gps_frame();
	return *frame_;
	//return gams::pose::GPSFrame(gams::pose::Pose(0,0,0));
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




void platforms::turtlebot_platform::set_home(gams::pose::Position home)
{
	this->home_ = home;
}

