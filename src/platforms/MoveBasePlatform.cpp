#include "madara/knowledge/containers/NativeDoubleVector.h"
#include "MoveBasePlatform.h"
#include "threads/TopicListener.h"
#include "threads/TopicListener2.h"
#include "threads/TopicPublisher.h"

//gams::pose::GPSFrame platforms::MoveBasePlatform::gps_frame;

// factory class for creating a MoveBasePlatform 
gams::platforms::BasePlatform *
platforms::MoveBasePlatformFactory::create(
		const madara::knowledge::KnowledgeMap & args,
		madara::knowledge::KnowledgeBase * knowledge,
		gams::variables::Sensors * sensors,
		gams::variables::Platforms * platforms, gams::variables::Self * self) {
	return new MoveBasePlatform(knowledge, sensors, self);
}

// Constructor
platforms::MoveBasePlatform::MoveBasePlatform(
		madara::knowledge::KnowledgeBase * knowledge,
		gams::variables::Sensors * sensors, gams::variables::Self * self) :
		gams::platforms::RosBase(knowledge, sensors, self),
		ros_namespace_(knowledge->get(".ros_namespace").to_string()),
		node_handle_(ros_namespace_)//,
		//sw_pose_(get_sw_pose()),
		//move_base_frame(sw_pose_)
{
	// as an example of what to do here, create a coverage sensor
	if (knowledge && sensors) {
		// set the data plane for the threader
		threader_.set_data_plane(*knowledge);

		// create a coverage sensor
		gams::variables::Sensors::iterator it = sensors->find("coverage");
		if (it == sensors->end()) // create coverage sensor
				{
			// get origin
			gams::pose::Position origin(gams::pose::gps_frame());
			madara::knowledge::containers::NativeDoubleArray origin_container;
			origin_container.set_name("sensor.coverage.origin", *knowledge, 3);
			origin.from_container(origin_container);

			// establish sensor
			gams::variables::Sensor* coverage_sensor =
					new gams::variables::Sensor("coverage", knowledge, 2.5,
							origin);
			(*sensors)["coverage"] = coverage_sensor;
		}
		(*sensors_)["coverage"] = (*sensors)["coverage"];
		status_.init_vars(*knowledge, get_id());

		// create threads
		threader_.run(1.0, "TopicListener2", topic_listener_ = new threads::TopicListener2(node_handle_, self_, status_));
		threader_.run(1.0, "TopicPublisher", topic_publisher_ = new threads::TopicPublisher(node_handle_, self_));
		// end create threads

		self_->init_vars(*knowledge, get_id());
		goalId_.set_name(".goalId_", *knowledge);


		ptr_gps_frame = new gams::pose::GPSFrame();
		madara::knowledge::containers::NativeDoubleVector sw_position (".movebase_sw_position", *knowledge_);
			// move_base use XY coordinate system, while gps use lat lon (XY)
		sw_pose_ = gams::pose::Pose(*ptr_gps_frame, sw_position[1], sw_position[0], 0, 0, 0, 0);
		ptr_move_base_frame = new gams::pose::CartesianFrame(sw_pose_);
		//gams::pose::Position gloc(*ptr_gps_frame, sw_position.to_record(1).to_double(), sw_position.to_record(0).to_double());
		//ptr_move_base_frame = new gams::pose::CartesianFrame(gloc);

		//cartesian_frame.bind_origin(get_sw_pose());
		gams::pose::Position pgps1(get_frame(), 39.99997056, 50.00000963);
		gams::pose::Position pgps2(get_frame(), 40.0000145, 49.99998194);
		std::cerr.precision(10);
		std::cerr<<"\n ---- dist: "<<pgps1.distance_to(pgps2);
		knowledge->print();

/*
		gams::pose::Position pgps=gams::pose::Position(50,40,0);
		std::cerr<<"\n ------------------ testing conversion 1";
		gams::pose::GPSFrame gps_frame2;
		std::cerr<<"\n ------------------ testing conversion 2";
		//gams::pose::Position gloc(gps_frame2,50,40);
		gams::pose::Position gloc(get_frame(),50,40);
		std::cerr<<"\n ------------------ testing conversion 3";
   	    gams::pose::CartesianFrame cart_frame0(gloc);
   	 std::cerr<<"\n ------------------ testing conversion 4";
   	    gams::pose::Position cloc2(cart_frame0, -1, 0);
   	 std::cerr<<"\n ------------------ testing conversion 5";
   	    std::cerr.precision(10);
   	    gams::pose::Position transformed2 = cloc2.transform_to(get_frame());
   	 std::cerr<<"\n ------------------transformed2: "<<transformed2.x()<<", "<<transformed2.y()<<", "<<transformed2.z()<<") ";
   	std::cerr<<"\n -------------------gps: "<<&get_frame()<<".... move_base: "<<ptr_move_base_frame;

   	 std::cerr<<"\n ------------------ testing conversion 6";
   	gams::pose::Position cloc3(*ptr_move_base_frame, 1, 0);
   	//std::cerr<<"\n cloc3 originframe: "<<cloc3.frame().origin_frame().origin();
   	std::cerr<<"\n ------------------ testing conversion 7";
   	gams::pose::Position transformed3 = cloc3.transform_to(get_frame());
   	std::cerr<<"\n ------------------ testing conversion 8";
   	std::cerr<<"\n ------------------transformed3: "<<transformed3.x()<<", "<<transformed3.y()<<", "<<transformed3.z()<<") ";
   	std::cerr<<"\n ------------------ testing conversion 9";

	//	gams::pose::Position pcar=gams::pose::Position(move_base_frame, pgps);
	//	std::cerr<<"\n ------------------pcar: "<<pcar.x()<<", "<<pcar.y()<<", "<<pcar.z()<<") ";
   	//gams::pose::Position location(get_frame(), 50.00009, 40);
   	//gams::pose::Position movebase_location = location.transform_to(move_base_frame);
   	//gams::pose::Position location(move_base_frame, 1, 1);
   	//transformed2 = location.transform_to(get_frame());
   	//   	 std::cerr<<"\n ------------------transformed2: "<<transformed2.x()<<", "<<transformed2.y()<<", "<<transformed2.z()<<") ";

*/

		tfListener_.waitForTransform("/map", "/base_footprint", ros::Time(0), ros::Duration(1.0));
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
platforms::MoveBasePlatform::~MoveBasePlatform() {
	threader_.terminate();
	threader_.wait();
	delete ptr_gps_frame;
	delete ptr_move_base_frame;
}

// Polls the sensor environment for useful information. Required.
int platforms::MoveBasePlatform::sense(void) {


	std::vector <double> orientationTemp;
	tf::StampedTransform transform_in_map;
	try {
			tfListener_.lookupTransform("/map", "/base_footprint", ros::Time(0), transform_in_map);

			std::cerr<<"\n MoveBasePlatform::sense: cartesian location: "<<transform_in_map.getOrigin().x()<<", "<<transform_in_map.getOrigin().y()<<", "<<transform_in_map.getOrigin().z()<<") ";
			// location (convert from cartesian system to gps system)
			gams::pose::Position moveBase_loc(*ptr_move_base_frame, transform_in_map.getOrigin().y(), transform_in_map.getOrigin().x());
			gams::pose::Position gps_loc(get_frame(), moveBase_loc);
			gps_loc.to_container(self_->agent.location);
			std::cerr<<"\n MoveBasePlatform::sense: gps location: "<<gps_loc.x()<<", "<<gps_loc.y()<<", "<<gps_loc.z()<<") ";

			// orientation
			gams::pose::Quaternion q(transform_in_map.getRotation().x(), transform_in_map.getRotation().y(), transform_in_map.getRotation().z(), transform_in_map.getRotation().w());
			gams::pose::Orientation moveBase_orient(*ptr_move_base_frame, q);
		    gams::pose::euler::YawPitchRoll moveBase_yawpitchroll (moveBase_orient);
			//self_->agent.orientation.set (2, moveBase_yawpitchroll.c ());
			//self_->agent.orientation.set (0, moveBase_yawpitchroll.a ());
			//self_->agent.orientation.set (1, moveBase_yawpitchroll.b ());
	}
	catch(tf::TransformException &exception) {
		  ROS_ERROR("%s", exception.what());
	}
	return gams::platforms::PLATFORM_OK;
}

// Analyzes platform information. Required.
int platforms::MoveBasePlatform::analyze(void) {
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
			madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_ALWAYS,"\n ------------- Still trying to move!!! Please wait for goal %d...\n", goalId_.to_integer());
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
std::string platforms::MoveBasePlatform::get_name() const {
	return "MoveBasePlatform";
}

// Gets the unique identifier of the platform.
std::string platforms::MoveBasePlatform::get_id() const {
	return self_->prefix.to_string();
}

// Gets the position accuracy in meters. Optional.
double platforms::MoveBasePlatform::get_accuracy(void) const {
	// will depend on your localization capabilities for robotics
	std::string rosParameter ="/move_base/DWAPlannerROS/xy_goal_tolerance";
	double accuracy;
	if (node_handle_.getParam(rosParameter.c_str(), accuracy))
		return accuracy+0.05; // should be this
	return 0.5; // otherwise, use 0.5
}

// Gets Location of platform, within its parent frame. Optional.
gams::pose::Position platforms::MoveBasePlatform::get_location() const {
	gams::pose::Position result;
	// is this really working??
	std::cerr<<"\n -------------------------------------------------------------";
	std::cerr<<"\n ------------platforms::MoveBasePlatform::get_location() -----";
	std::cerr<<"\n -------------------------------------------------------------";
	return result;
}

// Gets Rotation of platform, within its parent frame. Optional.
gams::pose::Orientation platforms::MoveBasePlatform::get_orientation() const {
	gams::pose::Orientation result;
	result.from_container(self_->agent.orientation);
	return result;
}

// Gets sensor radius. Optional.
double platforms::MoveBasePlatform::get_min_sensor_range() const {
	// should be in square meters
	return 0.0;
}

// Gets move speed. Optional.
double platforms::MoveBasePlatform::get_move_speed() const {
	// should be in meters/s
	return 0.0;
}

// Instructs the agent to return home. Optional.
int platforms::MoveBasePlatform::home(void) {
	/**
	 * Movement functions work in a polling manner. In a real implementation,
	 * we would want to check a finite state machine, external thread or
	 * platform status to determine what to return. For now, we will simply
	 * return that we are in the process of moving to the final pose.
	 **/
	return gams::platforms::PLATFORM_IN_PROGRESS;
}

// Instructs the agent to land. Optional.
int platforms::MoveBasePlatform::land(void) {
	/**
	 * Movement functions work in a polling manner. In a real implementation,
	 * we would want to check a finite state machine, external thread or
	 * platform status to determine what to return. For now, we will simply
	 * return that we are in the process of moving to the final pose.
	 **/
	return gams::platforms::PLATFORM_IN_PROGRESS;
}

// Moves the platform to a location. Optional.
int platforms::MoveBasePlatform::move(const gams::pose::Position & location,
		double epsilon) {
	/**
	 * Movement functions work in a polling manner. In a real implementation,
	 * we would want to check a finite state machine, external thread or
	 * platform status to determine what to return. For now, we will simply
	 * return that we are in the process of moving to the final pose.
	 **/
	// lets use basePlatform::move to basic initialization. That method handles duplicate move solicitations (same target)
	gams::pose::Position target(*ptr_gps_frame, location.x(), location.y());


	gams::pose::Position movebase_location = target.transform_to(*ptr_move_base_frame);
	std::vector<double> stdLocation;
	stdLocation.push_back(movebase_location.latitude());
	stdLocation.push_back(movebase_location.longitude());
	stdLocation.push_back(movebase_location.altitude());
	self_->agent.dest.set(stdLocation);
	std::vector<double> stdOrientation;
	stdOrientation.push_back(0);
	stdOrientation.push_back(0);
	stdOrientation.push_back(0);
	stdOrientation.push_back(1);


	self_->agent.dest_orientation.set(stdOrientation);


	topic_publisher_->move();

	std::cerr<<"\n ------------------------------------------------------------------------------";
	std::cerr<<"\n ---> move  ("<<movebase_location.latitude()<<", "<<movebase_location.longitude()<<") gps_frame: ("<<this->get_frame().origin().x()<<", "<<this->get_frame().origin().y()<<", "<<this->get_frame().origin().z()<<") ";
	std::cerr<<"\n ------------------------------------------------------------------------------";
	return gams::platforms::PLATFORM_MOVING;
}

// Rotates the platform to match a given angle. Optional.
int platforms::MoveBasePlatform::rotate(const gams::pose::Orientation & target,
		double epsilon) {
	/**
	 * Movement functions work in a polling manner. In a real implementation,
	 * we would want to check a finite state machine, external thread or
	 * platform status to determine what to return. For now, we will simply
	 * return that we are in the process of moving to the final pose.
	 **/
	// lets use basePlatform::orient to basic initialization.
	gams::platforms::BasePlatform::orient (target);

	gams::pose::Orientation movebase_target (*ptr_move_base_frame, target);
	movebase_target.to_container(self_->agent.dest_orientation);
	topic_publisher_->move();
	std::cerr<<"\n ---------------- ROTATE!!!!";

	return gams::platforms::PLATFORM_MOVING;
}

// Pauses movement, keeps source and dest at current values. Optional.
void platforms::MoveBasePlatform::pause_move(void) {
}

// Set move speed. Optional.
void platforms::MoveBasePlatform::set_move_speed(const double& speed) {
}

// Stops movement, resetting source and dest to current location. Optional.
void platforms::MoveBasePlatform::stop_move(void) {
}

// Instructs the agent to take off. Optional.
int platforms::MoveBasePlatform::takeoff(void) {
	return gams::platforms::PLATFORM_OK;
}

const gams::pose::ReferenceFrame &
platforms::MoveBasePlatform::get_frame(void) const {
	//return gps_frame;
	return *ptr_gps_frame;
	//return my_gps_frame;
	//return gams::pose::default_frame();
	//return gams::pose::gps_frame();
}


gams::pose::Pose platforms::MoveBasePlatform::get_sw_pose()
{
	madara::knowledge::containers::NativeDoubleVector sw_position (".movebase_sw_position", *knowledge_);
	// move_base use XY coordinate system, while gps use lat lon (YX)
	return gams::pose::Pose(get_frame()	, sw_position[1], sw_position[0], 0, 0, 0, 0);
}
