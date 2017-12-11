
#include "gams/loggers/GlobalLogger.h"
#include "TopicListener2.h"

namespace knowledge = madara::knowledge;

// constructor
platforms::threads::TopicListener2::TopicListener2 (ros::NodeHandle node_handle, gams::variables::Self* self, gams::variables::PlatformStatus status)
{
	odomChanged_=false;
	scanChanged_=false;
	node_handle_=node_handle;
	self_ = self;
	status_ = status;


	subOdom_ = node_handle_.subscribe("/odom", 1, &platforms::threads::TopicListener2::processOdom, this);
	subScan_ = node_handle_.subscribe("/scan", 1, &platforms::threads::TopicListener2::processScanOnce, this);
	subFeed_ = node_handle_.subscribe("/move_base/feedback", 1, &platforms::threads::TopicListener2::processFeedback, this);
	subStatus_ = node_handle_.subscribe("/move_base/status", 1, &platforms::threads::TopicListener2::processStatus, this);

	listener.waitForTransform("/map", "/odom", ros::Time(), ros::Duration(1.0));
}

// destructor
platforms::threads::TopicListener2::~TopicListener2 ()
{
}

/**
 * Initialization to a knowledge base. If you don't actually need access
 * to the knowledge base, just scheduling things in madara::threads::Threader,
 * then you can decide to delete this function or simply do nothing inside of
 * the function.
 **/
void
platforms::threads::TopicListener2::init (knowledge::KnowledgeBase & knowledge)
{
	// point our data plane to the knowledge base initializing the thread
	data_ = knowledge;

	//moveSpeed_.set_name(".moveSpeed", knowledge);
	min_sensor_range_.set_name(".min_sensor_range", knowledge);
	max_sensor_range_.set_name(".max_sensor_range", knowledge);
	goalId_.set_name(".goalId_", knowledge);
	frame_.set_name(".frame", knowledge);
	initial_lat_.set_name(".initial_lat", knowledge);
	initial_lon_.set_name(".initial_lon", knowledge);
	initial_alt_.set_name(".initial_alt", knowledge);
	//location_.set_name(".location_", knowledge);
	//location_.set_name(".location", knowledge);
	//orientation_.set_name(".orientation", knowledge);
}

/**
 * Executes the actual thread logic. Best practice is to simply do one loop
 * iteration. If you want a long running thread that executes something
 * frequently, see the madara::threads::Threader::runHz method in your
 * controller.
 **/
void
platforms::threads::TopicListener2::run (void)
{
	ros::spinOnce();
}



void platforms::threads::TopicListener2::processOdom(const nav_msgs::Odometry::ConstPtr& odom)
{
	//std::cerr<<"\n##############$$$$$$$$$$$$$%%%%%%%%%%%%%%% odom: "<<odom->pose.pose.position.x<<", "<<odom->pose.pose.position.y;
	std::vector <double> locationTemp;

	locationTemp.push_back(odom->pose.pose.position.x);
	locationTemp.push_back(odom->pose.pose.position.y);
	locationTemp.push_back(odom->pose.pose.position.z);
	//location_.set(locationTemp);

	tf::StampedTransform transform_in_map;
	try {
	  listener.lookupTransform("/map", "/odom", ros::Time(), transform_in_map);
	  /*std::cerr<<"\n transform_in_map: ("<<transform_in_map.getOrigin().getX()<<", "<<transform_in_map.getOrigin().getY()<<", "<<transform_in_map.getOrigin().getZ()<<") "

					  <<" rot: ("<<transform_in_map.getRotation().getX()<<", "
					  <<transform_in_map.getRotation().getY()<<", "
					  <<transform_in_map.getRotation().getZ()<<", "
					  <<transform_in_map.getRotation().getW()<<") ";*/

		tf::Vector3 loc(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
		loc = transform_in_map * loc;
		if (frame_=="gpsTocartesian")
		{
			//gams::pose::Position loc2(loc.x(), loc.y(), loc.z());
			gams::pose::Position loc2(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
			std::cerr<<"\n======================= loc cart: "<<loc2.x()<<", "<<loc2.y()<<", "<<loc2.z();
			gams::utility::GPSPosition gpsP = gams::utility::GPSPosition::to_gps_position (loc2,
				  			gams::utility::GPSPosition(initial_lat_.to_double(), initial_lon_.to_double(), initial_alt_.to_double()));
			loc = tf::Vector3(gpsP.x, gpsP.y, gpsP.z);

			std::cerr<<"\n======================= location gps: "<<loc.getX()<<", "<<loc.getY()<<", "<<loc.getZ();
		}
		std::vector <double> locationTemp2;
		locationTemp2.push_back(loc.getX());
		locationTemp2.push_back(loc.getY());
		locationTemp2.push_back(loc.getZ());
		//location_.set(locationTemp2);
		self_->agent.location.set(locationTemp2);
		std::vector <double> orientationTemp;
		orientationTemp.push_back(odom->pose.pose.orientation.x);
		orientationTemp.push_back(odom->pose.pose.orientation.y);
		orientationTemp.push_back(odom->pose.pose.orientation.z);
		orientationTemp.push_back(odom->pose.pose.orientation.w);
		self_->agent.orientation.set(orientationTemp);

		//std::cerr<<"\n locationTemp: "<<locationTemp<<" loc: "<<loc.getX()<<", "<<loc.getY()<<", "<<loc.getZ()<<") ";
	} catch(tf::TransformException &exception) {
	  ROS_ERROR("%s", exception.what());
	}
	//location_= gams::pose::Position(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
	//orientation_= gams::pose::Orientation(gams::pose::Quaternion(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w));
	//moveSpeed_ = sqrt(odom->twist.twist.linear.x*odom->twist.twist.linear.x + odom->twist.twist.linear.y*odom->twist.twist.linear.y+odom->twist.twist.linear.z*odom->twist.twist.linear.z);
}





void platforms::threads::TopicListener2::processScanOnce(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	this->min_sensor_range_ = scan->range_min;
	this->max_sensor_range_ = scan->range_max;
/*	std::cerr<<"\n ############################################################################";
	std::cerr<<"\n ###############################processScanOnce##############################";
	std::cerr<<"\n ############################################################################";
	std::cerr<<"\n ############################################################################";
	madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_ALWAYS,"\n\n ------------- SCAN --------------");
	*/
	//subScan_.shutdown();
}

void platforms::threads::TopicListener2::processFeedback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& feed)
{
	std::cerr<<"\n########################################### processFeedback "<<(int)(feed->status.status)<<"! "<<feed->status.text;
	std::cerr<<"\n ########################################### goalId: "<<feed->status.goal_id.id;
	cleanAllStatus();
	if (feed->status.status==0)//PENDING   	=0
		status_.waiting=1;
	if (feed->status.status==1)//ACTIVE		=1
			status_.moving=1;
	if (feed->status.status==2)//PREEMPTED 	=2
			status_.movement_available=1;
	if (feed->status.status==3)//SUCCEEDED 	=3
			status_.movement_available=1;
	if (feed->status.status==4)//ABORTED   	=4
			status_.failed=1;
	if (feed->status.status==5)//REJECTED	=5
			status_.reduced_movement=1;
	if (feed->status.status==6)//PREEMPTING	=6
			status_.waiting=1;
	if (feed->status.status==7)//RECALLING	=7
			status_.waiting=1;
	if (feed->status.status==8)//RECALLED	=8
			status_.movement_available=1;
	if (feed->status.status==9)//LOST		=9
			status_.reduced_sensing=1;
}

void platforms::threads::TopicListener2::processStatus(const actionlib_msgs::GoalStatusArray::ConstPtr& statusValue)
{
	std::cerr<<"\n ################################ STATUS: SIZE: "<<statusValue->status_list.size();
	for (unsigned int i = 0; i < statusValue->status_list.size(); i++)
	{
		std::cerr<<"\n ##############"<<i<<": goalId: "<<statusValue->status_list[i].goal_id.id<<" -> "<<statusValue->status_list[i].status<<" - "<<statusValue->status_list[i].text;
		if (statusValue->status_list[i].goal_id.id.compare(goalId_.to_string())==0)
		{
			cleanAllStatus();
			if (statusValue->status_list[i].status==0)//PENDING   	=0
				status_.waiting=1;
			if (statusValue->status_list[i].status==1)//ACTIVE		=1
					status_.moving=1;
			if (statusValue->status_list[i].status==2)//PREEMPTED 	=2
					status_.movement_available=1;
			if (statusValue->status_list[i].status==3)//SUCCEEDED 	=3
					status_.movement_available=1;
			if (statusValue->status_list[i].status==4)//ABORTED   	=4
					status_.failed=1;
			if (statusValue->status_list[i].status==5)//REJECTED	=5
					status_.reduced_movement=1;
			if (statusValue->status_list[i].status==6)//PREEMPTING	=6
					status_.waiting=1;
			if (statusValue->status_list[i].status==7)//RECALLING	=7
					status_.waiting=1;
			if (statusValue->status_list[i].status==8)//RECALLED	=8
					status_.movement_available=1;
			if (statusValue->status_list[i].status==9)//LOST		=9
					status_.reduced_sensing=1;

		}
	}
}

void platforms::threads::TopicListener2::cleanAllStatus(void)
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
