
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

	listener.waitForTransform("/map", "/base_footprint", ros::Time(0), ros::Duration(1.0));
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

	min_sensor_range_.set_name(".min_sensor_range", knowledge);
	max_sensor_range_.set_name(".max_sensor_range", knowledge);
	goalId_.set_name(".goalId_", knowledge);
	frame_.set_name(".frame", knowledge);
	initial_lat_.set_name(".initial_lat", knowledge);
	initial_lon_.set_name(".initial_lon", knowledge);
	initial_alt_.set_name(".initial_alt", knowledge);
	frameType_.set_name(".frameType", knowledge);

	//subOdom_ = node_handle_.subscribe(knowledge.get (".ros_namespace").to_string ()+"/odom", 1, &platforms::threads::TopicListener2::processOdom, this);
	subScan_ = node_handle_.subscribe(knowledge.get (".ros_namespace").to_string ()+"/scan", 1, &platforms::threads::TopicListener2::processScanOnce, this);
	subFeed_ = node_handle_.subscribe(knowledge.get (".ros_namespace").to_string ()+"/move_base/feedback", 1, &platforms::threads::TopicListener2::processFeedback, this);
	subStatus_ = node_handle_.subscribe(knowledge.get (".ros_namespace").to_string ()+"/move_base/status", 1, &platforms::threads::TopicListener2::processStatus, this);
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
	std::vector <double> locationTemp;
	std::vector <double> orientationTemp;
	tf::StampedTransform transform_in_map;
	try {
			listener.lookupTransform("/map", "/base_footprint", ros::Time(0), transform_in_map);
			if (frameType_=="cartesian")
			{
				locationTemp.push_back(transform_in_map.getOrigin().y()); // lat
				locationTemp.push_back(transform_in_map.getOrigin().x()); // lon
				orientationTemp.push_back(transform_in_map.getRotation().getY());
				orientationTemp.push_back(transform_in_map.getRotation().getX());
			}
			if (frameType_=="GPS")
			{
				locationTemp.push_back(transform_in_map.getOrigin().x()); // lat
				locationTemp.push_back(transform_in_map.getOrigin().y()); // lon
				orientationTemp.push_back(transform_in_map.getRotation().getX());
				orientationTemp.push_back(transform_in_map.getRotation().getY());
			}

		  	locationTemp.push_back(transform_in_map.getOrigin().z());
		  	self_->agent.location.set(locationTemp);

			orientationTemp.push_back(transform_in_map.getRotation().getZ());
			orientationTemp.push_back(transform_in_map.getRotation().getW());
			self_->agent.orientation.set(orientationTemp);
	}
	catch(tf::TransformException &exception) {
		  ROS_ERROR("%s", exception.what());
	}
	ros::spinOnce();
}



void platforms::threads::TopicListener2::processOdom(const nav_msgs::Odometry::ConstPtr& odom)
{

	std::vector <double> locationTemp;
	std::vector <double> orientationTemp;

	if (frameType_=="cartesian")
	{
		locationTemp.push_back(odom->pose.pose.position.y);// lat in cartesian system :S
		locationTemp.push_back(odom->pose.pose.position.x);// lon in cartesian system :S
		locationTemp.push_back(odom->pose.pose.position.z);// alt


		orientationTemp.push_back(odom->pose.pose.orientation.y);
		orientationTemp.push_back(odom->pose.pose.orientation.x);
	}
	if (frameType_=="GPS")
	{
		locationTemp.push_back(odom->pose.pose.position.x);// lat
		locationTemp.push_back(odom->pose.pose.position.y);// lon
		locationTemp.push_back(odom->pose.pose.position.z);// alt

		orientationTemp.push_back(odom->pose.pose.orientation.x);
		orientationTemp.push_back(odom->pose.pose.orientation.y);
	}
	self_->agent.location.set(locationTemp);
	orientationTemp.push_back(odom->pose.pose.orientation.z);
	orientationTemp.push_back(odom->pose.pose.orientation.w);
	self_->agent.orientation.set(orientationTemp);

}





void platforms::threads::TopicListener2::processScanOnce(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	this->min_sensor_range_ = scan->range_min;
	this->max_sensor_range_ = scan->range_max;
}

void platforms::threads::TopicListener2::processFeedback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& feed)
{
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
	for (unsigned int i = 0; i < statusValue->status_list.size(); i++)
	{
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
