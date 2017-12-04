
#include "gams/loggers/GlobalLogger.h"
#include "TopicListener.h"

namespace knowledge = madara::knowledge;

// constructor
platforms::threads::TopicListener::TopicListener (ros::NodeHandle node_handle)
{
	odomChanged_=false;
	scanChanged_=false;
	node_handle_=node_handle;

	subOdom_ = node_handle_.subscribe("/odom", 1, &platforms::threads::TopicListener::processOdom, this);
	subScan_ = node_handle_.subscribe("/scan", 1, &platforms::threads::TopicListener::processScanOnce, this);


}

// destructor
platforms::threads::TopicListener::~TopicListener ()
{
}

/**
 * Initialization to a knowledge base. If you don't actually need access
 * to the knowledge base, just scheduling things in madara::threads::Threader,
 * then you can decide to delete this function or simply do nothing inside of
 * the function.
 **/
void
platforms::threads::TopicListener::init (knowledge::KnowledgeBase & knowledge)
{
	// point our data plane to the knowledge base initializing the thread
	data_ = knowledge;

	moveSpeed_.set_name(".moveSpeed", knowledge);
	min_sensor_range_.set_name(".min_sensor_range", knowledge);
	max_sensor_range_.set_name(".max_sensor_range", knowledge);
	//location_.set_name(".location_", knowledge);
	location_.set_name(".location", knowledge);
	orientation_.set_name(".orientation", knowledge);
}

/**
 * Executes the actual thread logic. Best practice is to simply do one loop
 * iteration. If you want a long running thread that executes something
 * frequently, see the madara::threads::Threader::runHz method in your
 * controller.
 **/
void
platforms::threads::TopicListener::run (void)
{
	ros::spinOnce();
}



void platforms::threads::TopicListener::processOdom(const nav_msgs::Odometry::ConstPtr& odom)
{
	//std::cerr<<"\n##############$$$$$$$$$$$$$%%%%%%%%%%%%%%% odom: "<<odom->pose.pose.position.x<<", "<<odom->pose.pose.position.y;
	std::vector <double> locationTemp;

	locationTemp.push_back(odom->pose.pose.position.x);
	locationTemp.push_back(odom->pose.pose.position.y);
	locationTemp.push_back(odom->pose.pose.position.z);
	location_.set(locationTemp);
	//location_= gams::pose::Position(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
	//orientation_= gams::pose::Orientation(gams::pose::Quaternion(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w));
	//moveSpeed_ = sqrt(odom->twist.twist.linear.x*odom->twist.twist.linear.x + odom->twist.twist.linear.y*odom->twist.twist.linear.y+odom->twist.twist.linear.z*odom->twist.twist.linear.z);
}





void platforms::threads::TopicListener::processScanOnce(const sensor_msgs::LaserScan::ConstPtr& scan)
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
