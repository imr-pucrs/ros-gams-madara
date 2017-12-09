
#include "gams/loggers/GlobalLogger.h"
#include "TopicPublisher.h"

namespace knowledge = madara::knowledge;

// constructor
platforms::threads::TopicPublisher::TopicPublisher (ros::NodeHandle node_handle,gams::variables::Self* self)
: self_(self)
{
	node_handle_=node_handle;
	goalChanged_=false;
	cancelRequested_=false;

	std::string topicName = "/move_base/goal";
	pubGoal_ = node_handle.advertise<move_base_msgs::MoveBaseActionGoal>(topicName.c_str(), 1);
}

// destructor
platforms::threads::TopicPublisher::~TopicPublisher ()
{
}

/**
 * Initialization to a knowledge base. If you don't actually need access
 * to the knowledge base, just scheduling things in madara::threads::Threader,
 * then you can decide to delete this function or simply do nothing inside of
 * the function.
 **/
void
platforms::threads::TopicPublisher::init (knowledge::KnowledgeBase & knowledge)
{
	// point our data plane to the knowledge base initializing the thread
	data_ = knowledge;
}

/**
 * Executes the actual thread logic. Best practice is to simply do one loop
 * iteration. If you want a long running thread that executes something
 * frequently, see the madara::threads::Threader::runHz method in your
 * controller.
 **/
void
platforms::threads::TopicPublisher::run (void)
{
	if (goalChanged_)
	{
		move_base_msgs::MoveBaseActionGoal msg;
		msg.header.frame_id="map";
		msg.header.stamp = ros::Time::now();
		msg.goal.target_pose.header.frame_id = "map";
		msg.goal.target_pose.pose.position.x = self_->agent.dest.to_record(0).to_double();
		msg.goal.target_pose.pose.position.y = self_->agent.dest.to_record(1).to_double();
		msg.goal.target_pose.pose.position.z = self_->agent.desired_altitude.to_double();
		msg.goal.target_pose.pose.orientation.x = self_->agent.orientation.to_record(0).to_double();
		msg.goal.target_pose.pose.orientation.y = self_->agent.orientation.to_record(1).to_double();
		msg.goal.target_pose.pose.orientation.z = self_->agent.orientation.to_record(2).to_double();
		msg.goal.target_pose.pose.orientation.w = self_->agent.orientation.to_record(3).to_double();
		pubGoal_.publish(msg);
		goalChanged_ = false;
	}
	if (cancelRequested_)
	{
		actionlib_msgs::GoalID msg;
		msg.stamp = ros::Time::now();
		pubCancel_.publish(msg);
		cancelRequested_=false;
	}
	ros::spinOnce();
}

void platforms::threads::TopicPublisher::move(void)
{
	goalChanged_ = true;
}


void platforms::threads::TopicPublisher::cancel(void)
{
	cancelRequested_ = true;

}
