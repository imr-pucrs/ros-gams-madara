
#include "square_patrol.h"

#include <iostream>

gams::algorithms::BaseAlgorithm *
algorithms::square_patrolFactory::create (
		const madara::knowledge::KnowledgeMap & /*args*/,
		madara::knowledge::KnowledgeBase * knowledge,
		gams::platforms::BasePlatform * platform,
		gams::variables::Sensors * sensors,
		gams::variables::Self * self,
		gams::variables::Agents * agents)
{
	gams::algorithms::BaseAlgorithm * result (0);

	if (knowledge && sensors && platform && self)
	{
		result = new square_patrol (knowledge, platform, sensors, self);
	}
	else
	{
		madara_logger_ptr_log (gams::loggers::global_logger.get (),
				gams::loggers::LOG_MAJOR,
				"algorithms::square_patrolFactory::create:"
				" failed to create due to invalid pointers. "
				" knowledge=%p, sensors=%p, platform=%p, self=%p, agents=%p\n",
				knowledge, sensors, platform, self, agents);
	}

	/**
	 * Note the usage of logger macros with the GAMS global logger. This
	 * is highly optimized and is just an integer check if the log level is
	 * not high enough to print the message
	 **/
	if (result == 0)
	{
		madara_logger_ptr_log (gams::loggers::global_logger.get (),
				gams::loggers::LOG_ERROR,
				"algorithms::square_patrolFactory::create:"
				" unknown error creating square_patrol algorithm\n");
	}
	else
	{
		madara_logger_ptr_log (gams::loggers::global_logger.get (),
				gams::loggers::LOG_MAJOR,
				"algorithms::square_patrolFactory::create:"
				" successfully created square_patrol algorithm\n");
	}

	return result;
}

algorithms::square_patrol::square_patrol (
		madara::knowledge::KnowledgeBase * knowledge,
		gams::platforms::BasePlatform * platform,
		gams::variables::Sensors * sensors,
		gams::variables::Self * self,
		gams::variables::Agents * agents)
: gams::algorithms::BaseAlgorithm (knowledge, platform, sensors, self, agents)
{
	status_.init_vars (*knowledge, "square_patrol", self->agent.prefix);
	status_.init_variable_values ();
	wayPoints.push_back(gams::pose::Position(0.8,0,0));
	wayPoints.push_back(gams::pose::Position(1,3,0));
	wayPoints.push_back(gams::pose::Position(-2,3,0));
	wayPoints.push_back(gams::pose::Position(-2,0,0));
	//*/
	/*
  wayPoints.push_back(gams::pose::Position(1,0,0));
  wayPoints.push_back(gams::pose::Position(-1,0,0));
//*/
	now = 0;
	countPaused=0;
	platformStatus = NULL;
}

algorithms::square_patrol::~square_patrol ()
{
}

int algorithms::square_patrol::analyze (void)
{
	platformStatus = platform_->get_platform_status();
	if (*platformStatus->movement_available)
	{
		now = (now + 1)% wayPoints.size();
		madara_logger_ptr_log (gams::loggers::global_logger.get (),
				gams::loggers::LOG_MAJOR,"++++++++++++++++++++++++++++++++++++ Executing now %d / %d\n", now, wayPoints.size());
	}
	return 0;
}


int algorithms::square_patrol::execute (void)
{
	madara_logger_ptr_log (gams::loggers::global_logger.get (), gams::loggers::LOG_MAJOR," Executing now %d / %d -- min %f\n", now, wayPoints.size(), platform_->get_min_sensor_range());
	if (wayPoints.size() >0)
	{
		if ((*platformStatus->ok) || (*platformStatus->movement_available) || (*platformStatus->failed))
		{
			platform_->move(wayPoints[now],0.1);
			platform_->set_move_speed((now+1)*0.1+0.2);

			madara_logger_ptr_log (gams::loggers::global_logger.get (),
					gams::loggers::LOG_MAJOR,"\n######################\n ######################asking platform to move!!! now: %d / %d\n\n", now, wayPoints.size());
		}

	}
	if (now == 0)
	{
		if (countPaused == 2)
			platform_->pause_move();
		if (countPaused == 20)
			platform_->pause_move();

		std::cerr<<"countPaused: "<<countPaused<<std::endl;
		countPaused++;
	}

	return 0;
}


int
algorithms::square_patrol::plan (void)
{

	return 0;
}

