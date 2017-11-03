 
#include "patrol.h"

#include <iostream>

gams::algorithms::BaseAlgorithm *
algorithms::patrolFactory::create (
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
    result = new patrol (knowledge, platform, sensors, self);
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "algorithms::patrolFactory::create:" 
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
      "algorithms::patrolFactory::create:" 
      " unknown error creating patrol algorithm\n");
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "algorithms::patrolFactory::create:" 
      " successfully created patrol algorithm\n");
  }

  return result;
}

algorithms::patrol::patrol (
  madara::knowledge::KnowledgeBase * knowledge,
  gams::platforms::BasePlatform * platform,
  gams::variables::Sensors * sensors,
  gams::variables::Self * self,
  gams::variables::Agents * agents)
  : gams::algorithms::BaseAlgorithm (knowledge, platform, sensors, self, agents)
{
  status_.init_vars (*knowledge, "patrol", self->agent.prefix);
  status_.init_variable_values ();
}

algorithms::patrol::~patrol ()
{
}

int
algorithms::patrol::analyze (void)
{
  return 0;
}
      

int
algorithms::patrol::execute (void)
{

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,"\n\n asking plataform to move!!!\n\n");

  platform_->move(gams::pose::Position(1,1,0), 0.1);

  return 0;
}


int
algorithms::patrol::plan (void)
{
  return 0;
}
