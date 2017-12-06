 
#include "ZigZagCoverage.h"

#include <iostream>

gams::algorithms::BaseAlgorithm *
algorithms::ZigZagCoverageFactory::create (
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
    result = new ZigZagCoverage (knowledge, platform, sensors, self);
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "algorithms::ZigZagCoverageFactory::create:" 
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
      "algorithms::ZigZagCoverageFactory::create:" 
      " unknown error creating ZigZagCoverage algorithm\n");
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "algorithms::ZigZagCoverageFactory::create:" 
      " successfully created ZigZagCoverage algorithm\n");
  }

  return result;
}

algorithms::ZigZagCoverage::ZigZagCoverage (
  madara::knowledge::KnowledgeBase * knowledge,
  gams::platforms::BasePlatform * platform,
  gams::variables::Sensors * sensors,
  gams::variables::Self * self,
  gams::variables::Agents * agents)
  : gams::algorithms::area_coverage::BaseAreaCoverage (knowledge, platform, sensors, self, agents)
{
  status_.init_vars (*knowledge, "ZigZagCoverage", self->agent.prefix);
  status_.init_variable_values ();
}

algorithms::ZigZagCoverage::~ZigZagCoverage ()
{
}
/*

int
algorithms::ZigZagCoverage::analyze (void)
{
  return 0;
}
      

int
algorithms::ZigZagCoverage::execute (void)
{
	std::cerr<<"\n ------- ZigZagCoverage::execute!!!";
  return 0;
}


int
algorithms::ZigZagCoverage::plan (void)
{
  return 0;
}
*/


void algorithms::ZigZagCoverage::generate_new_position (void)
{

	std::cerr<<"\n ------- ZigZagCoverage::generate_new_position!!!";
	next_position_ = gams::utility::Position(1.0, 1.0, 0.0);
}
