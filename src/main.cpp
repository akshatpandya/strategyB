#include "../include/strategy/strategyb.h"
#include "ros/ros.h"
#include <time.h>

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "strategy");
	strategyb STRATEGY;

  STRATEGY.initialHerd();
}
