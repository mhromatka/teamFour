#ifndef FUZZYLOGICCONTROLLER_H
#define FUZZYLOGICCONTROLLER_H

#include "ros/ros.h"
#include "fuzzylite/FuzzyLite.h"
#include <string>
#include <sstream>



namespace AU_UAV_ROS{
	class FuzzyLogicController{
    private:
  	public:
        
        double FuzzyLogicOne(double in1, double in2);
	double FuzzyLogicTwo(double distance, double angle);
	double MFuzzyLogicTwo1(double collImminence, double collAngle, double oDist);
	
//	FuzzyLogicTwo(void);

    };
};
#endif
