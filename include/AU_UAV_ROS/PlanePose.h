
#include "ros/ros.h"
#include "AU_UAV_ROS/standardFuncs.h"

namespace AU_UAV_ROS{
	class PlanePose{
  	public:
        //default constructor
        PlanePose(void);

        PlanePose(int id, double x, double y, double z, double heading);

        double setX(double newX);

        double setY(double newY);

        double setZ(double newZ);

        double setHeading(double newHeading);

        void update(const AU_UAV_ROS::TelemetryUpdate &msg);
    };
};
