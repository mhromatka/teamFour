#ifndef PLANEPOSE_H
#define PLANEPOSE_H

#include "ros/ros.h"
#include "AU_UAV_ROS/standardFuncs.h"
#include "AU_UAV_ROS/TelemetryUpdate.h"

namespace AU_UAV_ROS{
	class PlanePose{
    private:
        int planePoseID;
        double planePoseX;
        double planePoseY;
        double planePoseZ;
        double planePoseHeading;
        
  	public:
        //default constructor
        PlanePose(void);

        PlanePose(int planePoseID, double planePoseX, double planePoseY, double planePoseZ, double planePoseHeading);

        void setX(double newX);

        void setY(double newY);

        void setZ(double newZ);

        void setHeading(double newHeading);

        double getX();
        
        double getY();
        
        double getZ();
        
        double getHeading();
        AU_UAV_ROS::position getPosition();
        
        void update(const AU_UAV_ROS::TelemetryUpdate &msg);
    };
};
#endif
