/*
Standard funcs.

A small collection of functions related to UAV flight that are used throughout the code.
IMPORTANT NOTE: All of the angles passed to the functions and returned from these functions are in degrees.
*/

#ifndef STANDARD_FUNCS
#define STANDARD_FUNCS

#include "AU_UAV_ROS/standardDefs.h" /* for EARTH_RADIUS in meters */
#include "AU_UAV_ROS/PlanePose.h"
#include "AU_UAV_ROS/FuzzyLogicController.h"
#include <map>

const double PI = 4*atan(1);
const double DEGREE_TO_RAD = PI/180; /* convert degrees to radians */

//Returns the angle at which two planes will collide at the collision point
double getCollisionAngle(AU_UAV_ROS::PlanePose first, AU_UAV_ROS::PlanePose second);

//this function returns the distance of param "first" to the collision point based on
//heading and position stored in each PlanePose object
double getPlaneDistToColl(AU_UAV_ROS::PlanePose first, AU_UAV_ROS::PlanePose second);

//return all fuzzy parameters
AU_UAV_ROS::fuzzyParams getFuzzyParams(AU_UAV_ROS::PlanePose* currentUAV, AU_UAV_ROS::PlanePose* closestUAV);


//This function is passed a heading value returned by our fuzzy logic engine
//The function returns a waypoint to pass to the simulator
AU_UAV_ROS::waypoint getCAWaypoint(double fuzzyHeading, AU_UAV_ROS::position currentPose);

AU_UAV_ROS::waypoint convertPositionToWaypoint(AU_UAV_ROS::position position);

//convert PlanePose object to a simple position struct data type
AU_UAV_ROS::position convertPlanePoseToWaypoint(double x_coordinate, double y_coordinate, double altitude);

//get closest plane
int getClosestPlane(int planeID, std::map<int,AU_UAV_ROS::PlanePose> planeMap);

//return distance (in meters) between two AU_UAV_ROS::position variables given in meters
double getDist(AU_UAV_ROS::position first, AU_UAV_ROS::position second);


/* these methods moved to collisionAvoidance.cpp

//return a boolean value to decide whether to enter "Collision Avoidance" mode
bool firstFuzzyEngine(double distanceToCollision, double overlapDistance);

//This function will take inputs of min(A,B), A-B, bearing angle and output the heading
double secondFuzzyEngine(double distanceToCollision, double overlapDistance, double relativeBearingAngle);
*/
//This function will return the minimum distance to collision or the min(A,B)
//does NOT work in three space yet, whatever.
double getDistanceToCollision(AU_UAV_ROS::PlanePose first, AU_UAV_ROS::PlanePose second);

//This function will take two plane positions and returns the intersection of the lines produced by each plane's respective heading. This point of intersection is the location of a possible intersection
//It will expect the planes' positions to be in meters and heading to be in degrees
AU_UAV_ROS::position getTwoPlanesIntersect(AU_UAV_ROS::PlanePose first, AU_UAV_ROS::PlanePose second);

//This function will take two plane positions and find the difference in the distance away from the two's shared collision point. 
//For example, if plane1 is 10 meters away from the collision point where plane1 and plane2 would crash and plane 2 is 15 meters away from the same collision point, this function returns 5 meters.
double getOverlapDistance(AU_UAV_ROS::PlanePose first, AU_UAV_ROS::PlanePose second);

//This function will return XYZ coordinates for any waypoint
//(usually used to find a plane's position)
//(the same coordinates being published to the RVIZ simulator)
//This function DOES take the earth's curvature into consideration
//The function uses the "getActualDistance" function with the top/left-most point and the plane's lat/long/alt
AU_UAV_ROS::position getXYZ(AU_UAV_ROS::waypoint planePose);

//This function will return the actual distance between two points in space (lat/long/alt) in meters
//This function DOES take the earth's curvature into consideration
double getActualDistance(AU_UAV_ROS::waypoint first, AU_UAV_ROS::waypoint second);

//This will take two waypoints and measure the heading between them (based on position)
//This is an estimation of plane heading based on the position heading from a point a time t and time t-1
//waypoints must be in meters
//A zero degree heading points directly North (and East is 90 degrees and West is -90 degrees to keep in [-180,180] range)
/* Given position at t-1 and at t, calculate heading on [-180, 180] where E = 90, N = 0 degrees */
double getNewHeading(AU_UAV_ROS::position first, AU_UAV_ROS::position second);

//returns angle phi for the first plane
double getBearingAngle(AU_UAV_ROS::PlanePose first, AU_UAV_ROS::PlanePose second);

//return the midpoint between two points in three space
AU_UAV_ROS::position getMidpoint(AU_UAV_ROS::position planePose1, AU_UAV_ROS::position planePose2);




//The rest of the functions in this file were written by the APF team in 2011 REU
//They may or may not be used in our algorithm

/* Modify the angle so that it remains on the interval [-180, 180] */
double manipulateAngle(double angle);

/* 
 Returns the Cardinal angle between two points of latitude and longitude in degrees.  The starting point is given
 by lat1 and long1 (the first two parameters), and the final point is given by lat2 and long2 (the final two parameters).
 The value returned is on the interval [-180, 180].
 */
double findAngle(double lat1, double long1, double lat2, double long2);

/*
 Given a waypoint (latitude, longitude, and altitude) as well as the bearing and angular distance to travel,
 calculateCoordinate will return the new location in the form of a waypoint.
 */
AU_UAV_ROS::waypoint calculateCoordinate(AU_UAV_ROS::waypoint currentPosition, double bearing, double distance);

/* Convert Cardinal direction to an angle in the Cartesian plane */
double toCartesian(double UAVBearing);

/* Convert angle in the Cartesian plane to a Cardinal direction */
double toCardinal(double angle);

/* 
 Returns the distance between two points of latitude and longitude in meters.  The first two parameters
 are the latitude and longitude of the starting point, and the last two parameters are the latitude and
 longitude of the ending point. 
 */
double findDistance(double lat1, double long1, double lat2, double long2);

#endif

