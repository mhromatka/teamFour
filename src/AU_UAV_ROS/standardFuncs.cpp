/*
 Implementation of standardFuncs.h.  For information on how to use these functions, visit standardFuncs.h.  Comments in this file
 are related to implementation, not usage.
 */

#include <cmath>
#include <algorithm>
#include <stdlib.h>
#include <ros/ros.h>
#include "AU_UAV_ROS/standardFuncs.h"
#include "AU_UAV_ROS/standardDefs.h"
#include "AU_UAV_ROS/PlanePose.h"
#include "AU_UAV_ROS/FuzzyLogicController.h"
#include "fuzzylite/FuzzyLite.h"
#include <map>

#define WEST_MOST_LONGITUDE -85.490356
#define NORTH_MOST_LATITUDE 32.606573

#define METER_TO_LATITUDE (1.0/110897.21)
#define METER_TO_LONGITUDE (1.0/93865.7257)

#define DEGREES_TO_RADIANS (M_PI/180.0)
#define RADIANS_TO_DEGREES (180.0/M_PI)

double getCollisionAngle(AU_UAV_ROS::PlanePose first, AU_UAV_ROS::PlanePose second){
    AU_UAV_ROS::position collisionPoint = getTwoPlanesIntersect(first, second);
    
    //grab the (x, y, alt) values from each PlanePose object
    AU_UAV_ROS::position planePose1 = first.getPosition();
    AU_UAV_ROS::position planePose2 = second.getPosition();
    
    double distBtwnPlanes = getDist(planePose1, planePose2); //a
    double otherPlaneDistToColl = getPlaneDistToColl(second, first); //b
    double bearingAngle = getBearingAngle(first, second);//B
    //use law of sines to find collision angle, angle at which planes will collide, = asin(a * (sin(B)/b))
    double collAngle = asin(distBtwnPlanes * (sin(-bearingAngle)/otherPlaneDistToColl));
    // CHANGE ABOVE BEARING ANGLE TO NON-NEG WHEN CHANGE BEARING ANGLE TO BE SAME ORIENTATION AS OTHER ANGLES
    
    
    return collAngle;
}


//finds distance to collision for plane1
double getPlaneDistToColl(AU_UAV_ROS::PlanePose first, AU_UAV_ROS::PlanePose second){
    AU_UAV_ROS::position collisionPoint = getTwoPlanesIntersect(first, second);
    
    AU_UAV_ROS::position planePose1 = first.getPosition();
    double distance = sqrt(pow((collisionPoint.x_coordinate - planePose1.x_coordinate),2)+pow((collisionPoint.y_coordinate - planePose1.y_coordinate),2));
    
    
    return distance;
}

//This function will take in the planeMap and the closest/current planes' IDs
//and return the fuzzy parameters of distance to collision, distance between planes,
//overlap distance, and bearing angle
AU_UAV_ROS::fuzzyParams getFuzzyParams(AU_UAV_ROS::PlanePose* currentUAV, AU_UAV_ROS::PlanePose* closestUAV)
{
    AU_UAV_ROS::fuzzyParams fuzzyParams;
    fuzzyParams.distanceToCollision = getDistanceToCollision(*currentUAV, *closestUAV);
    fuzzyParams.overlapDistance = getOverlapDistance(*currentUAV, *closestUAV);
    fuzzyParams.bearingAngle = getBearingAngle(*currentUAV, *closestUAV);
    fuzzyParams.distBtwnPlanes = getDist(currentUAV->getPosition(), closestUAV->getPosition());

	return fuzzyParams;
}



//This function is passed a heading value returned by our fuzzy logic engine
//The function returns a waypoint to pass to the simulator

AU_UAV_ROS::waypoint getCAWaypoint(double fuzzyHeading, AU_UAV_ROS::position currentPose)
{
    AU_UAV_ROS::position NWPmeters;

    NWPmeters.x_coordinate = currentPose.x_coordinate + 30.0*sin(fuzzyHeading*DEGREES_TO_RADIANS);
    NWPmeters.y_coordinate = currentPose.y_coordinate + 30.0*cos(fuzzyHeading*DEGREES_TO_RADIANS);
    NWPmeters.altitude = currentPose.altitude;
    
    AU_UAV_ROS::waypoint nextWaypoint = convertPositionToWaypoint(NWPmeters);

    return nextWaypoint;
}

AU_UAV_ROS::waypoint convertPositionToWaypoint(AU_UAV_ROS::position position)
{
    double deltaX = position.x_coordinate;
    double deltaY = position.y_coordinate;    
    AU_UAV_ROS::waypoint waypoint;
    
    //Take x and y and convert back to lat long

    waypoint.longitude = WEST_MOST_LONGITUDE + (deltaX*METER_TO_LONGITUDE);
    waypoint.latitude = NORTH_MOST_LATITUDE + (deltaY*METER_TO_LATITUDE);
    waypoint.altitude = position.altitude;
    
    return waypoint;
}

//Convert a plane pose object (which stores planeID, x, y, z, heading) into a simple position type (meters)
AU_UAV_ROS::position convertPlanePoseToWaypoint(double x_coordinate, double y_coordinate, double altitude)
{
    AU_UAV_ROS::position position;
    position.x_coordinate = x_coordinate;
    position.y_coordinate = y_coordinate;
    position.altitude = altitude;
    
    return position;
}

//get closest plane
//eventually, it will be useful to make this function return closest plane with an imminent collision!!!!!
int getClosestPlane(int planeID, std::map<int,AU_UAV_ROS::PlanePose> planeMap)
{
    int closestPlane = -1;
    double smallestDist = std::numeric_limits<double>::infinity();
    
    for (int i =0; i < planeMap.size(); i++) {
        double dist = getDist(planeMap.find(planeID)->second.getPosition(), planeMap.find(i)->second.getPosition());
        
        if (i!=planeID) 
        {
            if (dist<smallestDist) 
            {
                smallestDist = dist;
                closestPlane = i;
            }
        }
    }
    return closestPlane;
}

//return distance (in meters) between two AU_UAV_ROS::position variables given in meters
double getDist(AU_UAV_ROS::position first, AU_UAV_ROS::position second)
{
    double dist = sqrt(pow((first.x_coordinate - second.x_coordinate),2)+pow((first.y_coordinate - second.y_coordinate),2)+pow((first.altitude - second.altitude),2));

return dist;
}


//This function will return the minimum distance to collision or the min(A,B)
//does NOT work in three space yet, whatever.
double getDistanceToCollision(AU_UAV_ROS::PlanePose first, AU_UAV_ROS::PlanePose second)
{
    AU_UAV_ROS::position collisionPoint = getTwoPlanesIntersect(first, second);
    
    AU_UAV_ROS::position planePose1 = first.getPosition();
    AU_UAV_ROS::position planePose2 = second.getPosition();
    
    double aValue = sqrt(pow((collisionPoint.x_coordinate - planePose1.x_coordinate),2)+pow((collisionPoint.y_coordinate - planePose1.y_coordinate),2));
    double bValue = sqrt(pow((collisionPoint.x_coordinate - planePose2.x_coordinate),2)+pow((collisionPoint.y_coordinate - planePose2.y_coordinate),2));
    
    if (aValue > bValue)
    {
        return bValue;
    }
    else
    {
        return aValue;
    }
}

//This function will take two plane positions and returns the intersection of the lines produced by each plane's respective heading. This point of intersection is the location of a possible intersection
//It will expect the planes' positions to be in meters and heading to be in degrees
AU_UAV_ROS::position getTwoPlanesIntersect(AU_UAV_ROS::PlanePose first, AU_UAV_ROS::PlanePose second)
{
    double planeHeading1 = first.getHeading()*DEGREES_TO_RADIANS;//planeHeading1*DEGREES_TO_RADIANS;
    double planeHeading2 = second.getHeading()*DEGREES_TO_RADIANS;//planeHeading2*DEGREES_TO_RADIANS;
    
    //convert planeHeading to "m" here where m is the slope of the line in the X-Y plane
    planeHeading1 = 1/tan(planeHeading1);
    planeHeading2 = 1/tan(planeHeading2);
    
    //grab the (x, y, alt) values from each PlanePose object
    AU_UAV_ROS::position planePose1 = first.getPosition();
    AU_UAV_ROS::position planePose2 = second.getPosition();
    
    double x_coordinate = ((planeHeading1*planePose1.x_coordinate)
                           -(planeHeading2*planePose2.x_coordinate)
                           -planePose1.y_coordinate
                           + planePose2.y_coordinate)
                           /(planeHeading1 - planeHeading2);
    double y_coordinate = planePose1.y_coordinate + planeHeading1*(x_coordinate - planePose1.x_coordinate);
    
    //collisionPoint returned in meters
    AU_UAV_ROS::position collisionPoint;
    collisionPoint.x_coordinate = x_coordinate;
    collisionPoint.y_coordinate = y_coordinate;
    collisionPoint.altitude = 0.0;
    
    return collisionPoint;
}

//This function will take two plane positions and find the difference in the distance away from the two's shared collision point. 
//For example, if plane1 is 10 meters away from the collision point where plane1 and plane2 would crash and plane 2 is 15 meters away from the same collision point, this function returns -5 meters.
double getOverlapDistance(AU_UAV_ROS::PlanePose first, AU_UAV_ROS::PlanePose second)
{
    AU_UAV_ROS::position collisionPoint = getTwoPlanesIntersect(first, second);
    
    //grab the (x, y, alt) values from each PlanePose object
    AU_UAV_ROS::position planePose1 = first.getPosition();
    AU_UAV_ROS::position planePose2 = second.getPosition();
    
    double overlapDistance = sqrt(pow((collisionPoint.x_coordinate - planePose1.x_coordinate),2)+pow((collisionPoint.y_coordinate - planePose1.y_coordinate),2)) - sqrt(pow((collisionPoint.x_coordinate - planePose2.x_coordinate),2)+pow((collisionPoint.y_coordinate - planePose2.y_coordinate),2));

    return overlapDistance;
}

//This function will return XYZ coordinates for any waypoint
//(usually used to find a plane's position)
//(the same coordinates being published to the RVIZ simulator)
//This function DOES take the earth's curvature into consideration
//The function uses the "getActualDistance" function with the top/left-most point and the plane's lat/long/alt
AU_UAV_ROS::position getXYZ(AU_UAV_ROS::waypoint planePose)
{
	AU_UAV_ROS::waypoint origin;
	
	origin.longitude=WEST_MOST_LONGITUDE;
	origin.latitude=NORTH_MOST_LATITUDE;
	origin.altitude=0.0;
    
	AU_UAV_ROS::waypoint northsouthpoint;
	northsouthpoint.latitude=planePose.latitude;
	northsouthpoint.longitude=WEST_MOST_LONGITUDE;
    
	AU_UAV_ROS::waypoint eastwestpoint;
	eastwestpoint.latitude=NORTH_MOST_LATITUDE;
	eastwestpoint.longitude=planePose.longitude;
    
	AU_UAV_ROS::position planeXYZ;

	planeXYZ.x_coordinate = (eastwestpoint.longitude - origin.longitude)/METER_TO_LONGITUDE;
	planeXYZ.y_coordinate = (northsouthpoint.latitude - origin.latitude)/METER_TO_LATITUDE;

	planeXYZ.altitude = planePose.altitude;
    
	return planeXYZ;
}

//This function will return the actual distance between two points in space (lat/long/alt) in meters
//This function DOES take the earth's curvature into consideration
double getActualDistance(AU_UAV_ROS::waypoint first, AU_UAV_ROS::waypoint second)
{
    double deltaLat = first.latitude - second.latitude;
	double deltaLong = first.longitude - second.longitude;
	

    double deltaY = deltaLat*110897.4592048873;
    double deltaX = deltaLong*93865.73571034615;

    double dist = sqrt(pow(deltaX,2) + pow(deltaY,2));
    
    return dist;
    
    
    /* this is Matt's method which we decided not to use:
        //difference in latitudes in radians
        double lat1 = first.latitude*DEGREES_TO_RADIANS;
        double lat2 = second.latitude*DEGREES_TO_RADIANS;
        double long1 = first.longitude*DEGREES_TO_RADIANS;
        double long2 = second.longitude*DEGREES_TO_RADIANS;
        
        double deltaLat = lat2 - lat1;
        double deltaLong = long2 - long1;
        
        //haversine crazy math, should probably be verified further beyond basic testing
        //calculate distance from current position to destination
        double a = pow(sin(deltaLat / 2.0), 2);
        a = a + cos(lat1)*cos(lat2)*pow(sin(deltaLong/2.0), 2);
        a = 2.0 * asin(sqrt(a));
        
        return EARTH_RADIUS * a;
     */
}

//This will take two waypoints and measure the heading between them (based on position)
//This is an estimation of plane heading based on the position heading from a point a time t and time t-1
//waypoints must be in meters
//A zero degree heading points directly North (and East is 90 degrees and West is -90 degrees to keep in [-180,180] range)
double getNewHeading(AU_UAV_ROS::position first, AU_UAV_ROS::position second)
{
    double deltaX = second.x_coordinate - first.x_coordinate;
	double deltaY = second.y_coordinate - first.y_coordinate;
	double heading = atan2(deltaX,deltaY);
    
	heading = (heading*RADIANS_TO_DEGREES);
    heading = manipulateAngle(heading);
    
	return heading;
}

//returns angle phi for the first plane
double getBearingAngle(AU_UAV_ROS::PlanePose first, AU_UAV_ROS::PlanePose second)
{
    //grab the (x, y, alt) values from each PlanePose object
    AU_UAV_ROS::position planePose1 = first.getPosition();
    AU_UAV_ROS::position planePose2 = second.getPosition();
    double theta = first.getHeading();
    
    //AU_UAV_ROS::position midpoint = getMidpoint(planePose1, planePose2);
    
    //get phi for plane 1 only, or the angle between North and the segment connecting the two planes
    double phi = getNewHeading(planePose1, planePose2);
    double bearingAngle = phi - theta;

	return bearingAngle;
}

//return the midpoint between two points in three space
AU_UAV_ROS::position getMidpoint(AU_UAV_ROS::position planePose1, AU_UAV_ROS::position planePose2)
{
    AU_UAV_ROS::position midpoint;
    midpoint.x_coordinate = ((planePose1.x_coordinate - planePose2.x_coordinate)/2)+planePose2.x_coordinate;
    midpoint.y_coordinate = ((planePose1.y_coordinate - planePose2.y_coordinate)/2)+planePose2.y_coordinate;
    midpoint.altitude = ((planePose1.altitude - planePose2.altitude)/2)+planePose2.altitude;
    
    return midpoint;
}

//The rest of the functions in this file were written by the APF team in 2011 REU
    //They may or may not be used in our algorithm
//
//
//
//
//



//The rest of the functions in this file were written by the APF team in 2011 REU
    //They may or may not be used in our algorithm
//
//
//
//
//

/* Modify the angle so that it remains on the interval [-180, 180] */
double manipulateAngle(double angle){
	while (angle > 180){
		/* decrease angle by one 360 degree cycle */
		angle-=360;
	}
    
	while (angle < -180){
		/* increase angle by one 360 degree cycle cycle */
		angle+=360;
	}
    
	while (angle == -0){
		/* increase angle by one 360 degree cycle cycle */
		angle=0;
	}
    
	return angle;
}


/* 
 Returns the Cardinal angle between two points of latitude and longitude in degrees.  The starting point is given
 by lat1 and long1 (the first two parameters), and the final point is given by lat2 and long2 (the final two parameters).
 The value returned is on the interval [-180, 180].
 */
double findAngle(double lat1, double long1, double lat2, double long2){
	double lonDiff = 0.0, angle = 0.0;
	double x = 0.0, y = 0.0;
    
	/* Convert latitudes to radians */
	lat2 *= DEGREE_TO_RAD;
	lat1 = lat1 * DEGREE_TO_RAD;
    
	lonDiff = (long2 - long1) * DEGREE_TO_RAD; /* convert difference in longitude to radians */
	
	/* Haversine math: see http://www.movable-type.co.uk/scripts/latlong.html for more information */
	y = sin(lonDiff)*cos(lat2);
	x = cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lonDiff);
    
	angle = atan2(y, x) * 180/PI;
    
	//Angle will be in degrees.
	return angle;
}

/*
 Given a waypoint (latitude, longitude, and altitude) as well as the bearing and angular distance to travel,
 calculateCoordinate will return the new location in the form of a waypoint.
 */
AU_UAV_ROS::waypoint calculateCoordinate(AU_UAV_ROS::waypoint currentPosition, double bearing, double distance){
	// Calculate final latitude and longitude; see movable-type.co.uk/scripts/latlong.html for more detail
	bearing *= DEGREES_TO_RADIANS; // convert angle of force to radians
    
	double lat1 = currentPosition.latitude*DEGREES_TO_RADIANS; // lat1 = current latitude in radians
	double dLat = distance*cos(bearing); // calculate change in latitude
	double lat2 = lat1 + dLat; // calculate final latitude
	double dPhi = log(tan(lat2/2+PI/4)/tan(lat1/2+PI/4));
	double q = (!(dPhi < 0.0001)) ? dLat/dPhi : cos(lat1);  // East-West line gives dPhi=0
	double dLon = distance*sin(bearing)/q; // calculate change in longitude
	
	// check for some daft bugger going past the pole, normalise latitude if so
	if (abs(lat2) > PI/2) 
		lat2 = lat2>0 ? PI-lat2 : -(PI-lat2);
	
	double lon2 = (currentPosition.longitude*DEGREES_TO_RADIANS+dLon) * RADIANS_TO_DEGREES; // calculate final latitude and convert to degrees
    
	//wrap around if necessary to ensure final longitude is on the interval [-180, 180]
	lon2 = manipulateAngle(lon2);
    
	lat2 *= RADIANS_TO_DEGREES; // convert final latitude to degrees
    
	AU_UAV_ROS::waypoint coordinate;
	coordinate.latitude = lat2;
	coordinate.longitude = lon2;
	coordinate.altitude = currentPosition.altitude;
	
	return coordinate;
}

/* Convert Cardinal direction to an angle in the Cartesian plane */
double toCartesian(double UAVBearing){
	UAVBearing = manipulateAngle(UAVBearing); /* get angle on the interval [-180, 180] */
    
	if (UAVBearing < 180 && UAVBearing >= 0) /* UAV bearing is in the first or fourth quadrant */
		return 90 - UAVBearing;
	else if (UAVBearing < 0 && UAVBearing >= -90) /* UAV bearing is in the second quadrant */
		return -1*UAVBearing + 90;
	else if (UAVBearing < -90 && UAVBearing > -180) /* UAV bearing is in the third quadrant */
		return -1*(UAVBearing + 180) - 90;
	else if (UAVBearing == 180 || UAVBearing == -180)
		return -90;
	else
		return -999; /* should never happen in current setup */
}

/* Convert angle in the Cartesian plane to a Cardinal direction */
double toCardinal(double angle){
	angle = manipulateAngle(angle); /* get angle on the interval [-180, 180] */
    
	if (angle <= 90 && angle >= -90) /* angle is in the first or fourth quadrant */
		return 90 - angle;
	else if (angle >= 90 && angle <= 180) /* angle is in the second quadrant */
		return -1*angle + 90;
	else if (angle <= -90 && angle >= -180) /* angle is in third quadrant */
		return -180 + -1*(90 + angle);
	else 
		return -999; /* should never happen in current setup */ 
}



/* 
 Returns the distance between two points of latitude and longitude in meters.  The first two parameters
 are the latitude and longitude of the starting point, and the last two parameters are the latitude and
 longitude of the ending point. 
 */
double findDistance(double lat1, double long1, double lat2, double long2){
	double latDiff = 0.0, lonDiff = 0.0;
	double squareHalfChord = 0.0, angularDistance = 0.0;
	
	/* Get difference in radians */
	latDiff = (lat1 - lat2)*DEGREE_TO_RAD;
	lonDiff = (long2 - long1)*DEGREE_TO_RAD;
    
	/* Find the square of half of the chord length between the two points */
	/* sin(lat difference / 2)^2 + cos(lat1) * cos(lat2)*sin(lon difference / 2)^2 */
	squareHalfChord = pow(sin(latDiff / 2), 2) + 
    pow(sin(lonDiff / 2), 2) *
    cos(lat1 * DEGREE_TO_RAD) *
    cos(lat2 * DEGREE_TO_RAD);
    
	/* Calculate the angular distance in radians */
	/* 2 * arctan(sqrt(squareHalfchrod), sqrt(1 - squareHalfChord)) */
	angularDistance = 2 * atan2(sqrt(squareHalfChord),
                                sqrt(1 - squareHalfChord));
    
	/* Return result in kilometers */
	return angularDistance * EARTH_RADIUS;
}



/*
 Given position at t-1 and at t, calculate heading on [-180, 180] where E = 90, N = 0 degrees */
