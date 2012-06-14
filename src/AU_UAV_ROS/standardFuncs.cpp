/*
 Implementation of standardFuncs.h.  For information on how to use these functions, visit standardFuncs.h.  Comments in this file
 are related to implementation, not usage.
 */

#include <cmath>
#include <stdlib.h>
#include <ros/ros.h>
#include "AU_UAV_ROS/standardFuncs.h"
#include "AU_UAV_ROS/standardDefs.h"
#include "AU_UAV_ROS/PlanePose.h"

#define WEST_MOST_LONGITUDE -85.490356
#define NORTH_MOST_LATITUDE 32.606573

#define METERS_TO_LATITUDE (1.0/111200.0)

#define DEGREES_TO_RADIANS (M_PI/180.0)
#define RADIANS_TO_DEGREES (180.0/M_PI)

//This function will take two plane positions and returns the intersection of the lines produced by each plane's respective heading. This point of intersection is the location of a possible intersection
//It will expect the planes' positions to be in meters and heading to be in degrees
AU_UAV_ROS::waypoint getTwoPlanesIntersect(AU_UAV_ROS::PlanePose planePose1, AU_UAV_ROS::PlanePose planePose2)
{
    double planeHeading1 = planePose1.getHeading()*DEGREES_TO_RADIANS;//planeHeading1*DEGREES_TO_RADIANS;
    double planeHeading2 = planePose2.getHeading()*DEGREES_TO_RADIANS;//planeHeading2*DEGREES_TO_RADIANS;
    
    //convert planeHeading to "m" here where m is the slope of the line in the X-Y plane
    planeHeading1 = 1/tan(planeHeading1);
    planeHeading2 = 1/tan(planeHeading2);
    //printf("planeHeading1 is %f\n planeHeading2 is %f\n", planeHeading1, planeHeading2);
    
    double planePose1Lat = planePose1.getX();
    double planePose1Long = planePose1.getY();
    double planePose2Lat = planePose2.getX();
    double planePose2Long = planePose2.getY();
    
    double x_coordinate = ((planeHeading1*planePose1Lat)
                           -(planeHeading2*planePose2Lat)
                           -planePose1Long
                           + planePose2Long)
                           /(planeHeading1 - planeHeading2);
    double y_coordinate = planePose1Long + planeHeading1*(x_coordinate - planePose1Lat);
    
    //collisionPoint returned in meters
    AU_UAV_ROS::waypoint collisionPoint;
    collisionPoint.latitude = x_coordinate;
    collisionPoint.longitude = y_coordinate;
    collisionPoint.altitude = 0.0;
    
    return collisionPoint;
}

//This function will take two plane positions and find the difference in the distance away from the two's shared collision point. 
//For example, if plane1 is 10 meters away from the collision point where plane1 and plane2 would crash and plane 2 is 15 meters away from the same collision point, this function returns 5 meters.
double getAMinusB(AU_UAV_ROS::PlanePose planePose1, AU_UAV_ROS::PlanePose planePose2)
{
    AU_UAV_ROS::waypoint collisionPoint = getTwoPlanesIntersect(planePose1, planePose2);
    double planePose1Lat = planePose1.getX();
    double planePose1Long = planePose1.getY();
    double planePose2Lat = planePose2.getX();
    double planePose2Long = planePose2.getY();
    
    double AMinusB = sqrt(pow((collisionPoint.latitude - planePose1Lat) + (collisionPoint.longitude - planePose1Long),2))
    - sqrt(pow((collisionPoint.latitude - planePose2Lat),2)+pow((collisionPoint.longitude - planePose2Long),2));
    //printf("A - B is %f    ", AMinusB);
    return abs(AMinusB);
}

//This function will return the plane's XYZ coordinates (the same coordinates being published to the RVIZ simulator)
//This function DOES take the earth's curvature into consideration
//The function uses the "getActualDistance" function with the top/left-most point and the plane's lat/long/alt
AU_UAV_ROS::waypoint getPlaneXYZ(AU_UAV_ROS::waypoint planePosition)
{
	AU_UAV_ROS::waypoint origin;
	
	origin.latitude=NORTH_MOST_LATITUDE;
	origin.longitude=WEST_MOST_LONGITUDE;
	origin.altitude=0.0;
    
	AU_UAV_ROS::waypoint northsouthpoint;
	northsouthpoint.latitude=planePosition.latitude;
	northsouthpoint.longitude=WEST_MOST_LONGITUDE;
    
	AU_UAV_ROS::waypoint eastwestpoint;
	eastwestpoint.latitude=NORTH_MOST_LATITUDE;
	eastwestpoint.longitude=planePosition.longitude;
    
	AU_UAV_ROS::waypoint planeXYZ;
	planeXYZ.latitude = getActualDistance(origin,eastwestpoint);
	planeXYZ.longitude = -getActualDistance(origin,northsouthpoint);
	planeXYZ.altitude = planePosition.altitude;
    
	return planeXYZ;
}

//This function will return the actual distance between two points in space (lat/long/alt)
//This function DOES take the earth's curvature into consideration
double getActualDistance(AU_UAV_ROS::waypoint first, AU_UAV_ROS::waypoint second)
{
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
}

//This will take two waypoints and measure the heading between them (based on position)
//This is an estimation of plane heading based on the position heading from a point a time t and time t-1
//waypoints must be in meters
//A zero degree heading points directly North (and East is 90 degrees and West is -90 degrees to keep in [-180,180] range)
double getNewHeading(AU_UAV_ROS::waypoint first, AU_UAV_ROS::waypoint second)
{
	first.latitude=0;
	first.longitude=0;
    
	second.latitude=0;
	second.longitude=1;
    
	double deltaX = second.latitude - first.latitude;
	double deltaY = second.longitude - first.longitude;
	double heading = atan(deltaX/deltaY);
	heading = (heading*RADIANS_TO_DEGREES);
	if (deltaX <= 0)
	{
		heading = -heading;
	}	
	printf("Heading is %f  ", heading);
	return heading;
}

double getRelativeBearingAngle(double myHeading, double theirHeading)
{
	return 0;
}


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
 double getPlaneDist(int planeID){
 //	ros::Rate rate(1.0);
 //create TF listener here
 tf::TransformListener listener;
 tf::StampedTransform transform;
 //	rate.sleep();
 try{
 listener.lookupTransform("/0", "/world", ros::Time::now(), transform);
 
 //		ros::Time now = ros::Time::now();
 //		listener.waitForTransform("/0", "/world", now, ros::Duration(0.7);
 //		listener.lookupTransform("/0", "/world", now, transform);
 }
 catch (tf::TransformException ex){
 ROS_ERROR("%s",ex.what());
 }
 
 //return distance here
 double distance;
 distance = sqrt(pow(transform.getOrigin().x(),2) + pow(transform.getOrigin().y(),2) + pow(transform.getOrigin().z(),2));
 //distance = 2.0;	
 return distance;
 
 //ROS_INFO("x is ", transform.getOrigin().x);
 //ROS_INFO("y is ", transform.getOrigin().y);
 //ROS_INFO("z is ", transform.getOrigin().z);
 
 }
 */


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