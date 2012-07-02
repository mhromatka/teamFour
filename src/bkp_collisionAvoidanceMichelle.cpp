/*
collisionAvoidance
This is where students will be able to program in a collision avoidance algorithm.  The telemetry callback
is already setup along with a dummy version of how the service request would work.
*/

//standard C++ headers
#include <sstream>
#include <stdlib.h>
#include <time.h>
#include <map>
#include <iostream>
#include <fstream>
#include <sstream>

//ROS, FL, Dubins headers
#include "ros/ros.h"
#include "AU_UAV_ROS/TelemetryUpdate.h"
#include "AU_UAV_ROS/GoToWaypoint.h"
#include "AU_UAV_ROS/RequestWaypointInfo.h"
#include "AU_UAV_ROS/standardFuncs.h"
#include "AU_UAV_ROS/FuzzyLogicController.h"
#include "AU_UAV_ROS/dubins.h"

<<<<<<< HEAD:src/collisionAvoidance.cpp
#define rho 15/(22.5*(M_PI/180.0)) //TODO-check if units are correct, this is currently in meters/radian
#define TIMESTEP 1 
#define UAV_AIRSPEED 11.176

//collisionAvoidance does not have a header file, define methods here
bool firstFuzzyEngine(double distanceToCollision, double overlapDistance, double distToWP, double distBtwnPlanes);
double secondFuzzyEngine(double distBtwnPlanes, double bearingAngle);
void isNewWaypoint(AU_UAV_ROS::PlanePose* myUAV, int myPlaneID);
=======
//collisionAvoidance does not have a header file, define methods here
//bool firstFuzzyEngine(double distanceToCollision, double overlapDistance);
//double secondFuzzyEngine(double distBtwnPlanes, double bearingAngle);//double distanceToCollision, double overlapDistance, double relativeBearingAngle);
double firstFuzzyEngine(double distanceToCollision, double overlapDistance);
double secondFuzzyEngine(double collImminence, double collAngle, double overlapDistance);//double distanceToCollision, double overlapDistance, double relativeBearingAngle);
>>>>>>> 9d3afac02e4be9aacfa8de2af322d2d36d3cae27:src/bkp_collisionAvoidanceMichelle.cpp

//ROS service client for calling a service from the coordinator
ros::ServiceClient goToWaypointClient;
ros::ServiceClient requestInfoClient;

//initialize some cool variables
std::map<int,AU_UAV_ROS::PlanePose> planeMap;
AU_UAV_ROS::FuzzyLogicController fuzzy1;
<<<<<<< HEAD:src/collisionAvoidance.cpp
std::ofstream myfile;
double distBtwnPlanes = -1;
double distToWP;
double currentHeading = 0.0;
int counter = 0;
AU_UAV_ROS::position nextGoalXY;
AU_UAV_ROS::fuzzyParams fuzzyParams;
//Initilize deez pointers
AU_UAV_ROS::PlanePose* currentUAV;
AU_UAV_ROS::PlanePose* oldUAV;
AU_UAV_ROS::PlanePose* closestUAV;

/* this method returns a dubins path for a UAV that would take it to the next waypoint
 should be called whenever a path needs to be calculated */
DubinsPath* setupDubins(AU_UAV_ROS::PlanePose* myUAV,int myPlaneID) {
    DubinsPath* myDubinsPath = new DubinsPath;
    //starting point
    double q0[3];
    //ending point
    double q1[3];
=======



//this function is run everytime new telemetry information from any plane is recieved
void telemetryCallback(const AU_UAV_ROS::TelemetryUpdate::ConstPtr& msg)
{

 //store current Pose (LatLongAlt) in a waypoint struct
    AU_UAV_ROS::waypoint planeLatLongAlt;

    planeLatLongAlt.longitude = msg->currentLongitude;
    planeLatLongAlt.latitude = msg->currentLatitude;
    planeLatLongAlt.altitude = msg->currentAltitude;
>>>>>>> 9d3afac02e4be9aacfa8de2af322d2d36d3cae27:src/bkp_collisionAvoidanceMichelle.cpp
    
    q0[0]=myUAV->getX();
    q0[1]=myUAV->getY();
    q0[2]=90.0 - myUAV->getHeading();
    //ROS_INFO("\n ourbearing is %f \n and dubins bearing is %f \n X= %f \n Y= %f", myUAV->getHeading(), q0[2], myUAV->getX(), myUAV->getY());
    q0[2] = q0[2]*DEGREES_TO_RADIANS;
    AU_UAV_ROS::position waypoints[2];
    
    isNewWaypoint(myUAV, myPlaneID);
    
    //grab x, y, z positions of next two waypoints
    waypoints[0] = getXYZ(myUAV->goalWaypoints[0]);
    waypoints[1] = getXYZ(myUAV->goalWaypoints[1]);
    ROS_INFO("goal %f", myUAV->goalWaypoints[0].latitude);
    
    //set x and y to the next waypoint as the endpoint of this dubins path
    q1[0]=waypoints[0].x_coordinate;
    q1[1]=waypoints[0].y_coordinate;
    
    //get bearing for the plane to be at when it reaches the end of path
    double deltaX = waypoints[1].x_coordinate - waypoints[0].x_coordinate;
    double deltaY = waypoints[1].y_coordinate - waypoints[0].y_coordinate;
    
    q1[2]=atan2(deltaY,deltaX); //in radians
    //create path
    ROS_INFO("q0[0]=%f  q0[1]=%f  q0[2]=%f",q0[0],q0[1],q0[2]);
    ROS_INFO("q1[0]=%f  q1[1]=%f  q1[2]=%f",q1[0],q1[1],q1[2]);
    dubins_init(q0,q1,rho,myDubinsPath);
    return myDubinsPath;
}

<<<<<<< HEAD:src/collisionAvoidance.cpp
void isNewWaypoint(AU_UAV_ROS::PlanePose* myUAV, int myPlaneID){
    int number = myUAV->goalWaypoints.size() - 1;
    
    AU_UAV_ROS::RequestWaypointInfo srv;
    srv.request.planeID=myPlaneID;
    srv.request.isAvoidanceWaypoint = false;
    srv.request.positionInQueue = number;
    requestInfoClient.call(srv);
    ROS_INFO("LATITUDE %f", srv.response.latitude);
    
    if (srv.response.latitude < 0)
    {
        //bump here!
        myUAV->goalWaypoints.erase(myUAV->goalWaypoints.begin());
    }
}
=======
    //ROS_INFO("currentPose for %d is:\n X = %f\n Y = %f\n Z = %f\n H = %f\n", msg->planeID, currentPose.x_coordinate, currentPose.y_coordinate, currentPose.altitude, newHeading);
    
   // ROS_INFO("Count during plane %d 's telem update is %d", msg->planeID, planeMap.count(msg->planeID));
    }
	if(planeMap.count(msg->planeID)==0)
    {
     //   ROS_INFO("Create a map for the very first time for %d", msg->planeID);
>>>>>>> 9d3afac02e4be9aacfa8de2af322d2d36d3cae27:src/bkp_collisionAvoidanceMichelle.cpp

//this function is run everytime new telemetry information from any plane is recieved
void telemetryCallback(const AU_UAV_ROS::TelemetryUpdate::ConstPtr& msg)
{
//add one to the counter
    if (msg->planeID == 0) {
        counter++;
    }

//planeLatLongAlt -> current UAV's waypoint (Lat, Long, Alt)
//currentPose -> current UAV's position (XYZ)
    	AU_UAV_ROS::waypoint planeLatLongAlt;
    	planeLatLongAlt.longitude = msg->currentLongitude;
    	planeLatLongAlt.latitude = msg->currentLatitude;
    	planeLatLongAlt.altitude = msg->currentAltitude;
        AU_UAV_ROS::position currentPose = getXYZ(planeLatLongAlt);
    
//If planeMap is empty, add current information
	if(planeMap.count(msg->planeID)==0)
    	{
            currentUAV = new AU_UAV_ROS::PlanePose;
            currentUAV->setID(msg->planeID);
            currentUAV->setX(currentPose.x_coordinate);
            currentUAV->setY(currentPose.y_coordinate);
            currentUAV->setZ(currentPose.altitude);
            currentUAV->setVelocity(11.176);
            currentUAV->setHeading(0.0);
            planeMap[msg->planeID] = *currentUAV;
            delete currentUAV;
    	}
    
//After initial planeMap creation, don't update planeMap until planes have been sent a new Waypoint
    else if (planeMap.count(msg->planeID)!=0 && msg->currentWaypointIndex == -1)
    {
<<<<<<< HEAD:src/collisionAvoidance.cpp
        //Don't do anything here!
=======

       // ROS_INFO("Don't do nothing.");

        //break
>>>>>>> 9d3afac02e4be9aacfa8de2af322d2d36d3cae27:src/bkp_collisionAvoidanceMichelle.cpp
    }
    
    //After initial update, we can now update things like newHeading during second time through
    //This should come after everything - no it shouldn't
    else
    {
<<<<<<< HEAD:src/collisionAvoidance.cpp
        currentUAV = new AU_UAV_ROS::PlanePose;
        currentUAV->setID(msg->planeID);
        currentUAV->setX(currentPose.x_coordinate);
        currentUAV->setY(currentPose.y_coordinate);
        currentUAV->setZ(currentPose.altitude);
        currentUAV->setVelocity(11.176);
=======

        //ROS_INFO("PLANES\n HAVE\n BEEN\n SENT\n WAY\n POINTS!");
        //ROS_INFO("PlanePose: \n X is %f \n Y is %f \n Z is %f \n Heading is %f", planeMap.find(msg->planeID)->second.getX(), planeMap.find(msg->planeID)->second.getY(), planeMap.find(msg->planeID)->second.getZ(), planeMap.find(msg->planeID)->second.getHeading());

        //get new heading
        //first parameter is the current plane's old position
        //second parameter is the current plane's current position
        
        AU_UAV_ROS::position oldPose = planeMap.find(msg->planeID)->second.getPosition();
        double newHeading = getNewHeading(oldPose, currentPose);

//	ROS_INFO("%d oldPose\n x = %f\n y = %f\n z = %f", msg->planeID, oldPose.x_coordinate, oldPose.y_coordinate, oldPose.altitude);
 //       ROS_INFO("%d currentPose\n x = %f\n y = %f\n z = %f", msg->planeID, currentPose.x_coordinate, currentPose.y_coordinate, currentPose.altitude);

        
        //not sure if I need anything here
        //set newHeading in planeMap
        //void AU_UAV_ROS::PlanePose.setHeading(newHeading);
        
        
    //Find FUZZY LOGIC PARAMETERS FOR CLOSEST PLANE:    
        //get closest plane to current plane here ----> maybe change this function to get "most dangerous" plane
        int closestPlane = getClosestPlane(msg->planeID, planeMap);
    
        //get distanceToCollision
        double distanceToCollision = getDistanceToCollision(planeMap.find(msg->planeID)->second, 
                                                        planeMap.find(closestPlane)->second);
    
        //get overlapDistance
        double overlapDistance = getOverlapDistance(planeMap.find(msg->planeID)->second, 
                                                planeMap.find(closestPlane)->second);
    
        //get bearingAngle
        double bearingAngle = getBearingAngle(planeMap.find(msg->planeID)->second, 
                                          planeMap.find(closestPlane)->second);
        
	
	double collisionAngle = getCollisionAngle(planeMap.find(msg->planeID)->second, 
                                          planeMap.find(closestPlane)->second);
	
	AU_UAV_ROS::position planePose1 = planeMap.find(msg->planeID)->second.getPosition();
    	AU_UAV_ROS::position planePose2 = planeMap.find(closestPlane)->second.getPosition();
	double distBtwnPlanes = getDist(planePose1, planePose2);
        
    /*----------------------------------------------------------------------------------*/
>>>>>>> 9d3afac02e4be9aacfa8de2af322d2d36d3cae27:src/bkp_collisionAvoidanceMichelle.cpp
        
        //Grab all of this plane's waypoints and store
        int i = 0;
        AU_UAV_ROS::waypoint nextGoal;
        AU_UAV_ROS::RequestWaypointInfo srv;
        srv.request.planeID=msg->planeID;
        srv.request.isAvoidanceWaypoint = false;
        srv.request.positionInQueue = 0;
        requestInfoClient.call(srv);
        while (srv.response.latitude > 0) {
            nextGoal.latitude = srv.response.latitude;
            nextGoal.longitude = srv.response.longitude;
            nextGoal.altitude = srv.response.altitude;
            //ROS_INFO("\n lat is %f \n long is %f", srv.response.latitude, srv.response.longitude);
            currentUAV->goalWaypoints.push_back(nextGoal);
            i++;
            
            srv.request.planeID=msg->planeID;
            srv.request.isAvoidanceWaypoint = false;
            srv.request.positionInQueue = i;
            
            requestInfoClient.call(srv);
        }
        
/*----------------------------------------------------------------------------------*/
        //This section stores a waypoint and position struct of where the current Plane would
        //travel if collision avoidance was NOT initiated (dubins path)
        AU_UAV_ROS::waypoint nextWaypoint;
        AU_UAV_ROS::position wouldBePose;
        
<<<<<<< HEAD:src/collisionAvoidance.cpp
        //update heading
        //Get the current Plane's heading by looking at the heading between the current Pose and Pose at last time step
        oldUAV = &planeMap.find(msg->planeID)->second;
        double currentHeading = getNewHeading(oldUAV->getPosition(), currentUAV->getPosition());
        
        //update currentUAV object
        currentUAV->setHeading(currentHeading);
        //ROS_INFO("\n X = %f \n Y = %f H = %f", currentUAV->getX(), currentUAV->getY(), currentUAV->getHeading());

        //check if dubins Vector is empty
        if (oldUAV->dubinsPoints.empty()) //currentUAV->dubinsPoints.empty())
        {
            ROS_INFO("empty for %d", msg->planeID);
            //create a new dubins path that goes from current position to next waypoint
            DubinsPath* newDubinspath = setupDubins(currentUAV, msg->planeID);
            int i=0;
            double q[3];
            int step = 1;
            while (i==0)
            {
                //step along dubins path and create waypoints at these points
                i = dubins_path_sample(newDubinspath,(UAV_AIRSPEED+.5)*step,q);
                AU_UAV_ROS::position newDubinsPoint;
                newDubinsPoint.x_coordinate = q[0];
                newDubinsPoint.y_coordinate = q[1];
                newDubinsPoint.altitude = msg->currentAltitude;
                currentUAV->dubinsPoints.push_back(newDubinsPoint);
                
                //print out dubins stuff
                std::stringstream ss; 
                ss<<"/Users/Jeffrey/Documents/data/dubins" << msg->planeID << ".txt";      
                std::string str(ss.str());
                myfile.open(str.c_str(), std::ofstream::app);
                myfile << newDubinsPoint.x_coordinate << " " << newDubinsPoint.y_coordinate << " " << "\n";
                myfile.close();
                
                //ROS_INFO("\n ******X is %f \n *******Y is %f", newDubinsPoint.x_coordinate, newDubinsPoint.y_coordinate);
                step++;
            }
//            currentUAV->dubinsPoints.pop_back(); //there seems to be 1 extra waypoint at end of avoidancepoints
            delete newDubinspath; 
=======
        //always stay at same altitude for next waypoint
        nextWaypoint.altitude = msg->currentAltitude;
    	
        //Decide to enter fuzzy logic??????
        double enterCA = firstFuzzyEngine(distanceToCollision, overlapDistance);

	if (enterCA > 0.35)
        {
            
	//fuzzyHeading is just change in heading, so we need to add it to the currentHeading
           // double fuzzyHeading = secondFuzzyEngine(distBtwnPlanes, bearingAngle)//distanceToCollision, overlapDistance, bearingAngle)
               //                     + newHeading;
		

		double fuzzyHeading = secondFuzzyEngine(enterCA, collisionAngle, overlapDistance) + newHeading;

		ROS_INFO("fuzzy heading= %f new heading = %f ", fuzzyHeading, newHeading);
		ROS_INFO("position is= %f, %f ", planePose1.x_coordinate, planePose1.y_coordinate); 
            //	ROS_INFO("newHeading is %f", newHeading);

     
            //convert fuzzyHeading to a waypoint to send the plane here
            //this should be in LatLongAlt
            
            nextWaypoint = getCAWaypoint(fuzzyHeading, currentPose);

		//ROS_INFO("nextWaypoint is %f   %f    %f", nextWaypoint.latitude, nextWaypoint.longitude, nextWaypoint.altitude);

            //nextWaypoint.latitude = 32.606573;
            //nextWaypoint.longitude = -85.490356;
            
>>>>>>> 9d3afac02e4be9aacfa8de2af322d2d36d3cae27:src/bkp_collisionAvoidanceMichelle.cpp
        }
        else
        {
<<<<<<< HEAD:src/collisionAvoidance.cpp
            //set current dubins to old dubins
            currentUAV->dubinsPoints = oldUAV->dubinsPoints;
=======
            //send current waypoint here!
            //explore using some other path planning here as well :P
            AU_UAV_ROS::RequestWaypointInfo requestsrv;
            requestsrv.request.planeID = msg->planeID;
            requestsrv.request.isAvoidanceWaypoint = false;
            requestsrv.request.positionInQueue = 0;
            
            if(requestInfoClient.call(requestsrv))
            {

               // ROS_INFO("Received response from service request %d", (count-1));

                //store response into the next waypoint
                nextWaypoint.latitude = requestsrv.response.latitude;
                nextWaypoint.longitude = requestsrv.response.longitude;
                nextWaypoint.altitude = requestsrv.response.altitude;
            }
            else
            {

              //  ROS_ERROR("Did not receive response");

            }
>>>>>>> 9d3afac02e4be9aacfa8de2af322d2d36d3cae27:src/bkp_collisionAvoidanceMichelle.cpp
        }
        
        AU_UAV_ROS::waypoint temp = convertPositionToWaypoint(currentUAV->dubinsPoints[0]);
        ROS_INFO("dubins[0] is %f for %d", temp.latitude, msg->planeID);
        
        wouldBePose = currentUAV->dubinsPoints[0]; //currentUAV->dubinsPoints[0];
        //figure out would-be heading without CA:
    	double wouldBeHeading = getNewHeading(currentPose, wouldBePose);
/*----------------------------------------------------------------------------------*/
        //Find FUZZY LOGIC PARAMETERS FOR CLOSEST PLANE:    
        //get closest plane to current plane here ----> maybe change this function to get "most dangerous" plane
        int closestPlane = getClosestPlane(msg->planeID, planeMap);
        //ROS_INFO("%d, %d", msg->planeID, closestPlane);
        closestUAV = &planeMap.find(closestPlane)->second;
        fuzzyParams = getFuzzyParams(currentUAV, closestUAV);
/*----------------------------------------------------------------------------------*/
        nextGoal = currentUAV->goalWaypoints[0];
        //ROS_INFO("next goalWaypoints is %f  %f", nextGoal.latitude, nextGoal.longitude);
        nextGoalXY = getXYZ(nextGoal);
        
        //find distToWP
        distToWP = getActualDistance(planeLatLongAlt, nextGoal);
        //find distBtwnPlanes
        distBtwnPlanes = getDist(currentUAV->getPosition(), closestUAV->getPosition());
        
        
        //Decide to enter fuzzy logic??????
        bool enterCA = firstFuzzyEngine(fuzzyParams.distanceToCollision, fuzzyParams.overlapDistance, distToWP, distBtwnPlanes);
        //ENTER CA:
        if (enterCA)
        {
            ROS_INFO("true");
            //fuzzyHeading is just change in heading, so we need to add it to the wouldBeHeading
            double fuzzyHeading = secondFuzzyEngine(distBtwnPlanes, fuzzyParams.bearingAngle) + wouldBeHeading;

<<<<<<< HEAD:src/collisionAvoidance.cpp
            //convert fuzzyHeading into a waypoint for the current Plane to go to
            nextWaypoint = getCAWaypoint(fuzzyHeading, currentPose);
=======
           // ROS_INFO("Received response from service request %d", (count-1));
>>>>>>> 9d3afac02e4be9aacfa8de2af322d2d36d3cae27:src/bkp_collisionAvoidanceMichelle.cpp

            //erase dubins
            currentUAV->dubinsPoints.clear();
        }
        //DIDN'T ENTER CA:
        else
        {
            //nextWaypoint = dubins
            //ROS_INFO("dubins is %f / %f / %f", currentUAV->dubinsPoints.front().x_coordinate, currentUAV->dubinsPoints.front().y_coordinate, currentUAV->dubinsPoints.front().altitude);

            nextWaypoint = convertPositionToWaypoint(currentUAV->dubinsPoints.front());
            //pop dubins
            currentUAV->dubinsPoints.erase(currentUAV->dubinsPoints.begin());
        }
/*----------------------------------------------------------------------------------*/
    //go to nextWaypoint everytime

    ROS_INFO("this is where its going", nextWaypoint.latitude);
    //always update map
    //service request to go to the waypoint determined by fuzzy logicness OR normal waypoint
    AU_UAV_ROS::GoToWaypoint gotosrv;
    gotosrv.request.planeID = msg->planeID;
    gotosrv.request.latitude = nextWaypoint.latitude;
    gotosrv.request.longitude = nextWaypoint.longitude;
    
    //these settings mean it is an avoidance maneuver waypoint AND to clear the avoidance queue
    gotosrv.request.isAvoidanceManeuver = true;
    gotosrv.request.isNewQueue = true;
    if(goToWaypointClient.call(gotosrv))
    {
        //ROS_INFO("Received response");
        //ROS_INFO("Lat is %f and Long is %f", nextWaypoint.latitude, nextWaypoint.longitude);
        //ROS_INFO("CURRENT POSE Lat is %f and Long is %f", planeLatLongAlt.latitude, planeLatLongAlt.longitude);
    }
    else
    {
        ROS_ERROR("Did not receive response");
    }
        //update map with current Plane Pose, Heading (currentUAV)
        planeMap[msg->planeID] = *currentUAV; 
    /*----------------------------------------------------------------------------------*/
    //Write some Outputs (distance to closest object and distance to waypoint) to some files)
    std::stringstream ss; 
    ss<<"/Users/Jeffrey/Documents/data/UAV" << msg->planeID << ".txt";      
    //ss<<"/Users/Jeffrey/github/local/Team-IV/ros/AU_UAV_stack/AU_UAV_ROS/data/distanceToClosestPlane" << msg->planeID << ".txt";
    std::string str(ss.str());
    myfile.open(str.c_str(), std::ofstream::app);
    myfile << counter << " " << currentUAV->getX() << " " << currentUAV->getY() << " " << currentUAV->getHeading() << " " << distToWP << " " << distBtwnPlanes << " " << fuzzyParams.bearingAngle << " " <<
    nextGoalXY.x_coordinate << " " << nextGoalXY.y_coordinate << "\n";
    myfile.close();
    delete currentUAV;
    }


}

<<<<<<< HEAD:src/collisionAvoidance.cpp

//This function will take inputs of min(A,teB) and A-B and output true or false to enter the CA algorithm where A is the distance for the current plane to the collision point and B is the distance to collision point for the closest plane to current plane. 
bool firstFuzzyEngine(double distanceToCollision, double overlapDistance, double distToWP, double distBtwnPlanes)
=======
/*
//This function will take inputs of min(A,B) and A-B and output true or false to enter the CA algorithm where A is the distance for the current plane to the collision point and B is the distance to collision point for the closest plane to current plane. 
bool firstFuzzyEngine(double distanceToCollision, double overlapDistance)
>>>>>>> 9d3afac02e4be9aacfa8de2af322d2d36d3cae27:src/bkp_collisionAvoidanceMichelle.cpp
{
/*
    bool collisionPotential;
    if ((distToWP < 25) | (distBtwnPlanes > 50))
    {
        collisionPotential = false;
    }
    else
    {
        collisionPotential = true;
    }
*/
	bool collisionPotential = false; 	
	double output = fuzzy1.FuzzyLogicOne(distanceToCollision, overlapDistance);
	//ROS_INFO("for dToColl= %f and overlap= %f", distanceToCollision, overlapDistance);
	//ROS_INFO("fuzzy logic output = %f", output);
	
	if(output >= 0.4){
		collisionPotential = true; 
	}
	return collisionPotential;

}
*/

double firstFuzzyEngine(double distanceToCollision, double overlapDistance){
	double output = fuzzy1.FuzzyLogicOne(distanceToCollision, overlapDistance);
	//ROS_INFO("for dToColl= %f and overlap= %f", distanceToCollision, overlapDistance);

	return output; 
	
}


double secondFuzzyEngine(double collImminence, double collAngle, double overlapDistance){
	double output = fuzzy1.MFuzzyLogicTwo1(collImminence, collAngle, overlapDistance); 

	return output; 
}


/*
//This function will take inputs of min(A,B), A-B, bearing angle and output the heading
double secondFuzzyEngine(double distBtwnPlanes, double bearingAngle)//double distanceToCollision, double overlapDistance, double bearingAngle)
{
	double changeInHeading = fuzzy1.FuzzyLogicTwo(distBtwnPlanes, bearingAngle);
	//ROS_INFO("for distBtwnPlanes= %f and bearingAngle= %f ", distBtwnPlanes, bearingAngle); 	
	//ROS_INFO("Change in heading: %f", changeInHeading);
    	return changeInHeading;
}
*/


int main(int argc, char **argv)
{
	//standard ROS startup
	ros::init(argc, argv, "collisionAvoidance");
	ros::NodeHandle n;
	
	//subscribe to telemetry outputs and create client for the avoid collision service
	ros::Subscriber sub = n.subscribe("telemetry", 1000, telemetryCallback);
	goToWaypointClient = n.serviceClient<AU_UAV_ROS::GoToWaypoint>("go_to_waypoint");
    requestInfoClient = n.serviceClient<AU_UAV_ROS::RequestWaypointInfo>("request_waypoint_info");
	
	//create Fuzzy Logic Controllers for use in telemetryCallback
	//fl1 = AU_UAV_ROS::FuzzyLogicController();

	//random seed for if statement in telemetryCallback, remove when collision avoidance work begins
	srand(time(NULL));

	//needed for ROS to wait for callbacks
	ros::spin();	

	return 0;
}
