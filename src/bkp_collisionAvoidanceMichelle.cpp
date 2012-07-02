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

//ROS headers
#include "ros/ros.h"
#include "AU_UAV_ROS/TelemetryUpdate.h"
#include "AU_UAV_ROS/GoToWaypoint.h"
#include "AU_UAV_ROS/RequestWaypointInfo.h"
#include "AU_UAV_ROS/standardFuncs.h"
#include "AU_UAV_ROS/FuzzyLogicController.h"

//collisionAvoidance does not have a header file, define methods here
//bool firstFuzzyEngine(double distanceToCollision, double overlapDistance);
//double secondFuzzyEngine(double distBtwnPlanes, double bearingAngle);//double distanceToCollision, double overlapDistance, double relativeBearingAngle);
double firstFuzzyEngine(double distanceToCollision, double overlapDistance);
double secondFuzzyEngine(double collImminence, double collAngle, double overlapDistance);//double distanceToCollision, double overlapDistance, double relativeBearingAngle);

//ROS service client for calling a service from the coordinator
ros::ServiceClient goToWaypointClient;
ros::ServiceClient requestInfoClient;

//keeps count of the number of services requested
int count;
double newHeading = 0.0;//check to make sure planes actually do initiate with 0.0 heading????

std::map<int,AU_UAV_ROS::PlanePose> planeMap;
AU_UAV_ROS::FuzzyLogicController fuzzy1;



//this function is run everytime new telemetry information from any plane is recieved
void telemetryCallback(const AU_UAV_ROS::TelemetryUpdate::ConstPtr& msg)
{

 //store current Pose (LatLongAlt) in a waypoint struct
    AU_UAV_ROS::waypoint planeLatLongAlt;

    planeLatLongAlt.longitude = msg->currentLongitude;
    planeLatLongAlt.latitude = msg->currentLatitude;
    planeLatLongAlt.altitude = msg->currentAltitude;
    
    //return current Pose in meters
    AU_UAV_ROS::position currentPose = getXYZ(planeLatLongAlt);

    //ROS_INFO("currentPose for %d is:\n X = %f\n Y = %f\n Z = %f\n H = %f\n", msg->planeID, currentPose.x_coordinate, currentPose.y_coordinate, currentPose.altitude, newHeading);
    
    //ROS_INFO("Count during plane %d 's telem update is %d", msg->planeID, planeMap.count(msg->planeID));
    
	if(planeMap.count(msg->planeID)==0)
    {
        //ROS_INFO("Create a map for the very first time for %d", msg->planeID);

    //ROS_INFO("currentPose for %d is:\n X = %f\n Y = %f\n Z = %f\n H = %f\n", msg->planeID, currentPose.x_coordinate, currentPose.y_coordinate, currentPose.altitude, newHeading);
    
   // ROS_INFO("Count during plane %d 's telem update is %d", msg->planeID, planeMap.count(msg->planeID));
    }
	if(planeMap.count(msg->planeID)==0)
    {
     //   ROS_INFO("Create a map for the very first time for %d", msg->planeID);

        //create new PlanePose object with currentPose and 0.0 heading
        AU_UAV_ROS::PlanePose newPlane(msg->planeID, currentPose.x_coordinate, currentPose.y_coordinate, currentPose.altitude, 0.0);// = new AU_UAV_ROS::PlanePose::PlanePose(msg->planeID, msg->currentLatitude, msg->currentLatitude, msg->currentAltitude, 0.0);
        
        //add the initial plane position to the map after the first telemetry update (with key = planeID)
        planeMap [msg->planeID]=newPlane;
    }
    
    //After initial planeMap creation, don't update planeMap until planes have been sent a new Waypoint
    //May not be totally necessary, but cleaner
    else if (planeMap.count(msg->planeID)!=0 && msg->currentWaypointIndex == -1)
    {

       // ROS_INFO("Don't do nothing.");

        //break
    }
    
    //After initial update, we can now update things like newHeading during second time through
    //This should come after everything - no it shouldn't
    else
    {

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
        
        
        //initialize waypoint to go (either a CA waypoint or normal waypoint, doesn't matter)
        AU_UAV_ROS::waypoint nextWaypoint;
        
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
            
        }

        //if CA avoidance is not initiated, just send the next goal waypoint
        else
        {
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
        }

        

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

           // ROS_INFO("Received response from service request %d", (count-1));

            count++;
        }
        else
        {

            ROS_ERROR("Did not receive response");

        }
    
//Basic Idea:
        //Decide to enter Collision Avoidance or Not?
        // 1) purely based on distance
        // 2) based on FL with inputs of distance and A-B
    


        //get each plane's closest plane
    
        //get relative plane heading
        
        //if relative plane heading = danger (between current plane and closest neighbor) AND relative collision point = danger
            //get A, B
            //get A - B
            //get dist to collision
            //get dist between
        
        //if circles overlap = danger (between current plane and closest neighbor)
            //get A, B
            //get A - B
            //get dist to collision
            //get dist between
        //update map   

	AU_UAV_ROS::PlanePose newPlane(msg->planeID, currentPose.x_coordinate, currentPose.y_coordinate, currentPose.altitude, newHeading);
	planeMap[msg->planeID] = newPlane; 

    }
}

/*
//This function will take inputs of min(A,B) and A-B and output true or false to enter the CA algorithm where A is the distance for the current plane to the collision point and B is the distance to collision point for the closest plane to current plane. 
bool firstFuzzyEngine(double distanceToCollision, double overlapDistance)
{
	bool collisionPotential = false; 	
	double output = fuzzy1.FuzzyLogicOne(distanceToCollision, overlapDistance);
	ROS_INFO("for dToColl= %f and overlap= %f", distanceToCollision, overlapDistance);
	ROS_INFO("fuzzy logic output = %f", output);
	
	if(output >=.4){
		collisionPotential = true; 
		ROS_INFO("collisionPotential is TRUE");
			
	}
	//ROS_INFO("decide to start CA? %s", collisionPotential);
   	
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
	ROS_INFO("for distBtwnPlanes= %f and bearingAngle= %f ", distBtwnPlanes, bearingAngle); 	
	ROS_INFO("Change in heading: %f", changeInHeading);
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
	
	//initialize counting
	count = 0;

	//needed for ROS to wait for callbacks
	ros::spin();	

	return 0;
}
