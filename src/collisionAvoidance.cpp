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
#include "AU_UAV_ROS/standardFuncs.h"

//ROS service client for calling a service from the coordinator
ros::ServiceClient client;

//keeps count of the number of services requested
int count;
double heading = 0.0;//check to make sure planes actually do initiate with 0.0 heading????

std::map<int,AU_UAV_ROS::PlanePose> planeMap;


//this function is run everytime new telemetry information from any plane is recieved
void telemetryCallback(const AU_UAV_ROS::TelemetryUpdate::ConstPtr& msg)
{
	if(planeMap.count(msg->planeID)==0)
    {
        //AU_UAV_ROS::PlanePose newPlane(msg->planeID, msg->currentLatitude, msg->currentLongitude, msg->currentAltitude, 0.0);// = new AU_UAV_ROS::PlanePose::PlanePose(msg->planeID, msg->currentLatitude, msg->currentLatitude, msg->currentAltitude, 0.0);

        //planeMap [msg->planeID]=newPlane;
        //add this UAV's info the map
        
        //perform some other calcs for Fuzzy Input
    }
    else
    {
    //don't update map here yet
    
    //Decide to enter Collision Avoidance or Not?
    // 1) purely based on distance
    // 2) based on FL with inputs of distance and A-B
    
    //get new heading

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
        

        
        
    }
	



	//this 'if' statement will be changed to run when collision avoidance service needs to be used
	if(false) //collision detection OR run everytime
	{
		//this will be replaced by students to do more than just send a string
		std::stringstream ss;
		ss << "Sending service request " << count++;

		//dummying up a service request for the REU students to see
		AU_UAV_ROS::GoToWaypoint srv;
		srv.request.planeID = msg->planeID;
		srv.request.latitude = 32.606573;
		srv.request.longitude = -85.490356;
		srv.request.altitude = 400;
		
		//these settings mean it is an avoidance maneuver waypoint AND to clear the avoidance queue
		srv.request.isAvoidanceManeuver = true;
		srv.request.isNewQueue = false;

		//check to make sure the client call worked (regardless of return values from service)
		if(client.call(srv))
		{
			ROS_INFO("Received response from service request %d", (count-1));
		}
		else
		{
			ROS_ERROR("Did not receive response");
		}
	}
	//else goes back to path planning (straight line) unless clear the que?
}

int main(int argc, char **argv)
{
	//standard ROS startup
	ros::init(argc, argv, "collisionAvoidance");
	ros::NodeHandle n;
	
	//subscribe to telemetry outputs and create client for the avoid collision service
	ros::Subscriber sub = n.subscribe("telemetry", 1000, telemetryCallback);
	client = n.serviceClient<AU_UAV_ROS::GoToWaypoint>("go_to_waypoint");
	
	//random seed for if statement in telemetryCallback, remove when collision avoidance work begins
	srand(time(NULL));
	
	//initialize counting
	count = 0;

	//needed for ROS to wait for callbacks
	ros::spin();	

	return 0;
}
