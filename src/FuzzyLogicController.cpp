
/*

Fuzzy logic controllers for detecting collisions and
for determining appropriate collision avoidance maneuver

@authors Michelle Hromatka
         Jeffrey West

*/

#include "ros/ros.h"
#include "fuzzylite/FuzzyLite.h"
#include "AU_UAV_ROS/FuzzyLogicController.h"
#include <string>
#include <sstream>

double AU_UAV_ROS::FuzzyLogicController::FuzzyLogicOne(double in1, double in2){
        fl::FuzzyOperator& op = fl::FuzzyOperator::DefaultFuzzyOperator();
     	fl::FuzzyEngine engine("Collison-Detection", op);
	//don't know what these guys do: 
        engine.hedgeSet().add(new fl::HedgeNot);
        engine.hedgeSet().add(new fl::HedgeSomewhat);
        engine.hedgeSet().add(new fl::HedgeVery);

        fl::InputLVar* distanceToCollision = new fl::InputLVar("CollDist");
        distanceToCollision->addTerm(new fl::ShoulderTerm("VERYCLOSE", 12.0, 24.0, true));
        distanceToCollision->addTerm(new fl::TriangularTerm("CLOSE", 20.0, 48.0));
        distanceToCollision->addTerm(new fl::TriangularTerm("FAR", 42.0, 78.0));
        distanceToCollision->addTerm(new fl::ShoulderTerm("VERYFAR", 70.0, 78.0, false));
        engine.addInputLVar(distanceToCollision);
      
        //aMinusB, where A is the distance to collision point for the plane of interest
        //and B is dist to Collision for the nearest plane, aMinusB determines whether a collision
        //will actually happen or if the planes will be at that collision point at different times
        fl::InputLVar* aMinusB = new fl::InputLVar("OverlapDistance");
	aMinusB->addTerm(new fl::ShoulderTerm("VERYNEG", -24.0, -16.0, true));
        aMinusB->addTerm(new fl::TriangularTerm("NEG", -20.0, -8.0));
        aMinusB->addTerm(new fl::TriangularTerm("ZERO", -12.0, 12.0));
        aMinusB->addTerm(new fl::TriangularTerm("POS", 8.0, 20.0));
	aMinusB->addTerm(new fl::ShoulderTerm("VERYPOS", 16.0, 24.0, false));
        engine.addInputLVar(aMinusB);

        fl::OutputLVar* collImminence = new fl::OutputLVar("CollisionImminence");
        collImminence->addTerm(new fl::ShoulderTerm("SAFE", 0.0, 0.4, true));
	collImminence->addTerm(new fl::TriangularTerm("POSSIBLE", 0.3, 0.7));
	collImminence->addTerm(new fl::ShoulderTerm("DANGER", 0.6, 1.0, false));
        engine.addOutputLVar(collImminence);

	fl::RuleBlock* block = new fl::RuleBlock();
        block->addRule(new fl::MamdaniRule("if CollDist is VERYCLOSE and OverlapDistance is VERYNEG then CollisionImminence is SAFE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is VERYCLOSE and OverlapDistance is NEG then CollisionImminence is very DANGER", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is VERYCLOSE and OverlapDistance is ZERO then CollisionImminence is very DANGER", engine));
	block->addRule(new fl::MamdaniRule("if CollDist is VERYCLOSE and OverlapDistance is POS then CollisionImminence is very DANGER", engine));
 	block->addRule(new fl::MamdaniRule("if CollDist is VERYCLOSE and OverlapDistance is VERYPOS then CollisionImminence is SAFE", engine));
	block->addRule(new fl::MamdaniRule("if CollDist is CLOSE and OverlapDistance is VERYNEG then CollisionImminence is very SAFE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is CLOSE and OverlapDistance is NEG then CollisionImminence is POSSIBLE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is CLOSE and OverlapDistance is ZERO then CollisionImminence is very DANGER", engine));
	block->addRule(new fl::MamdaniRule("if CollDist is CLOSE and OverlapDistance is POS then CollisionImminence is POSSIBLE", engine));
 	block->addRule(new fl::MamdaniRule("if CollDist is CLOSE and OverlapDistance is VERYPOS then CollisionImminence is very SAFE", engine));
	block->addRule(new fl::MamdaniRule("if CollDist is FAR and OverlapDistance is VERYNEG then CollisionImminence is very SAFE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is FAR and OverlapDistance is NEG then CollisionImminence is POSSIBLE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is FAR and OverlapDistance is ZERO then CollisionImminence is very POSSIBLE", engine));
	block->addRule(new fl::MamdaniRule("if CollDist is FAR and OverlapDistance is POS then CollisionImminence is POSSIBLE", engine));
 	block->addRule(new fl::MamdaniRule("if CollDist is FAR and OverlapDistance is VERYPOS then CollisionImminence is very SAFE", engine));
	block->addRule(new fl::MamdaniRule("if CollDist is VERYFAR and OverlapDistance is VERYNEG then CollisionImminence is very SAFE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is VERYFAR and OverlapDistance is NEG then CollisionImminence is somewhat POSSIBLE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is VERYFAR and OverlapDistance is ZERO then CollisionImminence is POSSIBLE", engine));
	block->addRule(new fl::MamdaniRule("if CollDist is VERYFAR and OverlapDistance is POS then CollisionImminence is somewhat POSSIBLE", engine));
	block->addRule(new fl::MamdaniRule("if CollDist is VERYFAR and OverlapDistance is VERYPOS then CollisionImminence is very SAFE", engine));

        engine.addRuleBlock(block);
	distanceToCollision->setInput(in1);
	aMinusB->setInput(in2);
        engine.process();
	return collImminence->output().defuzzify();

}

/*
double AU_UAV_ROS::FuzzyLogicController::FuzzyLogicTwo(double in1, double in2){
    fl::FuzzyOperator& op = fl::FuzzyOperator::DefaultFuzzyOperator();
    fl::FuzzyEngine engine("Heading-Change", op);
	//don't know what these guys do: 
    engine.hedgeSet().add(new fl::HedgeNot);
    engine.hedgeSet().add(new fl::HedgeSomewhat);
    engine.hedgeSet().add(new fl::HedgeVery);
    
    fl::InputLVar* distanceBetweenPlanes = new fl::InputLVar("PlaneDist");
    distanceBetweenPlanes->addTerm(new fl::ShoulderTerm("VERYCLOSE", 12.0, 26.0, true));
    distanceBetweenPlanes->addTerm(new fl::TriangularTerm("CLOSE", 18.0, 48.0));
    distanceBetweenPlanes->addTerm(new fl::TriangularTerm("FAR", 42.0, 78.0));
    distanceBetweenPlanes->addTerm(new fl::ShoulderTerm("VERY FAR", 70.0, 78.0, false));
    engine.addInputLVar(distanceBetweenPlanes);
    
    //aMinusB, where A is the distance to collision point for the plane of interest
    //and B is dist to Collision for the nearest plane, aMinusB determines whether a collision
    //will actually happen or if the planes will be at that collision point at different times
    fl::InputLVar* bearingAngle = new fl::InputLVar("BearingAngle");
	bearingAngle->addTerm(new fl::ShoulderTerm("VERYNEG", -90.0, -45.0, true));
    bearingAngle->addTerm(new fl::TriangularTerm("NEG", -90, -22.5));
    bearingAngle->addTerm(new fl::TriangularTerm("LOWNEG", -45.0, 2.0));
    bearingAngle->addTerm(new fl::TriangularTerm("LOWPOS", 2.0, 45.0));
    bearingAngle->addTerm(new fl::TriangularTerm("POS", 22.5, 90.0));
	bearingAngle->addTerm(new fl::ShoulderTerm("VERYPOS", 45, 90.0, false));
    engine.addInputLVar(bearingAngle);
    
    fl::OutputLVar* changeHeading = new fl::OutputLVar("ChangeInHeading");
    changeHeading->addTerm(new fl::TrapezoidalTerm("VERYLEFT", -22.5, 0.0, ));
    changeHeading->addTerm(new fl::ShoulderTerm("LEFT", -12.0, 0.0));
	changeHeading->addTerm(new fl::TriangularTerm("NOCHANGE", -22.5, 22.5));
	changeHeading->addTerm(new fl::ShoulderTerm("RIGHT", 12.0, 22.5));
	changeHeading->addTerm(new fl::ShoulderTerm("VERYRIGHT", 0.0, 22.5, false));    
    engine.addOutputLVar(changeHeading);
    
	fl::RuleBlock* block = new fl::RuleBlock();
    block->addRule(new fl::MamdaniRule("if PlaneDist is VERYCLOSE and BearingAngle is VERYNEG then ChangeInHeading is RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is VERYCLOSE and BearingAngle is NEG then ChangeInHeading is very VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is VERYCLOSE and BearingAngle is LOWNEG then ChangeInHeading is very VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is VERYCLOSE and BearingAngle is LOWPOS then ChangeInHeading is very VERYLEFT", engine));
	block->addRule(new fl::MamdaniRule("if PlaneDist is VERYCLOSE and BearingAngle is POS then ChangeInHeading is very VERYLEFT", engine));
 	block->addRule(new fl::MamdaniRule("if PlaneDist is VERYCLOSE and BearingAngle is VERYPOS then ChangeInHeading is LEFT", engine));
    
    block->addRule(new fl::MamdaniRule("if PlaneDist is CLOSE and BearingAngle is VERYNEG then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is CLOSE and BearingAngle is NEG then ChangeInHeading is very RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is CLOSE and BearingAngle is LOWNEG then ChangeInHeading is very VERYRIGHT", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is CLOSE and BearingAngle is LOWPOS then ChangeInHeading is very VERYLEFT", engine));
	block->addRule(new fl::MamdaniRule("if PlaneDist is CLOSE and BearingAngle is POS then ChangeInHeading is very LEFT", engine));
 	block->addRule(new fl::MamdaniRule("if PlaneDist is CLOSE and BearingAngle is VERYPOS then ChangeInHeading is NOCHANGE", engine));    
    
    block->addRule(new fl::MamdaniRule("if PlaneDist is FAR and BearingAngle is VERYNEG then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is FAR and BearingAngle is NEG then ChangeInHeading is very RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is FAR and BearingAngle is LOWNEG then ChangeInHeading is very RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is FAR and BearingAngle is LOWPOS then ChangeInHeading is very LEFT", engine));
	block->addRule(new fl::MamdaniRule("if PlaneDist is FAR and BearingAngle is POS then ChangeInHeading is very LEFT", engine));
 	block->addRule(new fl::MamdaniRule("if PlaneDist is FAR and BearingAngle is VERYPOS then ChangeInHeading is NOCHANGE", engine));
    
    block->addRule(new fl::MamdaniRule("if PlaneDist is VERYFAR and BearingAngle is VERYNEG then ChangeInHeading is NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is VERYFAR and BearingAngle is NEG then ChangeInHeading is very NOCHANGE", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is VERYFAR and BearingAngle is LOWNEG then ChangeInHeading is very RIGHT", engine));
    block->addRule(new fl::MamdaniRule("if PlaneDist is VERYFAR and BearingAngle is LOWPOS then ChangeInHeading is very LEFT", engine));
	block->addRule(new fl::MamdaniRule("if PlaneDist is VERYFAR and BearingAngle is POS then ChangeInHeading is very NOCHANGE", engine));
 	block->addRule(new fl::MamdaniRule("if PlaneDist is VERYFAR and BearingAngle is VERYPOS then ChangeInHeading is NOCHANGE", engine));
    
    engine.addRuleBlock(block);
	distanceBetweenPlanes->setInput(in1);
	bearingAngle->setInput(in2);
    engine.process();
	return changeHeading->output().defuzzify();
    
}
 */
