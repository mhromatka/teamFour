
/*

Fuzzy logic controllers for detecting collisions and
for determining appropriate collision avoidance maneuver

@authors Michelle Hromatka
         Jeffrey West

*/

#include "fuzzylite/test.h"
#include "fuzzylite/FuzzyLite.h"
#include <string>
#include <sstream>

//construct the Fuzzy Logic controller
AU_UAV_ROS::FuzzyLogicController::FuzzyLogicController(){
        FuzzyOperator& op = FuzzyOperator::DefaultFuzzyOperator();
        FuzzyEngine engine("Collison-Detection", op);
	//don't know what these guys do: 
        engine.hedgeSet().add(new fl::HedgeNot);
        engine.hedgeSet().add(new fl::HedgeSomewhat);
        engine.hedgeSet().add(new fl::HedgeVery);

        fl::InputLVar* distanceToCollision = new fl::InputLVar("CollDist");
        distanceToCollision->addTerm(new fl::ShoulderTerm("VERYCLOSE", 12.0, 24.0, true));
        distanceToCollision->addTerm(new fl::TriangularTerm("CLOSE", 20.0, 48.0));
        distanceToCollision->addTerm(new fl::TriangularTerm("FAR", 42.0, 78.0));
        distanceToCollision->addTerm(new fl::ShoulderTerm("VERYFAR", 66.0, 72.0, false));
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

	flScalar in1, in2; 
	for (fl::flScalar in = 0.0; in < 1.1; in += 0.1) {
	    in1 = 14.0;//in*80.0; //veryclose 
	    in2 = in*96.0 - 48.0; 
            distanceToCollision->setInput(in1);
	    aMinusB->setInput(in2);
            engine.process();
            fl::flScalar out = collImminence->output().defuzzify();
            (void)out; //Just to avoid warning when building

	    //compose string for info purposes
	    std::stringstream ss;
            ss << "DtoColl= " << in1 << "  Fuzzified= " << distanceToCollision->fuzzify(in1);
	    std::string s1(ss.str());
            ROS_INFO(s1.c_str()); 
	    ss.str(std::string());//clear sstream
 	    ss << "Overlap= " << in2 << "  Fuzzified= " << aMinusB->fuzzify(in2);
	    std::string s2(ss.str());
            ROS_INFO(s2.c_str());
	    ss.str(std::string());//clear sstream
 	    ss << "Output= " << out << "  Fuzzified= " << collImminence->fuzzify(out);
	    std::string s3(ss.str());
	    ROS_INFO(s3.c_str()); 
            ROS_INFO("--");
        }
}





