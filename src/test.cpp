/*   Copyright 2010 Juan Rada-Vilela

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implie
   See the License for the specific language governing permissions and
   limitations under the License.
 */
#include "fuzzylite/test.h"
#include "fuzzylite/FuzzyLite.h"
#include "fuzzylite/OutputLVar.h"
#include "fuzzylite/InputLVar.h"
#include "AU_UAV_ROS/FuzzyLogicController.h"
#include <limits>

#include "fuzzylite/FunctionTerm.h"

AU_UAV_ROS::FuzzyLogicController fl1; 
namespace fl {
	void Test::FuzzyLogicOne(){

        FuzzyOperator& op = FuzzyOperator::DefaultFuzzyOperator();
        FuzzyEngine engine("Collison-Detection", op);

        fl::InputLVar* distanceToCollision = new fl::InputLVar("CollDist");
        distanceToCollision->addTerm(new fl::ShoulderTerm("VERYCLOSE", 12.0, 26.0, true));
        distanceToCollision->addTerm(new fl::TriangularTerm("CLOSE", 18.0, 48.0));
       	distanceToCollision->addTerm(new fl::TriangularTerm("FAR", 42.0, 78.0));//42
        distanceToCollision->addTerm(new fl::ShoulderTerm("VERYFAR", 70.0, 78.0, false));
        engine.addInputLVar(distanceToCollision);
      
        //aMinusB, where A is the distance to collision point for the plane of interest
        //and B is dist to Collision for the nearest plane, aMinusB determines whether a collision
        //will actually happen or if the planes will be at that collision point at different times
        fl::InputLVar* aMinusB = new fl::InputLVar("OverlapDistance");
	aMinusB->addTerm(new fl::ShoulderTerm("VERYNEG", -24.0, -16.0, true));
        aMinusB->addTerm(new fl::TriangularTerm("NEG", -20.0, -8.0));
        aMinusB->addTerm(new fl::TriangularTerm("ZERO", -12.0, 12.0));
        aMinusB->addTerm(new fl::ShoulderTerm("POS", 8.0, 20.0, false));
	aMinusB->addTerm(new fl::ShoulderTerm("VERYPOS", 16.0, 24.0, false));
        engine.addInputLVar(aMinusB);

        fl::OutputLVar* collImminence = new fl::OutputLVar("CollisionImminence");
        collImminence->addTerm(new fl::ShoulderTerm("SAFE", 0.0, 0.4, true));
	collImminence->addTerm(new fl::TriangularTerm("POSSIBLE", 0.25, 0.75));
	collImminence->addTerm(new fl::ShoulderTerm("DANGER", 0.6, 1.0, false));
        engine.addOutputLVar(collImminence);

	fl::RuleBlock* block = new fl::RuleBlock();
        block->addRule(new fl::MamdaniRule("if CollDist is VERYCLOSE and OverlapDistance is VERYNEG then CollisionImminence is SAFE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is VERYCLOSE and OverlapDistance is ZERO then CollisionImminence is DANGER", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is VERYCLOSE and OverlapDistance is NEG then CollisionImminence is DANGER", engine));
	block->addRule(new fl::MamdaniRule("if CollDist is VERYCLOSE and OverlapDistance is POS then CollisionImminence is DANGER", engine));
 	block->addRule(new fl::MamdaniRule("if CollDist is VERYCLOSE and OverlapDistance is VERYPOS then CollisionImminence is SAFE", engine));
	block->addRule(new fl::MamdaniRule("if CollDist is CLOSE and OverlapDistance is VERYNEG then CollisionImminence is SAFE", engine)); 
        block->addRule(new fl::MamdaniRule("if CollDist is CLOSE and OverlapDistance is NEG then CollisionImminence is POSSIBLE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is CLOSE and OverlapDistance is ZERO then CollisionImminence is DANGER", engine));
	block->addRule(new fl::MamdaniRule("if CollDist is CLOSE and OverlapDistance is POS then CollisionImminence is POSSIBLE", engine));
 	block->addRule(new fl::MamdaniRule("if CollDist is CLOSE and OverlapDistance is VERYPOS then CollisionImminence is SAFE", engine));
	block->addRule(new fl::MamdaniRule("if CollDist is FAR and OverlapDistance is VERYNEG then CollisionImminence is SAFE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is FAR and OverlapDistance is NEG then CollisionImminence is POSSIBLE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is FAR and OverlapDistance is ZERO then CollisionImminence is POSSIBLE", engine));
	block->addRule(new fl::MamdaniRule("if CollDist is FAR and OverlapDistance is POS then CollisionImminence is POSSIBLE", engine));
 	block->addRule(new fl::MamdaniRule("if CollDist is FAR and OverlapDistance is VERYPOS then CollisionImminence is SAFE", engine));
	block->addRule(new fl::MamdaniRule("if CollDist is VERYFAR and OverlapDistance is VERYNEG then CollisionImminence is SAFE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is VERYFAR and OverlapDistance is NEG then CollisionImminence is POSSIBLE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is VERYFAR and OverlapDistance is ZERO then CollisionImminence is POSSIBLE", engine));
	block->addRule(new fl::MamdaniRule("if CollDist is VERYFAR and OverlapDistance is POS then CollisionImminence is POSSIBLE", engine));
	block->addRule(new fl::MamdaniRule("if CollDist is VERYFAR and OverlapDistance is VERYPOS then CollisionImminence is SAFE", engine));

        engine.addRuleBlock(block);



/*
	fl::FuzzyOperator& op = fl::FuzzyOperator::DefaultFuzzyOperator();
     	fl::FuzzyEngine engine("Collison-Detection", op);

        fl::InputLVar* distanceToCollision = new fl::InputLVar("CollDist");
        distanceToCollision->addTerm(new fl::ShoulderTerm("VERYCLOSE", 12.0, 26.0, true));
        distanceToCollision->addTerm(new fl::TriangularTerm("CLOSE", 18.0, 48.0));
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
	collImminence->addTerm(new fl::TriangularTerm("POSSIBLE", 0.2, 0.8));
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
*/
	fl::flScalar in1 = 22.0, in2; 
        for (fl::flScalar in = 0.0; in < 200.0; in += 4.0) {
	    in2 = in;
            distanceToCollision->setInput(in1);
            aMinusB->setInput(in2);
            engine.process();
            fl::flScalar out = collImminence->output().defuzzify();
            (void)out; //Just to avoid warning when building
	    std::stringstream ss;
            ss << "DtoColl= " << in1 << "  Fuzzified= " << distanceToCollision->fuzzify(in1);
	    std::string s1(ss.str());
            ROS_INFO(s1.c_str()); 
	    ss.str(std::string());//clear sstream
 	    ss << "Overlap= "<< in2 << "  Fuzzified= " << aMinusB->fuzzify(in2);
	    std::string s2(ss.str());
            ROS_INFO(s2.c_str());
	    ss.str(std::string());//clear sstream
 	    ss << "Output= " << out << "  Fuzzified= " << collImminence->fuzzify(out);
	    std::string s3(ss.str());
	    ROS_INFO(s3.c_str()); 
        }
/*
	flScalar in1, in2; 
	for (fl::flScalar in = 0.0; in <300.0; in+= 10) {
	    in1 = 6.0;//in*80.0; //veryclose 
	    in2 = in - 150.0; 
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
	ROS_INFO("**************STARTING AGAIN WITH FIXED OVERLAP OF 8***************");
	for (fl::flScalar in = 0.0; in <200; in+=5) {
	    in1 = in;//in*80.0; //veryclose 
	    in2 = 8.0; 
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
*/}

void Test::FuzzyLogicTwo(){
        fl::FuzzyOperator& op = fl::FuzzyOperator::DefaultFuzzyOperator();
        fl::FuzzyEngine engine2("Heading-Change", op);

        fl::InputLVar* distanceBetweenPlanes = new fl::InputLVar("PlaneDist");
        distanceBetweenPlanes->addTerm(new fl::ShoulderTerm("VERYCLOSE", 12.0, 26.0, true));
        distanceBetweenPlanes->addTerm(new fl::TriangularTerm("CLOSE", 18.0, 48.0));
        distanceBetweenPlanes->addTerm(new fl::TriangularTerm("FAR", 42.0, 78.0));
        distanceBetweenPlanes->addTerm(new fl::ShoulderTerm("VERYFAR", 70.0, 78.0, false));
        engine2.addInputLVar(distanceBetweenPlanes);
        
        //aMinusB, where A is the distance to collision point for the plane of interest
        //and B is dist to Collision for the nearest plane, aMinusB determines whether a collision
        //will actually happen or if the planes will be at that collision point at different times
        fl::InputLVar* bearingAngle = new fl::InputLVar("BearingAngle");
        bearingAngle->addTerm(new fl::ShoulderTerm("VERYNEG", -90.0, -45.0, true));
        bearingAngle->addTerm(new fl::TriangularTerm("NEG", -89, -22.5));
        bearingAngle->addTerm(new fl::TriangularTerm("LOWNEG", -45.0, 2.0));
        bearingAngle->addTerm(new fl::TriangularTerm("LOWPOS", 2.0, 45.0));
        bearingAngle->addTerm(new fl::TriangularTerm("POS", 22.5, 89.0));
        bearingAngle->addTerm(new fl::ShoulderTerm("VERYPOS", 45, 90.0, false));
        engine2.addInputLVar(bearingAngle);
        fl::OutputLVar* changeHeading = new fl::OutputLVar("ChangeInHeading");
        changeHeading->addTerm(new fl::ShoulderTerm("VERYLEFT", -22.5, 0.0, true));
        changeHeading->addTerm(new fl::TriangularTerm("LEFT", -12.0, 0.0));
        changeHeading->addTerm(new fl::TriangularTerm("NOCHANGE", -12.0, 12.0));
        changeHeading->addTerm(new fl::TriangularTerm("RIGHT", 0.0, 12.0));
        changeHeading->addTerm(new fl::ShoulderTerm("VERYRIGHT", 0.0, 22.5, false));    
        engine2.addOutputLVar(changeHeading);

        fl::RuleBlock* block = new fl::RuleBlock();
        block->addRule(new fl::MamdaniRule("if PlaneDist is VERYCLOSE and BearingAngle is VERYNEG then ChangeInHeading is RIGHT", engine2));
        block->addRule(new fl::MamdaniRule("if PlaneDist is VERYCLOSE and BearingAngle is NEG then ChangeInHeading is VERYRIGHT", engine2));
        block->addRule(new fl::MamdaniRule("if PlaneDist is VERYCLOSE and BearingAngle is LOWNEG then ChangeInHeading is VERYRIGHT", engine2));
        block->addRule(new fl::MamdaniRule("if PlaneDist is VERYCLOSE and BearingAngle is LOWPOS then ChangeInHeading is VERYLEFT", engine2));
        block->addRule(new fl::MamdaniRule("if PlaneDist is VERYCLOSE and BearingAngle is POS then ChangeInHeading is VERYLEFT", engine2));
        block->addRule(new fl::MamdaniRule("if PlaneDist is VERYCLOSE and BearingAngle is VERYPOS then ChangeInHeading is LEFT", engine2));
        
        block->addRule(new fl::MamdaniRule("if PlaneDist is CLOSE and BearingAngle is VERYNEG then ChangeInHeading is NOCHANGE", engine2));
        block->addRule(new fl::MamdaniRule("if PlaneDist is CLOSE and BearingAngle is NEG then ChangeInHeading is RIGHT", engine2));
        block->addRule(new fl::MamdaniRule("if PlaneDist is CLOSE and BearingAngle is LOWNEG then ChangeInHeading is VERYRIGHT", engine2));
        block->addRule(new fl::MamdaniRule("if PlaneDist is CLOSE and BearingAngle is LOWPOS then ChangeInHeading is VERYLEFT", engine2));
        block->addRule(new fl::MamdaniRule("if PlaneDist is CLOSE and BearingAngle is POS then ChangeInHeading is LEFT", engine2));
        block->addRule(new fl::MamdaniRule("if PlaneDist is CLOSE and BearingAngle is VERYPOS then ChangeInHeading is NOCHANGE", engine2));    
        
        block->addRule(new fl::MamdaniRule("if PlaneDist is FAR and BearingAngle is VERYNEG then ChangeInHeading is NOCHANGE", engine2));
        block->addRule(new fl::MamdaniRule("if PlaneDist is FAR and BearingAngle is NEG then ChangeInHeading is RIGHT", engine2));
        block->addRule(new fl::MamdaniRule("if PlaneDist is FAR and BearingAngle is LOWNEG then ChangeInHeading is RIGHT", engine2));
        block->addRule(new fl::MamdaniRule("if PlaneDist is FAR and BearingAngle is LOWPOS then ChangeInHeading is LEFT", engine2));
        block->addRule(new fl::MamdaniRule("if PlaneDist is FAR and BearingAngle is POS then ChangeInHeading is LEFT", engine2));
        block->addRule(new fl::MamdaniRule("if PlaneDist is FAR and BearingAngle is VERYPOS then ChangeInHeading is NOCHANGE", engine2));
        
        block->addRule(new fl::MamdaniRule("if PlaneDist is VERYFAR and BearingAngle is VERYNEG then ChangeInHeading is NOCHANGE", engine2));
        block->addRule(new fl::MamdaniRule("if PlaneDist is VERYFAR and BearingAngle is NEG then ChangeInHeading is NOCHANGE", engine2));
        block->addRule(new fl::MamdaniRule("if PlaneDist is VERYFAR and BearingAngle is LOWNEG then ChangeInHeading is RIGHT", engine2));
        block->addRule(new fl::MamdaniRule("if PlaneDist is VERYFAR and BearingAngle is LOWPOS then ChangeInHeading is LEFT", engine2));
        block->addRule(new fl::MamdaniRule("if PlaneDist is VERYFAR and BearingAngle is POS then ChangeInHeading is NOCHANGE", engine2));
        block->addRule(new fl::MamdaniRule("if PlaneDist is VERYFAR and BearingAngle is VERYPOS then ChangeInHeading is NOCHANGE", engine2));
        engine2.addRuleBlock(block); 
        
        for (fl::flScalar in2 = -100.0; in2 < 100.0; in2 += 15.0){
            for (fl::flScalar in1 = 0.0; in1 < 80.0; in1 += 10.0){
                distanceBetweenPlanes->setInput(in1);
                bearingAngle->setInput(in2);
               
                engine2.process();
                
                fl::flScalar out = changeHeading->output().defuzzify();
		ROS_INFO("3");
		sleep(2);                
		ROS_INFO("4");
                (void)out; //Just to avoid warning when building
                std::stringstream ss;
                ss << "Dist BTWN Planes = " << in1 << "  Fuzzified= " << distanceBetweenPlanes->fuzzify(in1);
                std::string s1(ss.str());
                ROS_INFO(s1.c_str()); 
                ss.str(std::string());//clear sstream
                ss << "Bearing Angle = -200" << "  Fuzzified= " << bearingAngle->fuzzify(in2);
                std::string s2(ss.str());
                ROS_INFO(s2.c_str());
                ss.str(std::string());//clear sstream
                ss << "Output = " << out << "  Fuzzified= " << changeHeading->fuzzify(out);
                std::string s3(ss.str());
                ROS_INFO(s3.c_str()); 
                
            }
        }
    }

/*

    void Test::SimpleMamdani() {
        FuzzyOperator& op = FuzzyOperator::DefaultFuzzyOperator();
        FuzzyEngine engine("simple-mamdani", op);
        engine.hedgeSet().add(new fl::HedgeNot);
        engine.hedgeSet().add(new fl::HedgeSomewhat);
        engine.hedgeSet().add(new fl::HedgeVery);
        fl::InputLVar* energy = new fl::InputLVar("Energy");
        energy->addTerm(new fl::ShoulderTerm("LOW", 0.25, 0.5, true));
        energy->addTerm(new fl::TriangularTerm("MEDIUM", 0.25, 0.75));
        energy->addTerm(new fl::ShoulderTerm("HIGH", 0.50, 0.75, false));
        engine.addInputLVar(energy);

        fl::OutputLVar* health = new fl::OutputLVar("Health");
        health->addTerm(new fl::TriangularTerm("BAD", 0.0, 0.50));
        health->addTerm(new fl::TriangularTerm("REGULAR", 0.25, 0.75));
        health->addTerm(new fl::TriangularTerm("GOOD", 0.50, 1.00));
        engine.addOutputLVar(health);
        fl::RuleBlock* block = new fl::RuleBlock();
        block->addRule(new fl::MamdaniRule("if Energy is LOW then Health is BAD", engine));
        block->addRule(new fl::MamdaniRule("if Energy is MEDIUM then Health is REGULAR", engine));
        block->addRule(new fl::MamdaniRule("if Energy is HIGH then Health is GOOD", engine));
        engine.addRuleBlock(block);

        for (fl::flScalar in = 0.0; in < 1.1; in += 0.1) {
            energy->setInput(in);
            engine.process();
            fl::flScalar out = health->output().defuzzify();
            (void)out; //Just to avoid warning when building
            FL_LOG("Energy=" << in);
            FL_LOG("Energy is " << energy->fuzzify(in));
            FL_LOG("Health=" << out);
            FL_LOG("Health is " << health->fuzzify(out));
            FL_LOG("--");
        }
    }
*/
    void Test::ComplexMamdani() {
        FuzzyOperator& op = FuzzyOperator::DefaultFuzzyOperator();
        FuzzyEngine engine("complex-mamdani", op);

        engine.hedgeSet().add(new fl::HedgeNot);
        engine.hedgeSet().add(new fl::HedgeSomewhat);
        engine.hedgeSet().add(new fl::HedgeVery);



        fl::InputLVar* energy = new fl::InputLVar("Energy");
        energy->addTerm(new fl::TriangularTerm("LOW", 0.0, 0.50));
        energy->addTerm(new fl::TriangularTerm("MEDIUM", 0.25, 0.75));
        energy->addTerm(new fl::TriangularTerm("HIGH", 0.50, 1.00));
        engine.addInputLVar(energy);

        fl::InputLVar* distance = new fl::InputLVar("Distance");
        distance->addTerm(new fl::TriangularTerm("NEAR", 0, 500));
        distance->addTerm(new fl::TriangularTerm("FAR", 250, 750));
        distance->addTerm(new fl::TriangularTerm("FAR_AWAY", 500, 1000));
        engine.addInputLVar(distance);

        fl::OutputLVar* power = new fl::OutputLVar("Power");
        power->addTerm(new fl::ShoulderTerm("LOW", 0.25, 0.5, true));
        power->addTerm(new fl::TriangularTerm("MEDIUM", 0.25, 0.75));
        power->addTerm(new fl::ShoulderTerm("HIGH", 0.50, 0.75, false));
        engine.addOutputLVar(power);

        fl::RuleBlock* block = new fl::RuleBlock();
        block->addRule(new fl::MamdaniRule("if Energy is LOW and Distance is FAR_AWAY then Power is LOW", engine));
        block->addRule(new fl::MamdaniRule("if Energy is LOW and Distance is FAR then Power is very MEDIUM", engine));
        block->addRule(new fl::MamdaniRule("if Energy is LOW and Distance is NEAR then Power is HIGH", engine));
        block->addRule(new fl::MamdaniRule("if Energy is MEDIUM and Distance is FAR_AWAY then Power is LOW with 0.8", engine));
        block->addRule(new fl::MamdaniRule("if Energy is MEDIUM and Distance is FAR then Power is MEDIUM with 1.0", engine));
        block->addRule(new fl::MamdaniRule("if Energy is MEDIUM and Distance is NEAR then Power is HIGH with 0.3", engine));
        block->addRule(new fl::MamdaniRule("if Energy is HIGH and Distance is FAR_AWAY then Power is LOW with 0.43333", engine));
        block->addRule(new fl::MamdaniRule("if Energy is HIGH and Distance is FAR then Power is MEDIUM", engine));
        block->addRule(new fl::MamdaniRule("if Energy is HIGH and Distance is NEAR then Power is HIGH", engine));
        engine.addRuleBlock(block);

        for (int i = 0; i < block->numberOfRules(); ++i) {
            FL_LOG(block->rule(i)->toString());
        }

        return;
        for (fl::flScalar in = 0.0; in < 1.1; in += 0.1) {
            energy->setInput(in);
            distance->setInput(500);
            engine.process();
            fl::flScalar out = power->output().defuzzify();
            (void)out; //Just to avoid warning when building
            FL_LOG("Energy=" << in);
            FL_LOG("Energy is " << power->fuzzify(in));
            FL_LOG("Health=" << out);
            FL_LOG("Health is " << energy->fuzzify(out));
            FL_LOG("--");
        }
    }


    void Test::main(int args, char** argv) {

	ROS_INFO("starting MfuzzyControllerTwo1 in 2 seconds");
	ROS_INFO("======================================");
	sleep(2);
     	//for (fl::flScalar in2 = -50.0; in2 < 1.0; in2 += 5.0){
		ROS_INFO("*****************************************");
		fl::flScalar in1 = 8.0;
		fl::flScalar in3 = 14.0;
            for (fl::flScalar in2 = -150; in2 < 151.0; in2 += 20.0){       
                fl::flScalar out = fl1.MFuzzyLogicTwo1(in1, in2, in3);
                (void)out; //Just to avoid warning when building
                std::stringstream ss;
                ss << "CollImm = " << in1;
                std::string s1(ss.str());
                ROS_INFO(s1.c_str()); 
                ss.str(std::string());//clear sstream
                ss << "CollAngle = " << in2; 
                std::string s2(ss.str());
                ROS_INFO(s2.c_str());
		ss.str(std::string());//clear sstream
                ss << "Overlap = " << in3; 
                std::string s4(ss.str());
                ROS_INFO(s4.c_str());
                ss.str(std::string());//clear sstream
                ss << "Output = " << out ;
                std::string s3(ss.str());
                ROS_INFO(s3.c_str());
		ROS_INFO("--");        
            }
      //  }
	ROS_INFO("======================================");
  
/*
        FL_LOG("Starting in 2 second");
        FL_LOG("Example: Simple Pendulum");
        FL_LOG("========================");
        sleep(2);

        SimplePendulum();
        FL_LOG("=======================\n");

        FL_LOG("Starting in 2 second");
        FL_LOG("Example: Simple Takagi-Sugeno");
        FL_LOG("========================");
        sleep(2);

        SimpleTakagiSugeno();
        FL_LOG("=======================\n");

        FL_LOG("For further examples build the GUI...");
    }
*/
}
}

