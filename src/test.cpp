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
/*    void Test::FuzzyLogicOne(){
        FuzzyOperator& op = FuzzyOperator::DefaultFuzzyOperator();
        FuzzyEngine engine("Collison-Detection", op);
        
        fl::InputLVar* distanceToCollision = new fl::InputLVar("CollDist");
        distanceToCollision->addTerm(new fl::ShoulderTerm("VERYCLOSE", 12.0, 26.0, true));
        distanceToCollision->addTerm(new fl::TriangularTerm("CLOSE", 18.0, 48.0));
        distanceToCollision->addTerm(new fl::TriangularTerm("FAR", 42.0, 78.0));//42
        distanceToCollision->addTerm(new fl::ShoulderTerm("VERYFAR", 70.0, 78.0, false));
        //distanceToCollision->addTerm(new fl::ShoulderTerm("VERYCLOSE", 0.0, .5, true));
        //distanceToCollision->addTerm(new fl::TriangularTerm("CLOSE", .25, .75));
        //distanceToCollision->addTerm(new fl::TriangularTerm("FAR", .5, 1.0));//42
        //distanceToCollision->addTerm(new fl::ShoulderTerm("VERYFAR", .75, 1.0, false));
        engine.addInputLVar(distanceToCollision);
        
        //aMinusB, where A is the distance to collision point for the plane of interest
        //and B is dist to Collision for the nearest plane, aMinusB determines whether a collision
        //will actually happen or if the planes will be at that collision point at different times
        fl::InputLVar* aMinusB = new fl::InputLVar("OverlapDistance");
        aMinusB->addTerm(new fl::ShoulderTerm("VERYNEG", -24.0, 16.0, true));
        aMinusB->addTerm(new fl::TriangularTerm("NEG", -20.0, -8.0));
        aMinusB->addTerm(new fl::TriangularTerm("ZERO", -12.0, 12));
        aMinusB->addTerm(new fl::TriangularTerm("POS", 8.0, 20.0));
        aMinusB->addTerm(new fl::ShoulderTerm("VERYPOS", 16.0, 24.0, false));
        engine.addInputLVar(aMinusB);
        
        fl::OutputLVar* collImminence = new fl::OutputLVar("CollisionImminence");
        //collImminence->addTerm(new fl::ShoulderTerm("SAFE", 100.0, 140.0, true));
        //collImminence->addTerm(new fl::TriangularTerm("POSSIBLE", 130.0, 170.0));
        //collImminence->addTerm(new fl::ShoulderTerm("DANGER", 160.0, 200.0, false));
        collImminence->addTerm(new fl::ShoulderTerm("SAFE", 0.0, 0.4));
        collImminence->addTerm(new fl::TriangularTerm("POSSIBLE", 0.3, 0.7));
        collImminence->addTerm(new fl::ShoulderTerm("DANGER", 0.6, 1.0));
        engine.addOutputLVar(collImminence);
        
        fl::RuleBlock* block = new fl::RuleBlock();
        block->addRule(new fl::MamdaniRule("if CollDist is VERYCLOSE or OverlapDistance is VERYNEG then CollisionImminence is SAFE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is VERYCLOSE or OverlapDistance is NEG then CollisionImminence is DANGER", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is VERYCLOSE or OverlapDistance is ZERO then CollisionImminence is DANGER", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is VERYCLOSE or OverlapDistance is POS then CollisionImminence is DANGER", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is VERYCLOSE or OverlapDistance is VERYPOS then CollisionImminence is SAFE", engine));
        
        block->addRule(new fl::MamdaniRule("if CollDist is CLOSE or OverlapDistance is VERYNEG then CollisionImminence is SAFE", engine)); 
        block->addRule(new fl::MamdaniRule("if CollDist is CLOSE or OverlapDistance is NEG then CollisionImminence is POSSIBLE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is CLOSE or OverlapDistance is ZERO then CollisionImminence is DANGER", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is CLOSE or OverlapDistance is POS then CollisionImminence is POSSIBLE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is CLOSE or OverlapDistance is VERYPOS then CollisionImminence is SAFE", engine));
        
        block->addRule(new fl::MamdaniRule("if CollDist is FAR or OverlapDistance is VERYNEG then CollisionImminence is SAFE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is FAR or OverlapDistance is NEG then CollisionImminence is POSSIBLE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is FAR or OverlapDistance is ZERO then CollisionImminence is POSSIBLE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is FAR or OverlapDistance is POS then CollisionImminence is POSSIBLE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is FAR or OverlapDistance is VERYPOS then CollisionImminence is SAFE", engine));
        
        block->addRule(new fl::MamdaniRule("if CollDist is VERYFAR or OverlapDistance is VERYNEG then CollisionImminence is SAFE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is VERYFAR or OverlapDistance is NEG then CollisionImminence is POSSIBLE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is VERYFAR or OverlapDistance is ZERO then CollisionImminence is POSSIBLE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is VERYFAR or OverlapDistance is POS then CollisionImminence is POSSIBLE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is VERYFAR or OverlapDistance is VERYPOS then CollisionImminence is SAFE", engine));
        
        
        
        
        
        
        
        
        
        
    /*    
        block->addRule(new fl::MamdaniRule("if CollDist is VERYCLOSE and OverlapDistance is VERYNEG then CollisionImminence is SAFE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is VERYCLOSE and OverlapDistance is NEG then CollisionImminence is POSSIBLE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is VERYCLOSE and OverlapDistance is ZERO then CollisionImminence is POSSIBLE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is VERYCLOSE and OverlapDistance is POS then CollisionImminence is SAFE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is VERYCLOSE and OverlapDistance is VERYPOS then CollisionImminence is SAFE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is CLOSE and OverlapDistance is VERYNEG then CollisionImminence is POSSIBLE", engine)); 
        block->addRule(new fl::MamdaniRule("if CollDist is CLOSE and OverlapDistance is NEG then CollisionImminence is DANGER", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is CLOSE and OverlapDistance is ZERO then CollisionImminence is SAFE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is CLOSE and OverlapDistance is POS then CollisionImminence is SAFE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is CLOSE and OverlapDistance is VERYPOS then CollisionImminence is SAFE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is FAR and OverlapDistance is VERYNEG then CollisionImminence is DANGER", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is FAR and OverlapDistance is NEG then CollisionImminence is DANGER", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is FAR and OverlapDistance is ZERO then CollisionImminence is DANGER", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is FAR and OverlapDistance is POS then CollisionImminence is SAFE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is FAR and OverlapDistance is VERYPOS then CollisionImminence is SAFE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is VERYFAR and OverlapDistance is VERYNEG then CollisionImminence is SAFE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is VERYFAR and OverlapDistance is NEG then CollisionImminence is SAFE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is VERYFAR and OverlapDistance is ZERO then CollisionImminence is POSSIBLE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is VERYFAR and OverlapDistance is POS then CollisionImminence is SAFE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is VERYFAR and OverlapDistance is VERYPOS then CollisionImminence is SAFE", engine));
        
     
        engine.addRuleBlock(block);
        for (fl::flScalar in2 = -25.0; in2 < 25.0; in2 += 5.0){
            for (fl::flScalar in = -2.5; in < 80.0; in += 3.0) {
                distanceToCollision->setInput(in);
                aMinusB->setInput(in2);
                engine.process();
                fl::flScalar out = collImminence->output().defuzzify();
                (void)out; //Just to avoid warning when building
                std::stringstream ss;
                ss << "DtoColl= " << in << "  Fuzzified= " << distanceToCollision->fuzzify(in);
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
    }
     */  

    void Test::FuzzyLogicTwo(){
        fl::FuzzyOperator& op = fl::FuzzyOperator::DefaultFuzzyOperator();
        fl::FuzzyEngine engine("Heading-Change", op);

        fl::InputLVar* distanceBetweenPlanes = new fl::InputLVar("PlaneDist");
        distanceBetweenPlanes->addTerm(new fl::ShoulderTerm("VERYCLOSE", 12.0, 26.0, true));
        distanceBetweenPlanes->addTerm(new fl::TriangularTerm("CLOSE", 18.0, 48.0));
        distanceBetweenPlanes->addTerm(new fl::TriangularTerm("FAR", 42.0, 78.0));
        distanceBetweenPlanes->addTerm(new fl::ShoulderTerm("VERYFAR", 70.0, 78.0, false));
        engine.addInputLVar(distanceBetweenPlanes);
        
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
        engine.addInputLVar(bearingAngle);
        fl::OutputLVar* changeHeading = new fl::OutputLVar("ChangeInHeading");
        changeHeading->addTerm(new fl::ShoulderTerm("VERYLEFT", -45, 0.0, true));
        changeHeading->addTerm(new fl::TriangularTerm("LEFT", -22.5, 0.0));
        changeHeading->addTerm(new fl::TriangularTerm("NOCHANGE", -22.5, 22.5));
        changeHeading->addTerm(new fl::TriangularTerm("RIGHT", 0.0, 22.5));
        changeHeading->addTerm(new fl::ShoulderTerm("VERYRIGHT", 0.0, 45, false));    
        engine.addOutputLVar(changeHeading);

        fl::RuleBlock* block = new fl::RuleBlock();
        block->addRule(new fl::MamdaniRule("if PlaneDist is VERYCLOSE and BearingAngle is VERYNEG then ChangeInHeading is VERYRIGHT", engine));
        block->addRule(new fl::MamdaniRule("if PlaneDist is VERYCLOSE and BearingAngle is NEG then ChangeInHeading is VERYRIGHT", engine));
        block->addRule(new fl::MamdaniRule("if PlaneDist is VERYCLOSE and BearingAngle is LOWNEG then ChangeInHeading is VERYRIGHT", engine));
        block->addRule(new fl::MamdaniRule("if PlaneDist is VERYCLOSE and BearingAngle is LOWPOS then ChangeInHeading is VERYLEFT", engine));
        block->addRule(new fl::MamdaniRule("if PlaneDist is VERYCLOSE and BearingAngle is POS then ChangeInHeading is VERYLEFT", engine));
        block->addRule(new fl::MamdaniRule("if PlaneDist is VERYCLOSE and BearingAngle is VERYPOS then ChangeInHeading is VERYLEFT", engine));
        
        block->addRule(new fl::MamdaniRule("if PlaneDist is CLOSE and BearingAngle is VERYNEG then ChangeInHeading is RIGHT", engine));
        block->addRule(new fl::MamdaniRule("if PlaneDist is CLOSE and BearingAngle is NEG then ChangeInHeading is RIGHT", engine));
        block->addRule(new fl::MamdaniRule("if PlaneDist is CLOSE and BearingAngle is LOWNEG then ChangeInHeading is VERYRIGHT", engine));
        block->addRule(new fl::MamdaniRule("if PlaneDist is CLOSE and BearingAngle is LOWPOS then ChangeInHeading is VERYLEFT", engine));
        block->addRule(new fl::MamdaniRule("if PlaneDist is CLOSE and BearingAngle is POS then ChangeInHeading is LEFT", engine));
        block->addRule(new fl::MamdaniRule("if PlaneDist is CLOSE and BearingAngle is VERYPOS then ChangeInHeading is LEFT", engine));    
        
        block->addRule(new fl::MamdaniRule("if PlaneDist is FAR and BearingAngle is VERYNEG then ChangeInHeading is NOCHANGE", engine));
        block->addRule(new fl::MamdaniRule("if PlaneDist is FAR and BearingAngle is NEG then ChangeInHeading is RIGHT", engine));
        block->addRule(new fl::MamdaniRule("if PlaneDist is FAR and BearingAngle is LOWNEG then ChangeInHeading is RIGHT", engine));
        block->addRule(new fl::MamdaniRule("if PlaneDist is FAR and BearingAngle is LOWPOS then ChangeInHeading is LEFT", engine));
        block->addRule(new fl::MamdaniRule("if PlaneDist is FAR and BearingAngle is POS then ChangeInHeading is LEFT", engine));
        block->addRule(new fl::MamdaniRule("if PlaneDist is FAR and BearingAngle is VERYPOS then ChangeInHeading is NOCHANGE", engine));
        
        block->addRule(new fl::MamdaniRule("if PlaneDist is VERYFAR and BearingAngle is VERYNEG then ChangeInHeading is NOCHANGE", engine));
        block->addRule(new fl::MamdaniRule("if PlaneDist is VERYFAR and BearingAngle is NEG then ChangeInHeading is NOCHANGE", engine));
        block->addRule(new fl::MamdaniRule("if PlaneDist is VERYFAR and BearingAngle is LOWNEG then ChangeInHeading is RIGHT", engine));
        block->addRule(new fl::MamdaniRule("if PlaneDist is VERYFAR and BearingAngle is LOWPOS then ChangeInHeading is LEFT", engine));
        block->addRule(new fl::MamdaniRule("if PlaneDist is VERYFAR and BearingAngle is POS then ChangeInHeading is NOCHANGE", engine));
        block->addRule(new fl::MamdaniRule("if PlaneDist is VERYFAR and BearingAngle is VERYPOS then ChangeInHeading is NOCHANGE", engine));
        engine.addRuleBlock(block); 
        
        for (fl::flScalar in2 = -100.0; in2 < 100.0; in2 += 15.0){
            for (fl::flScalar in1 = 0.0; in1 < 80.0; in1 += 10.0){
                distanceBetweenPlanes->setInput(in1);
                bearingAngle->setInput(in2);
                engine.process();
                fl::flScalar out = changeHeading->output().defuzzify();
                (void)out; //Just to avoid warning when building
                std::stringstream ss;
                ss << "Dist BTWN Planes = " << in1 << "  Fuzzified= " << distanceBetweenPlanes->fuzzify(in1);
                std::string s1(ss.str());
                ROS_INFO(s1.c_str()); 
                ss.str(std::string());//clear sstream
                ss << "Bearing Angle = " << in2 << "  Fuzzified= " << bearingAngle->fuzzify(in2);
                std::string s2(ss.str());
                ROS_INFO(s2.c_str());
                ss.str(std::string());//clear sstream
                ss << "Output = " << out << "  Fuzzified= " << changeHeading->fuzzify(out);
                std::string s3(ss.str());
                ROS_INFO(s3.c_str()); 
                
            }
        }
    }

    
    void Test::main(int args, char** argv) {
        
        ROS_INFO("Hey look at me! Look look looooooooook!");
        ROS_INFO("======================================");
        sleep(2);
        //AU_UAV_ROS::FuzzyLogicController fl1;
        //fl::flScalar out = fl1.FuzzyLogicOne(15.0, 48.0);
        //std::stringstream ss;
        //ss << "Output= " << out;
        ////std::string s1(ss.str());
        //ROS_INFO(s1.c_str()); 
        FuzzyLogicTwo();	
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

