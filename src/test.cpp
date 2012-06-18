/*   Copyright 2010 Juan Rada-Vilela

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
 */
#include "ros/ros.h"
#include "fuzzylite/test.h"
#include "fuzzylite/FuzzyLite.h"
#include "fuzzylite/OutputLVar.h"
#include "fuzzylite/InputLVar.h"
#include <limits>
#include <string>
#include <sstream>

#include "fuzzylite/FunctionTerm.h"
namespace fl {

	void Test::FuzzyLogicOne(){
        FuzzyOperator& op = FuzzyOperator::DefaultFuzzyOperator();
        FuzzyEngine engine("Collison-Detection", op);
	//don't know what these guys do: 
        engine.hedgeSet().add(new fl::HedgeNot);
        engine.hedgeSet().add(new fl::HedgeSomewhat);
        engine.hedgeSet().add(new fl::HedgeVery);

        fl::InputLVar* distanceToCollision = new fl::InputLVar("CollDist");
        distanceToCollision->addTerm(new fl::ShoulderTerm("VERYCLOSE", 12.0, 24.0, true));
        distanceToCollision->addTerm(new fl::TriangularTerm("CLOSE", 20.0, 48.0));
        distanceToCollision->addTerm(new fl::TriangularTerm("FAR", 42.0, 78.0));//42
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
	block->addRule(new fl::MamdaniRule("if CollDist is FAR and OverlapDistance is VERYNEG then CollisionImminence is very SAFE", engine));//s
        block->addRule(new fl::MamdaniRule("if CollDist is FAR and OverlapDistance is NEG then CollisionImminence is POSSIBLE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is FAR and OverlapDistance is ZERO then CollisionImminence is very POSSIBLE", engine));
	block->addRule(new fl::MamdaniRule("if CollDist is FAR and OverlapDistance is POS then CollisionImminence is POSSIBLE", engine));
 	block->addRule(new fl::MamdaniRule("if CollDist is FAR and OverlapDistance is VERYPOS then CollisionImminence is very SAFE", engine));
	block->addRule(new fl::MamdaniRule("if CollDist is VERYFAR and OverlapDistance is VERYNEG then CollisionImminence is very SAFE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is VERYFAR and OverlapDistance is NEG then CollisionImminence is somewhat POSSIBLE", engine));
        block->addRule(new fl::MamdaniRule("if CollDist is VERYFAR and OverlapDistance is ZERO then CollisionImminence is POSSIBLE", engine));
	block->addRule(new fl::MamdaniRule("if CollDist is VERYFAR and OverlapDistance is POS then CollisionImminence is somewhat POSSIBLE", engine));
	block->addRule(new fl::MamdaniRule("if CollDist is VERYFAR and OverlapDistance is VERYPOS then CollisionImminence is very SAFE", engine));//s

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

    void Test::SimpleTakagiSugeno() {
        FuzzyOperator& op = FuzzyOperator::DefaultFuzzyOperator();
        op.setDefuzzifier(new TakagiSugenoDefuzzifier);
        FuzzyEngine engine("takagi-sugeno", op);


        fl::InputLVar* x = new fl::InputLVar("x");
        x->addTerm(new fl::TriangularTerm("NEAR_1", 0, 2));
        x->addTerm(new fl::TriangularTerm("NEAR_2", 1, 3));
        x->addTerm(new fl::TriangularTerm("NEAR_3", 2, 4));
        x->addTerm(new fl::TriangularTerm("NEAR_4", 3, 5));
        x->addTerm(new fl::TriangularTerm("NEAR_5", 4, 6));
        x->addTerm(new fl::TriangularTerm("NEAR_6", 5, 7));
        x->addTerm(new fl::TriangularTerm("NEAR_7", 6, 8));
        x->addTerm(new fl::TriangularTerm("NEAR_8", 7, 9));
        x->addTerm(new fl::TriangularTerm("NEAR_9", 8, 10));
        engine.addInputLVar(x);

        fl::OutputLVar* f_x = new fl::OutputLVar("f_x");
        f_x->addTerm(new fl::FunctionTerm("function", "(sin x) / x", 0, 10));
        engine.addOutputLVar(f_x);

        fl::RuleBlock* block = new fl::RuleBlock();
        block->addRule(new fl::TakagiSugenoRule("if x is NEAR_1 then f_x=0.84", engine));
        block->addRule(new fl::TakagiSugenoRule("if x is NEAR_2 then f_x=0.45", engine));
        block->addRule(new fl::TakagiSugenoRule("if x is NEAR_3 then f_x=0.04", engine));
        block->addRule(new fl::TakagiSugenoRule("if x is NEAR_4 then f_x=-0.18", engine));
        block->addRule(new fl::TakagiSugenoRule("if x is NEAR_5 then f_x=-0.19", engine));
        block->addRule(new fl::TakagiSugenoRule("if x is NEAR_6 then f_x=-0.04", engine));
        block->addRule(new fl::TakagiSugenoRule("if x is NEAR_7 then f_x=0.09", engine));
        block->addRule(new fl::TakagiSugenoRule("if x is NEAR_8 then f_x=0.12", engine));
        block->addRule(new fl::TakagiSugenoRule("if x is NEAR_9 then f_x=0.04", engine));

        engine.addRuleBlock(block);

        int n = 40;
        flScalar mse = 0;
        for (fl::flScalar in = x->minimum(); in < x->maximum() ;
                in += (x->minimum() + x->maximum()) / n) {
            x->setInput(in);
            engine.process();
            flScalar expected = f_x->term(0)->membership(in);
            flScalar obtained = f_x->output().defuzzify();
            flScalar se = (expected - obtained) * (expected - obtained);
            mse += isnan(se) ? 0 : se;
            FL_LOG("x=" << in << "\texpected_out=" << expected << "\tobtained_out=" << obtained
                    << "\tse=" << se);
        }
        FL_LOG("MSE=" << mse / n);
    }

    void Test::SimplePendulum() {

        FuzzyOperator& op = FuzzyOperator::DefaultFuzzyOperator();
        FuzzyEngine engine("pendulum-3d",op);

        fl::InputLVar* anglex = new fl::InputLVar("AngleX");
        std::vector<std::string> labels;
        labels.push_back("NEAR_0");
        labels.push_back("NEAR_45");
        labels.push_back("NEAR_90");
        labels.push_back("NEAR_135");
        labels.push_back("NEAR_180");
        anglex->createTerms(5, LinguisticTerm::MF_SHOULDER, 0, 180, labels);
        engine.addInputLVar(anglex);

        fl::InputLVar* anglez = new fl::InputLVar("AngleZ");
        labels.clear();
        labels.push_back("NEAR_0");
        labels.push_back("NEAR_45");
        labels.push_back("NEAR_90");
        labels.push_back("NEAR_135");
        labels.push_back("NEAR_180");
        anglez->createTerms(5, LinguisticTerm::MF_SHOULDER, 0, 180, labels);
        engine.addInputLVar(anglez);

        fl::OutputLVar* forcex = new fl::OutputLVar("ForceX");
        //fl::OutputLVar* forcex = new fl::OutputLVar("ForceX");
        labels.clear();
        labels.push_back("NL");
        labels.push_back("NS");
        labels.push_back("ZR");
        labels.push_back("PS");
        labels.push_back("PL");
        forcex->createTerms(5, LinguisticTerm::MF_TRIANGULAR, -1, 1, labels);
        engine.addOutputLVar(forcex);

        fl::OutputLVar* forcez = new fl::OutputLVar("ForceZ");
        labels.clear();
        labels.push_back("NL");
        labels.push_back("NS");
        labels.push_back("ZR");
        labels.push_back("PS");
        labels.push_back("PL");
        forcez->createTerms(5, LinguisticTerm::MF_TRIANGULAR, -1, 1, labels);
        engine.addOutputLVar(forcez);

        fl::RuleBlock* ruleblock = new fl::RuleBlock("Rules");
        ruleblock->addRule(new fl::MamdaniRule("if AngleX is NEAR_180 then ForceX is NL", engine));
        ruleblock->addRule(new fl::MamdaniRule("if AngleX is NEAR_135 then ForceX is NS", engine));
        ruleblock->addRule(new fl::MamdaniRule("if AngleX is NEAR_90 then ForceX is ZR", engine));
        ruleblock->addRule(new fl::MamdaniRule("if AngleX is NEAR_45 then ForceX is PS", engine));
        ruleblock->addRule(new fl::MamdaniRule("if AngleX is NEAR_0 then ForceX is PL", engine));

        ruleblock->addRule(new fl::MamdaniRule("if AngleZ is NEAR_180 then ForceZ is NL", engine));
        ruleblock->addRule(new fl::MamdaniRule("if AngleZ is NEAR_135 then ForceZ is NS", engine));
        ruleblock->addRule(new fl::MamdaniRule("if AngleZ is NEAR_90 then ForceZ is ZR", engine));
        ruleblock->addRule(new fl::MamdaniRule("if AngleZ is NEAR_45 then ForceZ is PS", engine));
        ruleblock->addRule(new fl::MamdaniRule("if AngleZ is NEAR_0 then ForceZ is PL", engine));
        engine.addRuleBlock(ruleblock);

        FL_LOG(engine.toString());
        for (int i = 0; i < 180; i += 20) {
            engine.setInput("AngleX", i);
            engine.process();
            FL_LOG("angle=" << i << "\tforce=" << engine.output("ForceX"));
        } 
    }*/

    void Test::main(int args, char** argv) {

	ROS_INFO("starting fuzzyController1 in 2 seconds");
	ROS_INFO("======================================");
	sleep(2);
	FuzzyLogicOne();
	ROS_INFO("======================================");
/*    
	FL_LOG("Starting in 2 second");
    	FL_LOG("Example: Simple Mamdani");
    	FL_LOG("=======================");
    	sleep(2);
    	SimpleMamdani();
    	FL_LOG("=======================\n");

    	FL_LOG("Starting in 2 second");
    	FL_LOG("Example: Complex Mamdani");
    	FL_LOG("========================");
    	sleep(2);
        ComplexMamdani();
        FL_LOG("=======================\n");

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

        FL_LOG("For further examples build the GUI..."); */
    }

}

