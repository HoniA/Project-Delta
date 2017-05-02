// ProjectDelta.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>
#include <assert.h>
#include <random>
#include <time.h>
#include <vector>
#include <math.h>
#include <algorithm>
#include <fstream>
#include "LY_NN.h"

using namespace std;

#define LYRAND (double)rand()/RAND_MAX

class ship{
public:
	double xpos;
	double ypos;
	double xnew; // these are placeholders
	double ynew; //used to calculate equation of line ship is following
	double theta;
	double omega;
	double u;
	double v;
	double dt;
	double T;

	void init();
	void simulate();
	bool checkForWall();
	bool checkForGoal(double xgoal, double ygoal1, double ygoal2);
	
};

void ship::init()
{
	xpos = rand() % 1000;
	ypos = rand() % 1000;
	xnew = xpos;
	ynew = ypos;
	theta = rand() % 360 *3.1415/180; // in radians
	omega = rand() % 10; // in radians per second
	dt = 0.2;
	T = 10.0;
	v = 3.0;
	u = 0;
}

void ship::simulate()
{
	// kinematics equations
	omega = omega + (u - omega)*dt / T;
	theta = theta + omega*dt;
	if (theta > 2 * 3.1415)
	{
		theta = theta - 2 * 3.1415;
	}
	else if (theta < -2 * 3.1415)
	{
		theta = theta + 2 * 3.1415;
	}
	ynew = ypos + v*cos(theta)*dt;
	xnew = xpos + v*sin(theta)*dt;
	assert(xnew != xpos || ynew != ypos); // ensures ship is moving in at least one direction and that program can calculate next position (LR_8)
	
}

bool ship::checkForWall()
{
	if (xpos < 0 || xpos>1000 || ypos < 0 || ypos>1000)
	{
		return true;
	}

	return false;
}

bool ship::checkForGoal(double xgoal, double ygoal1, double ygoal2)
{
	// get equation of line that ship is following
	// check if line crosses goal
	double m;
	double b;
	m = (ynew - ypos) / (xnew - xpos);
	b = ypos - m*xpos;

	if (xpos > xgoal && xnew < xgoal)
	{
		if ((m*xgoal + b) < ygoal2 && (m*xgoal + b) > ygoal1)
		{
			xpos = xnew;
			ypos = ynew;

			return true;
		}
	}

	else if (xnew > xgoal && xpos < xgoal)
	{
		if ((m*xgoal + b) < ygoal2 && (m*xgoal + b) > ygoal1)
		{
			xpos = xnew;
			ypos = ynew;

			return true;
		}
	}

	// reset ship's current position 
	xpos = xnew;
	ypos = ynew;

	return false;
}

class goal {
public:
	double x1;
	double y1;
	double x2;
	double y2;

	void init();
};

void goal::init() {
	x1 = rand() % 1000;
	y1 = rand() % 1000;
	x2 = x1; //making sure goal is vertical for more easy calculations
	y2 = rand() % 1000;

	
	// Keep goal small (not necessary)
	while (abs(y2 - y1) > 200 || y1 == y2 || abs(y2-y1) < 100)
	{
		y2 = rand() % 1000;
	}

	//ensure y2 is always larger than y1
	if (y2 < y1)
	{
		double temp;
		temp = y1;
		y1 = y2;
		y2 = temp;
	}

	assert(x1 < 1000 && x2 < 1000 && x1>0 && x2>0); //test to ensure program can represent goal inside boundaries (LR_2)
	assert(y1 < 1000 && y2 < 1000 && y1>0 && y2>0);
}

class policy {
public:
	vector<double> weights;
	double fitness;
	int timeStep;

	void init(int numWeights);
};

void policy::init(int numWeights) {
	for (int i = 0; i < numWeights; i++)
	{
		double weight;
		weight = LYRAND - LYRAND;
		weights.push_back(weight); //placeholder
	}
	fitness = 0; // placeholder
	timeStep = 0;
}

vector<policy> EA_init(int popSize, int numWeights);

vector<policy> EA_replicate(vector<policy> P, int popSize, int numWeights);

vector<policy> EA_evaluate();

vector<policy> EA_downselect(vector<policy> P, int popSize);

int main()
{
	srand(time(NULL));
	ofstream fout;

	// Initialize Goal
	goal G;
	G.init();

	// Initialize Ship
	ship testShip;
	testShip.init();
	assert(testShip.xpos > 0 && testShip.xpos < 1000 && testShip.ypos>0 && testShip.ypos < 1000); // ensures that program can represent a ship (LR_!)

	int timeStep = 0;
	while (testShip.checkForGoal(G.x1, G.y1, G.y2) == false && testShip.checkForWall() == false)
	{
		testShip.simulate();

		timeStep++;
		
		if (testShip.checkForGoal(G.x1, G.y1, G.y2))
		{
			assert(testShip.ynew > G.y1 && testShip.ynew < G.y2); //ensures that ship has passed through goal (MR_2)
			cout << "Ship Reached Goal!" << endl;
		}

		else if (testShip.checkForWall())
		{
			assert(testShip.xpos > 1000 || testShip.xpos < 0 || testShip.ypos > 1000 || testShip.ypos < 0); // ensures that simulation ends when ship exits range (LR_3)
			cout << "Ship Exited Boundary!" << endl;
		}

		if (timeStep > 1000)
		{
			break;
		}
	}

	cout << "Steps Taken for Test 1: " << timeStep << endl;

	//re-initialize for second test 
	testShip.init();
	testShip.omega = 0;
	testShip.u = 0;
	testShip.theta = 0;

	timeStep = 0;
	double initialX = testShip.xpos;

	while (testShip.checkForWall() ==false)
	{
		testShip.simulate();
		
		if (testShip.checkForGoal(G.x1,G.y1,G.y2))
		{
			break;
		}

		timeStep++;

		assert(testShip.xpos == initialX); // ensures that ship moves in straight line when u = w = 0; (MR_1)
	}

	cout << "Test 2 Completed" << endl;

	//////// ACTUAL PROGRAM ////////////////////

	// Initialize Goal
	goal Gol;
	Gol.init();

	int userInput = 0;
	double shipInitX = -1;
	double shipInitY = -1;
	double shipInitTheta = -1;
	double shipInitOmega = -1;
	ship S;

	cout << "Choose Type of Run: " << endl;
	cout << "1. Place ship directly left of goal (HR_1)" << endl;
	cout << "2. Place ship in specific random position (HR_3)" << endl;
	cout << "3. Place ship in any random position (HR_4)" << endl;
	cin >> userInput;

	if (userInput == 1)
	{
		S.init();

		S.xpos = G.x1 - 50;
		S.ypos = (G.y1 + G.y2) / 2;
		S.theta = 3.1415 / 2;
	}

	else if (userInput == 2)
	{
		S.init();

		S.xpos = G.x1 - 100;
		S.ypos = (G.y1 + G.y2) / 2 - 100;
	}

	else if (userInput == 3)
	{
		// Initialize Ship
		S.init();
	}

	shipInitX = S.xpos;
	shipInitY = S.ypos;
	shipInitTheta = S.theta;
	shipInitOmega = S.omega;

	// Initialize Neural Network
	neural_network NN;
	NN.setup(3, 5, 1); //inputs are x, y, theta

	NN.set_in_min_max(0, 1000);
	NN.set_in_min_max(0, 1000);
	NN.set_in_min_max(0, 6.28);
	NN.set_out_min_max(-15*3.1415/180, 15*3.1415/180);

	int popSize = 10;

	int numWeights = NN.get_number_of_weights();

	vector<policy> population;
	for (int i = 0; i < popSize; i++)
	{
		policy P;
		P.init(numWeights);
		population.push_back(P);
	}
	assert(population.size() == popSize);

	fout.clear();
	fout.open("ShipPos.txt");
	fout << "GOAL:" << "\t" << G.x1 << "\t" << G.y1 << "\t" << G.x2 << "\t" << G.y2 << endl;

	cout << "SHIP: " << S.xpos << ", " << S.ypos << endl;
	cout << "GOAL: " << G.x1 << ", " << G.y1 << " & " << G.y2 << endl;

	for (int generation = 0; generation < 10; generation++) // number of generations
	{
		fout << "GENERATION: " << generation << endl;
		for (int policyIndex = 0; policyIndex < popSize; policyIndex++)
		{
			fout << "POLICY: " << policyIndex << endl;

			S.xpos = shipInitX;
			S.ypos = shipInitY;
			S.theta = shipInitTheta;
			S.omega = shipInitOmega;

			NN.set_weights(population.at(policyIndex).weights, true);

			int tStep = 0;

			while (tStep < 5000)
			{
				// set state
				vector<double> state;
				state.push_back(S.xpos);
				state.push_back(S.ypos);
				state.push_back(S.theta);
				assert(state.size() == 3); // Fulfills MR_3

				// give NN state
				NN.set_vector_input(state);

				// execute and get output
				NN.execute();

				S.u = NN.get_output(0);

				// run one timestep
				S.simulate();
				
				
				
				fout << tStep << "\t" << S.xpos << "\t" << S.ypos << "\t" << S.u << endl;


				// determine fitness

				if (S.checkForGoal(G.x1, G.y1, G.y2))
				{
					cout << "SHIP REACHED GOAL" << endl;
					cout << "Time: " << tStep << endl;
					population.at(policyIndex).fitness = 10000-tStep;
					break;
				} // Fulfills HR_3 & HR_4 (No assert used because cout proves it.) 
				
				
				else if(S.checkForWall())
				{
					cout << "SHIP HIT WALL" << endl;
					cout << "Time: " << tStep << endl;
					population.at(policyIndex).fitness = -tStep*100000000000000;
					break;
				}

				else
				{
					population.at(policyIndex).fitness = - tStep*sqrt(pow((S.xpos-G.x1),2) + pow((S.ypos-(G.y1+G.y2)/2),2)) - tStep*(abs(S.theta-3.1415/2-atan(abs((G.y1+G.y2)/2-S.ypos)/abs(G.x1-S.xpos)))); //NEED SOME SORT OF FITNESS EVALUATION
				} // Fulfills MR_4

				//cout << tStep << "\t" << population.at(policyIndex).fitness << endl;

				if (tStep == 4999)
				{
					cout << "TOOK TOO LONG" << endl;
					break;
				}

				tStep++;

				population.at(policyIndex).timeStep = tStep;
			}
			fout << "GENERATION: " << generation << endl;
			for (int j = 0; j < popSize; j++)
			{
				fout << "POLICY: " << j << "FITNESS: " << population.at(j).fitness << endl;
			}


		}
		
		population = EA_downselect(population, popSize);

		population = EA_replicate(population, popSize, numWeights);

	}

	//sort(population.begin(), population.end());
	
	for (int i = 0; i < popSize; i++)
	{
		cout << i << "\t" << population.at(i).fitness << "\t" << population.at(i).timeStep << endl;
	}

	fout.close();

	system("pause");
    return 0;
}

vector<policy> EA_init(int popSize, int numWeights)
{
	vector<policy> population;

	for (int i = 0; i < popSize / 2; i++)
	{
		policy p;
		p.init(numWeights);
		population.push_back(p);
	}
	assert(population.size() == popSize / 2); // fulfills test that a population of policies is created (MR.1)

	return population;
}

vector<policy> EA_replicate(vector<policy> P, int popSize, int numWeights)
{
	vector<policy> population;
	population = P;
	assert(population.size() == popSize / 2);

	while (population.size() != popSize)
	{
		int index = rand() % (popSize / 2);
		policy mutPol;
		mutPol = population.at(index);

		for (int i = 0; i < numWeights; i++)
		{
			int weightIndex = rand() % numWeights;
			mutPol.weights.at(weightIndex) = mutPol.weights.at(weightIndex) + LYRAND/10 - LYRAND/10;
			assert(mutPol.weights.at(weightIndex) != population.at(index).weights.at(weightIndex)); // ensures program can mutate set of weights (LR_5)
		}
		
		population.push_back(mutPol);
	}

	assert(population.size() == popSize);

	return population;
}

vector<policy> EA_downselect(vector<policy> P, int popSize)
{
	vector<policy> population;
	assert(population.size() == 0);

	// binary tournament
	// halving size of vector
	// trying to minimize fitness
	// compare two random spots in population, send lower fitness to downselected population
	while (population.size() != popSize / 2)
	{
		int index = rand() % (popSize);
		int index2 = rand() % (popSize);

		// assure that index2 is not the same as index
		while (index == index2)
		{
			index2 = rand() % popSize;
		}

		if (P.at(index).fitness > P.at(index2).fitness)
		{
			population.push_back(P.at(index));
		}
		else
		{
			population.push_back(P.at(index2));
		}
	}
	assert(population.size() == popSize / 2); //fulfills test that population can be downselected - size of vector halved (MR.4)

	return population;
}