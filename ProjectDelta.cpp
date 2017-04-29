// ProjectDelta.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>
#include <assert.h>
#include <time.h>
#include <random>
#include <math.h>
#include <vector>
#include <algorithm>
#include "LY_NN.h"

using namespace std;

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
	T = 5.0;
	v = 3.0;
	u = 0;
}

void ship::simulate()
{
	// kinematics equations
	omega = omega + (u - omega)*dt / T;
	theta = theta + omega*dt;
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

	if ((m*xgoal+b)<ygoal2 && (m*xgoal+b)>ygoal1)
	{
		xpos = xnew;
		ypos = ynew;

		return true;
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
	while (abs(y2 - y1) > 10 || y1 == y2)
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
		weight = (double)rand() / RAND_MAX - (double)rand()/RAND_MAX;
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

	// Initialize Ship
	ship S;
	S.init();

	// Initialize Neural Network
	neural_network NN;
	NN.setup(3, 5, 1); //inputs are x, y, theta

	NN.set_in_min_max(0, 1000);
	NN.set_in_min_max(0, 1000);
	NN.set_in_min_max(0, 6.28);
	NN.set_out_min_max(-15, 15);

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

	for (int generation = 0; generation < 10; generation++) // number of generations
	{
		for (int policyIndex = 0; policyIndex < popSize; policyIndex++)
		{
			//S.init();

			NN.set_weights(population.at(policyIndex).weights, true);

			int tStep = 0;

			while (!S.checkForGoal(G.x1, G.y1, G.y2) && !S.checkForWall())
			{
				// set state
				vector<double> state;
				state.push_back(S.xpos);
				state.push_back(S.ypos);
				state.push_back(S.theta);

				// give NN state
				NN.set_vector_input(state);

				// execute and get output
				NN.execute();

				S.u = NN.get_output(0);

				// run one timestep
				S.simulate();
				
				population.at(policyIndex).timeStep = tStep;
				
				// determine fitness

				if (S.checkForGoal(G.x1, G.y1, G.y2))
				{
					cout << "SHIP REACHED GOAL" << endl;
					population.at(policyIndex).fitness = population.at(policyIndex).fitness +1000;
					break;
				}
				
				else if(S.checkForWall())
				{
					cout << "SHIP HIT WALL" << endl;
					population.at(policyIndex).fitness = population.at(policyIndex).fitness -1000;
					break;
				}

				else
				{
					population.at(policyIndex).fitness = population.at(policyIndex).fitness-1; //NEED SOME SORT OF FITNESS EVALUATION
				}

				cout << tStep << endl;

				if (tStep > 1000)
				{
					break;
				}

				tStep++;
			}
		}
		// EA for weights

		// Neural Network with weights

		// Simulate with given u

		// Evaluate fitness

		
		population = EA_downselect(population, popSize);

		population = EA_replicate(population, popSize, numWeights);

	}

	//sort(population.begin(), population.end());
	
	for (int i = 0; i < popSize; i++)
	{
		cout << i << "\t" << population.at(i).fitness << "\t" << population.at(i).timeStep << endl;
	}

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
			mutPol.weights.at(weightIndex) = mutPol.weights.at(weightIndex) + (double)rand() / RAND_MAX - (double)rand() / RAND_MAX;
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