// Copyright (c) 2015, Biagio Cosenza.
// Technische Universitaet Berlin. All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.


#ifndef _SIMULATION_H_
#define _SIMULATION_H_

#include "stdafx.h"
#include "vector_types.h"
#include "CLHelper.h"
#include "logFile.h"
#include "SimParam.h"
#include "BoidModel.h"
#include "WorldBox.h"
#include "WorldGround.h"
#include "OverlayText.h"
#include "SkyBox.h"
#include "Column.h"
#include "Tunnel.h"

/*
	Boid simulation controler. Handles interaction between view and model.
*/
class Simulation
{
private:
	GLenum error;
	CLHelper* clHelper;
	BoidModel* boidModel;
	LogFile* logFile;
	simParams_t simParams;
	WorldBox* worldBox;
	WorldGround* worldGround;
	OverlayText* overlayText;
	Skybox* skybox;
	Column* column1;
	Column* column2;
	Column* column3;
	Tunnel* tunnel;

	//vector which holds all objects which are drawn 
	std::vector<Renderable*> renderList;

	//position data for boids
	std::vector<Vec4> pos;
	//velocity data for boids
	std::vector<Vec4> vel;
	//goal position which is used in some models
	std::vector<Vec4> goal;
	//color of boids which is used in some models
	std::vector<Vec4> color;

	//string for complete simulation step time
	std::string simTimeAll;

	bool initOk;
	//timestamp when simulation step was done 
	unsigned long long timeLast;
	//time difference between two simulation steps
	float timeDiff;
	//index of current active boid model
	int currentModel;
	//model to switch to on the first simulation step (set via command-line arg)
	int pendingModel;
	//index of initial placement of boids
	int currentInitPlacement;

	//name of the current scene, shown in the overlay
	const char* currentSceneName;

	//create position and velocity data for boids dependend on currentInitPlacement
	void createData(std::vector<Vec4> *pos, std::vector<Vec4> *vel, std::vector<Vec4> *goal, std::vector<Vec4> *color);
	//restart the simulation
	void restart(int modelNum);
	//set simParams and restart for a given model number
	void switchToModel(int modelNum);
	//instantiate the boid model for a scene, including its obstacle data
	BoidModel* createModel(int modelNum);
	//build the combined obstacle buffers for the three columns
	void columnObstacleData(std::vector<Vec4>* cor, std::vector<unsigned int>* start, std::vector<unsigned int>* end, std::vector<Vec4>* posObst);
	//create random float between minimum mn and maximum mx
	float randFloat(float mn, float mx);
	//sample a position inside a spherical cluster of radius r cells,
	//centered at the given fractions of the world extent
	Vec4 clusterPos(float fx, float fy, float fz, float r);

	Simulation();
	~Simulation();

public:
	//initialize simulation
	void init();
	//start simulation, optionally with a specific model number
	void start(int initialModel = BOID_SIMPLE);
	//do a simulation step
	void simulationStep();
	//return complete simulation time
	long getBoidModelSimulationTime();
	//return number of boids used in the model
	int getBoidModelNumberOfBoids();
	//vector with all strings to be displayed in the overlay
	std::vector<const char*> getSimTimeDescriptions();
	//return time difference between to simulation steps
	float getTimeDiff();
	//handle key press
	void keyPress(unsigned char key);
	//name of the current scene, e.g. "3: SH"
	const char* getSceneName() const { return currentSceneName; }
	//returns vector with all objects to be rendered
	std::vector<Renderable*> getRenderList();

	//get the velocity and position of a specific boid
	void getPosVelOfBoid(unsigned int* boidIndex, float* posX, float* posY, float* posZ, float* velX, float* velY, float* velZ);

	//instance of the simulation. Singleton pattern
	static Simulation *pInstance;

	static inline Simulation &getInstance() {
		if (NULL == pInstance) { pInstance = new Simulation(); }
		return *pInstance;
	}
};

#endif