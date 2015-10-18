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
#include "clHelper.h"
#include "logFile.h"
#include "simParam.h"
#include "boidModel.h"
#include "worldBox.h"
#include "worldGround.h"
#include "overlayText.h"
#include "skyBox.h"
#include "column.h"
#include "tunnel.h"

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
	//index of initial placement of boids
	int currentInitPlacement;

	//create position and velocity data for boids dependend on currentInitPlacement
	void createData(std::vector<Vec4> *pos, std::vector<Vec4> *vel, std::vector<Vec4> *goal, std::vector<Vec4> *color);
	//restart the simulation
	void restart(int modelNum);
	//create random float between minimum mn and maximum mx
	float randFloat(float mn, float mx);

	Simulation();
	~Simulation();

public:
	//initialize simulation
	void init();
	//start simulation
	void start();
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