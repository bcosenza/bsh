#include "stdafx.h"
#include "simulation.h"
#include "vectorTypes.h"
#include "gfx.h"
#include <math.h>

Simulation *Simulation::pInstance = NULL;

Simulation::Simulation() {
	srand(time(NULL));
	initOk = GFX::getInstance().initOpenGL();

	simParams.cellSize = make_float3(CELL_SIZE_X, CELL_SIZE_Y, CELL_SIZE_Z);
	simParams.gridSize = make_uint3(GRID_SIZE_X, GRID_SIZE_Y, GRID_SIZE_Z);
	simParams.worldOrigin = make_float3(WORLD_ORIGIN_X, WORLD_ORIGIN_Y, WORLD_ORIGIN_Z);
	simParams.numBodies = NUM_BOIDS;
	simParams.numCells = GRID_SIZE_X * GRID_SIZE_Y * GRID_SIZE_Z;
	simParams.wAlignment = WEIGHT_ALIGNMENT;
	simParams.wCohesion = WEIGHT_COHESION;
	simParams.wSeparation = WEIGHT_SEPARATION;
	simParams.wOwn = WEIGHT_OWN;
	simParams.localSize = LOCAL_SIZE_VEC4;
	simParams.maxVel = MAX_VEL_SIMPLE;
	simParams.maxVelCor = MAX_VEL_COR_SIMPLE;

	pos.resize(simParams.numBodies);
	vel.resize(simParams.numBodies);
	goal.resize(simParams.numBodies);
	color.resize(simParams.numBodies);

	currentInitPlacement = MODEL_INIT_PLACEMENT;
	createData(&pos, &vel, &goal, &color);

	logFile = new LogFile("OCL Boid ");
	clHelper = new CLHelper(logFile);

	currentModel = BOID_SIMPLE;
	boidModel = new BoidModelSimple(clHelper, pos, vel, &simParams);
	
	worldBox = new WorldBox(simParams.gridSize.x, TRUE, simParams.gridSize.x, simParams.gridSize.y, simParams.gridSize.z);
	worldGround = new WorldGround(FALSE, simParams.gridSize.x, simParams.gridSize.y, simParams.gridSize.z);
	overlayText = new OverlayText();
	skybox = new Skybox("", "textures/posx.tga", "textures/negx.tga", "textures/negz.tga", "textures/posz.tga", "textures/posy.tga", "textures/negy.tga");

	column1 = new Column(false, simParams.cellSize.x, simParams.cellSize.y, simParams.cellSize.z, 8, 0, 6, 10);
	column2 = new Column(false, simParams.cellSize.x, simParams.cellSize.y, simParams.cellSize.z, 8, 0, 9, 10);
	column3 = new Column(false, simParams.cellSize.x, simParams.cellSize.y, simParams.cellSize.z, 6, 0, 7, 10);

	tunnel = new Tunnel(false, simParams.cellSize.x, simParams.cellSize.y, simParams.cellSize.z, 3, 3, 5, 5);

	renderList = std::vector<Renderable*>(9);
	renderList[0] = worldBox;
	renderList[1] = boidModel;
	renderList[2] = overlayText;
	renderList[3] = worldGround;
	renderList[4] = skybox;
	renderList[5] = column1;
	renderList[6] = column2;
	renderList[7] = column3;
	renderList[8] = tunnel; 
}

Simulation::~Simulation() {}

void Simulation::init() {
	simParams.cellSize = make_float3(CELL_SIZE_X, CELL_SIZE_Y, CELL_SIZE_Z);
	simParams.gridSize = make_uint3(GRID_SIZE_X, GRID_SIZE_Y, GRID_SIZE_Z);
	simParams.worldOrigin = make_float3(WORLD_ORIGIN_X, WORLD_ORIGIN_Y, WORLD_ORIGIN_Z);
	simParams.numBodies = NUM_BOIDS;
	simParams.numCells = GRID_SIZE_X * GRID_SIZE_Y * GRID_SIZE_Z;
	simParams.wAlignment = WEIGHT_ALIGNMENT;
	simParams.wCohesion = WEIGHT_COHESION;
	simParams.wSeparation = WEIGHT_SEPARATION;

//	std::vector<Vec4> pos(simParams.numBodies);
//	std::vector<Vec4> vel(simParams.numBodies);


//	createData(&pos, &vel);
	
//	logFile = new LogFile("OCL Boid ");
//	clHelper = new CLHelper(logFile);
//	boidModel = new BoidModelGrid(clHelper, pos, vel, &simParams);
}

void Simulation::restart(int modelNum){
	delete boidModel;
	delete worldBox;
	delete worldGround;
	std::vector<Vec4> cor(3 *(10 * 12 + 2 * 5)); std::vector<unsigned int> start(3 * 42); std::vector<unsigned int> end(3 * 42); std::vector<Vec4> posObst(3 * 42);
	std::vector<Vec4> cor2(406); std::vector<unsigned int> start2(208); std::vector<unsigned int> end2(208); std::vector<Vec4> posObst2(208);
	
	switch (modelNum){
		case BOID_SIMPLE:
			pos.resize(simParams.numBodies);
			vel.resize(simParams.numBodies);
			goal.resize(simParams.numBodies);
			color.resize(simParams.numBodies);
			createData(&pos, &vel, &goal, &color);

			column1->setVisibility(false);
			column2->setVisibility(false);
			column3->setVisibility(false);
			tunnel->setVisibility(false);

			boidModel = new BoidModelSimple(clHelper, pos, vel, &simParams);
			worldGround = new WorldGround(FALSE, simParams.gridSize.x, simParams.gridSize.y, simParams.gridSize.z);
			break;

		case BOID_GRID:
			pos.resize(simParams.numBodies);
			vel.resize(simParams.numBodies);
			goal.resize(simParams.numBodies);
			color.resize(simParams.numBodies);
			createData(&pos, &vel, &goal, &color);

			column1->setVisibility(false);
			column2->setVisibility(false);
			column3->setVisibility(false);
			tunnel->setVisibility(false);

			boidModel = new BoidModelGrid(clHelper, pos, vel, &simParams);
			worldGround = new WorldGround(FALSE, simParams.gridSize.x, simParams.gridSize.y, simParams.gridSize.z);
			break;

		case BOID_SH:
			pos.resize(simParams.numBodies);
			vel.resize(simParams.numBodies);
			goal.resize(simParams.numBodies);
			color.resize(simParams.numBodies);
			createData(&pos, &vel, &goal, &color);

			column1->setVisibility(false);
			column2->setVisibility(false);
			column3->setVisibility(false);
			tunnel->setVisibility(false);

			boidModel = new BoidModelSH(clHelper, pos, vel, &simParams);
			worldGround = new WorldGround(FALSE, simParams.gridSize.x, simParams.gridSize.y, simParams.gridSize.z);
			break;

		case BOID_GRID_2D:
			pos.resize(simParams.numBodies);
			vel.resize(simParams.numBodies);
			goal.resize(simParams.numBodies);
			color.resize(simParams.numBodies);
			createData(&pos, &vel, &goal, &color);

			column1->setVisibility(false);
			column2->setVisibility(false);
			column3->setVisibility(false);
			tunnel->setVisibility(false);

			boidModel = new BoidModelGrid_2D(clHelper, pos, vel, &simParams);
			worldGround = new WorldGround(TRUE, simParams.gridSize.x, simParams.gridSize.y, simParams.gridSize.z);
			break;

		case BOID_SH_2D:
			pos.resize(simParams.numBodies);
			vel.resize(simParams.numBodies);
			goal.resize(simParams.numBodies);
			color.resize(simParams.numBodies);
			createData(&pos, &vel, &goal, &color);

			column1->setVisibility(false);
			column2->setVisibility(false);
			column3->setVisibility(false);
			tunnel->setVisibility(false);

			boidModel = new BoidModelSH_2D(clHelper, pos, vel, &simParams);
			worldGround = new WorldGround(TRUE, simParams.gridSize.x, simParams.gridSize.y, simParams.gridSize.z);
			break;
		case BOID_SH_WAY1:
			pos.resize(simParams.numBodies);
			vel.resize(simParams.numBodies);
			goal.resize(simParams.numBodies);
			color.resize(simParams.numBodies);
			createData(&pos, &vel, &goal, &color);

			column1->setVisibility(false);
			column2->setVisibility(false);
			column3->setVisibility(false);
			tunnel->setVisibility(false);

			boidModel = new BoidModelSHWay1(clHelper, pos, vel, goal, color, &simParams);
			worldGround = new WorldGround(FALSE, simParams.gridSize.x, simParams.gridSize.y, simParams.gridSize.z);
			break;
		case BOID_SH_WAY2:
			pos.resize(simParams.numBodies);
			vel.resize(simParams.numBodies);
			goal.resize(simParams.numBodies);
			color.resize(simParams.numBodies);
			createData(&pos, &vel, &goal, &color);

			column1->setVisibility(false);
			column2->setVisibility(false);
			column3->setVisibility(false);
			tunnel->setVisibility(false);

			boidModel = new BoidModelSHWay2(clHelper, pos, vel, goal, color, &simParams);
			worldGround = new WorldGround(FALSE, simParams.gridSize.x, simParams.gridSize.y, simParams.gridSize.z);
			break;
		case BOID_SH_OBSTACLE:
			pos.resize(simParams.numBodies);
			vel.resize(simParams.numBodies);
			goal.resize(simParams.numBodies);
			color.resize(simParams.numBodies);
			createData(&pos, &vel, &goal, &color);

			column1->setVisibility(true);
			column2->setVisibility(true);
			column3->setVisibility(true);
			tunnel->setVisibility(false);

			column1->getObstacleForce(&cor, &start, &end, &posObst, 0);
			column2->getObstacleForce(&cor, &start, &end, &posObst, 42);
			column3->getObstacleForce(&cor, &start, &end, &posObst, 84);
			boidModel = new BoidModelSHObstacle(clHelper, pos, vel, goal, &simParams, cor, start, end, posObst);
			worldGround = new WorldGround(FALSE, simParams.gridSize.x, simParams.gridSize.y, simParams.gridSize.z);
			break;
		case BOID_SH_OBSTACLE_COMBINED:
			pos.resize(simParams.numBodies);
			vel.resize(simParams.numBodies);
			goal.resize(simParams.numBodies);
			color.resize(simParams.numBodies);
			createData(&pos, &vel, &goal, &color);

			column1->setVisibility(true);
			column2->setVisibility(true);
			column3->setVisibility(true);
			tunnel->setVisibility(false);

			column1->getObstacleForce(&cor, &start, &end, &posObst, 0);
			column2->getObstacleForce(&cor, &start, &end, &posObst, 42);
			column3->getObstacleForce(&cor, &start, &end, &posObst, 84);
			boidModel = new BoidModelSHCombined(clHelper, pos, vel, goal, color, &simParams, cor, start, end, posObst);
			worldGround = new WorldGround(FALSE, simParams.gridSize.x, simParams.gridSize.y, simParams.gridSize.z);
			break;
		case BOID_SH_OBSTACLE_TUNNEL:
			pos.resize(simParams.numBodies);
			vel.resize(simParams.numBodies);
			goal.resize(simParams.numBodies);
			color.resize(simParams.numBodies);
			createData(&pos, &vel, &goal, &color);

			column1->setVisibility(false);
			column2->setVisibility(false);
			column3->setVisibility(false);
			tunnel->setVisibility(true);

			tunnel->getObstacleForce(&cor2, &start2, &end2, &posObst2, 0);
			boidModel = new BoidModelSHObstacleTunnel(clHelper, pos, vel, goal, color, &simParams, cor2, start2, end2, posObst2);
			worldGround = new WorldGround(FALSE, simParams.gridSize.x, simParams.gridSize.y, simParams.gridSize.z);
			break;
	}

	worldBox = new WorldBox(simParams.gridSize.x, TRUE, simParams.gridSize.x, simParams.gridSize.y, simParams.gridSize.z);
	renderList[3] = worldGround;
	renderList[1] = boidModel;
	renderList[0] = worldBox;
}


void Simulation::start(){
	if (initOk){
		timeLast = GetTickCount64();
		GFX::getInstance().startRendering();
	}
}

void Simulation::simulationStep(){
	timeDiff = getTimeDiff();
	boidModel->simulate(timeDiff);
}

float Simulation::getTimeDiff(){
	unsigned long long timeNow = GetTickCount64();
	float timeReturn = ((float)(timeNow - timeLast) / 1000.f);
	timeLast = timeNow;

	if (timeReturn > 1.f)
		return 1.f;
	else
		return timeReturn;
}

void Simulation::keyPress(unsigned char key){
	switch (key)
	{
	case '1':	
		currentModel = BOID_SIMPLE;

		simParams.numBodies = NUM_BOIDS_SIMPLE;
		simParams.wAlignment = WEIGHT_ALIGNMENT;
		simParams.wCohesion = WEIGHT_COHESION;
		simParams.wSeparation = WEIGHT_SEPARATION;
		simParams.wOwn = WEIGHT_OWN;
		simParams.maxVel = MAX_VEL_SIMPLE;
		simParams.maxVelCor = MAX_VEL_COR_SIMPLE;
		simParams.gridSize = make_uint3(GRID_SIZE_X, GRID_SIZE_Y, GRID_SIZE_Z);
		simParams.numCells = GRID_SIZE_X * GRID_SIZE_Y * GRID_SIZE_Z;

		restart(currentModel);
		GFX::getInstance().setCam(CAMERA_PRESET_STANDARD);
		break;
	case '2':	
		currentModel = BOID_GRID;
		
		simParams.numBodies = NUM_BOIDS_GRID;
		simParams.wAlignment = WEIGHT_ALIGNMENT_GRID;
		simParams.wCohesion = WEIGHT_COHESION_GRID;
		simParams.wSeparation = WEIGHT_SEPARATION_GRID;
		simParams.wOwn = WEIGHT_OWN_GRID;
		simParams.maxVel = MAX_VEL_GRID;
		simParams.maxVelCor = MAX_VEL_COR_GRID;
		simParams.gridSize = make_uint3(GRID_SIZE_X, GRID_SIZE_Y, GRID_SIZE_Z);
		simParams.numCells = GRID_SIZE_X * GRID_SIZE_Y * GRID_SIZE_Z;

		restart(currentModel);
		GFX::getInstance().setCam(CAMERA_PRESET_STANDARD);
		break;
	case '3':	//switch model to SH
		currentModel = BOID_SH;

		simParams.numBodies = NUM_BOIDS_SIMPLE; //NUM_BOIDS_SH;
		simParams.wAlignment = WEIGHT_ALIGNMENT_SH;
		simParams.wCohesion = WEIGHT_COHESION_SH;
		simParams.wSeparation = WEIGHT_SEPARATION_SH;
		simParams.wOwn = WEIGHT_OWN_SH;
		simParams.maxVel = MAX_VEL_GRID;
		simParams.maxVelCor = MAX_VEL_COR_GRID;
		simParams.gridSize = make_uint3(GRID_SIZE_X_SH, GRID_SIZE_Y_SH, GRID_SIZE_Z_SH);
		simParams.numCells = GRID_SIZE_X_SH * GRID_SIZE_Y_SH * GRID_SIZE_Z_SH;
		
		restart(currentModel);
		GFX::getInstance().setCam(CAMERA_PRESET_SH);
		break;
	case '4':
		currentModel = BOID_GRID_2D;

		simParams.numBodies = NUM_BOIDS_GRID_2D; //NUM_BOIDS_SH;
		simParams.wAlignment = WEIGHT_ALIGNMENT_GRID_2D;
		simParams.wCohesion = WEIGHT_COHESION_GRID_2D;
		simParams.wSeparation = WEIGHT_SEPARATION_GRID_2D;
		simParams.wOwn = WEIGHT_OWN_GRID_2D;
		simParams.maxVel = MAX_VEL_GRID;
		simParams.maxVelCor = MAX_VEL_COR_GRID;
		simParams.gridSize = make_uint3(GRID_SIZE_X_GRID_2D, GRID_SIZE_Y_GRID_2D, GRID_SIZE_Z_GRID_2D);
		simParams.numCells = GRID_SIZE_X_GRID_2D * GRID_SIZE_Y_GRID_2D * GRID_SIZE_Z_GRID_2D;

		restart(currentModel);
		GFX::getInstance().setCam(CAMERA_PRESET_2D_FAR);
		break;
	case '5':
		currentModel = BOID_SH_2D;

		simParams.numBodies = NUM_BOIDS_SIMPLE; //NUM_BOIDS_SH;
		simParams.wAlignment = WEIGHT_ALIGNMENT_SH_2D;
		simParams.wCohesion = WEIGHT_COHESION_SH_2D;
		simParams.wSeparation = WEIGHT_SEPARATION_SH_2D;
		simParams.wOwn = WEIGHT_OWN_SH_2D;
		simParams.maxVel = MAX_VEL_GRID;
		simParams.maxVelCor = MAX_VEL_COR_GRID;
		simParams.gridSize = make_uint3(GRID_SIZE_X_SH_2D, GRID_SIZE_Y_SH_2D, GRID_SIZE_Z_SH_2D);
		simParams.numCells = GRID_SIZE_X_SH_2D * GRID_SIZE_Y_SH_2D * GRID_SIZE_Z_SH_2D;

		restart(currentModel);
		GFX::getInstance().setCam(CAMERA_PRESET_2D);
		break;
	case '6':
		currentModel = BOID_SH_WAY1;

		simParams.numBodies = NUM_BOIDS_SIMPLE; //NUM_BOIDS_SH;
		simParams.wAlignment = WEIGHT_ALIGNMENT_SH_WAY1;
		simParams.wCohesion = WEIGHT_COHESION_SH_WAY1;
		simParams.wSeparation = WEIGHT_SEPARATION_SH_WAY1;
		simParams.wOwn = WEIGHT_OWN_SH_WAY1;
		simParams.maxVel = MAX_VEL_GRID;
		simParams.maxVelCor = MAX_VEL_COR_GRID;
		simParams.gridSize = make_uint3(GRID_SIZE_X_SH_WAY1, GRID_SIZE_Y_SH_WAY1, GRID_SIZE_Z_SH_WAY1);
		simParams.numCells = GRID_SIZE_X_SH_WAY1 * GRID_SIZE_Y_SH_WAY1 * GRID_SIZE_Z_SH_WAY1;
		simParams.wPath = WEIGHT_GOAL_SH_WAY1;

		restart(currentModel);
		GFX::getInstance().setCam(CAMERA_PRESET_SH);
		break;
	case '7':
		currentModel = BOID_SH_WAY2;

		simParams.numBodies = NUM_BOIDS_SIMPLE; //NUM_BOIDS_SH;
		simParams.wAlignment = WEIGHT_ALIGNMENT_SH_WAY1;
		simParams.wCohesion = WEIGHT_COHESION_SH_WAY1;
		simParams.wSeparation = WEIGHT_SEPARATION_SH_WAY1;
		simParams.wOwn = WEIGHT_OWN_SH_WAY1;
		simParams.maxVel = MAX_VEL_GRID;
		simParams.maxVelCor = MAX_VEL_COR_GRID;
		simParams.gridSize = make_uint3(GRID_SIZE_X_SH_WAY1, GRID_SIZE_Y_SH_WAY1, GRID_SIZE_Z_SH_WAY1);
		simParams.numCells = GRID_SIZE_X_SH_WAY1 * GRID_SIZE_Y_SH_WAY1 * GRID_SIZE_Z_SH_WAY1;
		simParams.wPath = WEIGHT_GOAL_SH_WAY1;

		restart(currentModel);
		GFX::getInstance().setCam(CAMERA_PRESET_SH);
		break;
	case '8':
		currentModel = BOID_SH_OBSTACLE;

		simParams.numBodies = NUM_BOIDS_SIMPLE; //NUM_BOIDS_SH;
		simParams.wAlignment = WEIGHT_ALIGNMENT_SH_OBSTACLE;
		simParams.wCohesion = WEIGHT_COHESION_SH_OBSTACLE;
		simParams.wSeparation = WEIGHT_SEPARATION_SH_OBSTACLE;
		simParams.wOwn = WEIGHT_OWN_SH_OBSTACLE;
		simParams.maxVel = MAX_VEL_GRID;
		simParams.maxVelCor = MAX_VEL_COR_GRID;
		simParams.gridSize = make_uint3(GRID_SIZE_X_SH_OBSTACLE, GRID_SIZE_Y_SH_OBSTACLE, GRID_SIZE_Z_SH_OBSTACLE);
		simParams.numCells = GRID_SIZE_X_SH_OBSTACLE * GRID_SIZE_Y_SH_OBSTACLE * GRID_SIZE_Z_SH_OBSTACLE;
		simParams.wPath = WEIGHT_GOAL_SH_OBSTACLE;

		restart(currentModel);
		GFX::getInstance().setCam(CAMERA_PRESET_SH);
		break;
	case '9':
		currentModel = BOID_SH_OBSTACLE_COMBINED;

		simParams.numBodies = NUM_BOIDS_SIMPLE; //NUM_BOIDS_SH;
		simParams.wAlignment = WEIGHT_ALIGNMENT_SH_OBSTACLE;
		simParams.wCohesion = WEIGHT_COHESION_SH_OBSTACLE;
		simParams.wSeparation = WEIGHT_SEPARATION_SH_OBSTACLE;
		simParams.wOwn = WEIGHT_OWN_SH_OBSTACLE;
		simParams.maxVel = MAX_VEL_GRID;
		simParams.maxVelCor = MAX_VEL_COR_GRID;
		simParams.gridSize = make_uint3(GRID_SIZE_X_SH_OBSTACLE, GRID_SIZE_Y_SH_OBSTACLE, GRID_SIZE_Z_SH_OBSTACLE);
		simParams.numCells = GRID_SIZE_X_SH_OBSTACLE * GRID_SIZE_Y_SH_OBSTACLE * GRID_SIZE_Z_SH_OBSTACLE;
		simParams.wPath = WEIGHT_GOAL_SH_OBSTACLE;

		restart(currentModel);
		GFX::getInstance().setCam(CAMERA_PRESET_SH);
		break;
	case '0':
		currentModel = BOID_SH_OBSTACLE_TUNNEL;

		simParams.numBodies = NUM_BOIDS_SIMPLE; //NUM_BOIDS_SH;
		simParams.wAlignment = WEIGHT_ALIGNMENT_SH_OBSTACLE;
		simParams.wCohesion = WEIGHT_COHESION_SH_OBSTACLE;
		simParams.wSeparation = WEIGHT_SEPARATION_SH_OBSTACLE;
		simParams.wOwn = WEIGHT_OWN_SH_OBSTACLE;
		simParams.maxVel = MAX_VEL_GRID;
		simParams.maxVelCor = MAX_VEL_COR_GRID;
		simParams.gridSize = make_uint3(GRID_SIZE_X_SH_OBSTACLE, GRID_SIZE_Y_SH_OBSTACLE, GRID_SIZE_Z_SH_OBSTACLE);
		simParams.numCells = GRID_SIZE_X_SH_OBSTACLE * GRID_SIZE_Y_SH_OBSTACLE * GRID_SIZE_Z_SH_OBSTACLE;
		simParams.wPath = WEIGHT_GOAL_SH_OBSTACLE;

		restart(currentModel);
		GFX::getInstance().setCam(CAMERA_PRESET_SH);
		break;
	case 'R':	//restart model
	case 'r': 
		restart(currentModel);
		break;
	case '+':	//increase number of boids
		simParams.numBodies = simParams.numBodies * 2;
		restart(currentModel);
		break;
	case '-':	//decrease number of boids
		simParams.numBodies = simParams.numBodies / 2;
		if (simParams.numBodies <= NUM_BOIDS_MIN)
			simParams.numBodies = NUM_BOIDS_MIN;
		restart(currentModel);
		break;
	case 'V':
	case 'v':	//toggle visibility of world box
		worldBox->toggleVisibility();
		break;
	case 't':
	case 'T':	//switch to a different initial boid placement
		currentInitPlacement = (currentInitPlacement + 1) % MODEL_INIT_PLACEMENT_MAX;
		restart(currentModel);
		break;
	case 'g':
	case 'G':	//toggle visibility of the ground of the world
		worldGround->toggleVisibility();
		break;
	case 's':
	case 'S':
		skybox->toggleVisibility();
		break;
	case '\033': // escape quits
	case '\015': // Enter quits
	case 'Q': // Q quits
	case 'q': // q (or escape) 
		// Cleanup up and quit
		break;
	}
}

std::vector<Renderable*> Simulation::getRenderList(){
	return renderList;
}

void Simulation::createData(std::vector<Vec4> *pos, std::vector<Vec4> *vel, std::vector<Vec4> *goal, std::vector<Vec4> *color){
	Vec4 goalT; int j = 0;

	switch(currentInitPlacement){
	case 0:
		for (int i = 0; i < simParams.numBodies; i++)
		{
			float x = randFloat(2.f * CELL_SIZE_X, simParams.gridSize.x * CELL_SIZE_X - CELL_SIZE_X * 2.f);
			float z = randFloat(2.f * CELL_SIZE_Z, simParams.gridSize.z * CELL_SIZE_Z - CELL_SIZE_Z * 2.f);
			float y = randFloat(2.f * CELL_SIZE_Y, simParams.gridSize.y * CELL_SIZE_Y - CELL_SIZE_Y * 2.f);
			float w = 1.f;
			(*pos)[i] = Vec4(x, y, z, w);
			(*vel)[i] = Vec4(randFloat(-simParams.maxVel, simParams.maxVel), randFloat(-simParams.maxVel, simParams.maxVel), randFloat(-simParams.maxVel, simParams.maxVel), 0.f);
			(*goal)[i] = Vec4(x, y, z, 0.0f);
			(*color)[i] = BOID_COLOR;
		}
		break;
	case 1:
		for (int i = 0; i < simParams.numBodies; i += 2)
		{
			float theta = randFloat(0.0f, CL_M_PI);
			float phi = randFloat(0.0f, 2 * CL_M_PI);
			float r = TEST_SETUP_RADIUS/2;
			
			float x = CELL_SIZE_X * simParams.gridSize.x / 2 + randFloat(0.0f, r * CELL_SIZE_X) * sin(theta) * cos(phi);
			float y = CELL_SIZE_Y * simParams.gridSize.y / 2 + randFloat(0.0f, r * CELL_SIZE_Y) * sin(theta) * sin(phi);
			float z = CELL_SIZE_Z * simParams.gridSize.z * 1 / 4 + randFloat(0.0f, r * CELL_SIZE_Z) * cos(theta);
			float w = 1.f;
			(*pos)[i] = Vec4(x, y, z, w);
			(*vel)[i] = Vec4(0.0f, 0.0f, randFloat(0.0f, simParams.maxVel), 0.0f);
			(*goal)[i + 1] = Vec4(x, y, z, 0.0f);
			(*color)[i] = Vec4(0.17f, 0.37f, 0.21f, 1.f);

			theta = randFloat(0.0f, CL_M_PI);
			phi = randFloat(0.0f, 2 * CL_M_PI);

			x = CELL_SIZE_X * simParams.gridSize.x / 2 + randFloat(0.0f, r * CELL_SIZE_X) * sin(theta) * cos(phi);
			y = CELL_SIZE_Y * simParams.gridSize.y / 2 + randFloat(0.0f, r * CELL_SIZE_Y) * sin(theta) * sin(phi);
			z = CELL_SIZE_Z * simParams.gridSize.z * 3 / 4 + randFloat(0.0f, r * CELL_SIZE_Z) * cos(theta);
			w = 1.f;
			(*pos)[i + 1] = Vec4(x, y, z, w);
			(*vel)[i + 1] = Vec4(0.0f, 0.0f, randFloat(-simParams.maxVel, 0.0f), 0.0f);
			(*goal)[i] = Vec4(x, y, z, 0.0f);
			(*color)[i + 1] = Vec4(0.69f, 0.12f, 0.12f, 1.0f);
		}
		break;
	case 2:
		goalT = Vec4(CELL_SIZE_X * simParams.gridSize.x / 2, CELL_SIZE_Y * simParams.gridSize.y / 2, CELL_SIZE_Z * simParams.gridSize.z * 3 / 4, 0.0f);

		for (j; j < simParams.numBodies / 8; j++)
		{
			float theta = randFloat(0.0f, CL_M_PI);
			float phi = randFloat(0.0f, 2 * CL_M_PI);
			float r = TEST_SETUP_RADIUS / 4;

			float x = CELL_SIZE_X * simParams.gridSize.x / 2 + randFloat(0.0f, r * CELL_SIZE_X) * sin(theta) * cos(phi);
			float y = CELL_SIZE_Y * simParams.gridSize.y / 2 + randFloat(0.0f, r * CELL_SIZE_Y) * sin(theta) * sin(phi);
			float z = CELL_SIZE_Z * simParams.gridSize.z * 1 / 4 + randFloat(0.0f, r * CELL_SIZE_Z) * cos(theta);
			float w = 1.f;
			(*pos)[j] = Vec4(x, y, z, w);
			(*vel)[j] = Vec4(0.0f, 0.0f, randFloat(0.0f, simParams.maxVel), 0.0f);
			(*goal)[j] = Vec4(goalT.x, goalT.y, goalT.z, goalT.w);
			(*color)[j] = Vec4(0.17f, 0.37f, 0.21f, 1.f);
		}

		goalT = Vec4(CELL_SIZE_X * simParams.gridSize.x / 2, CELL_SIZE_Y * simParams.gridSize.y / 2, CELL_SIZE_Z * simParams.gridSize.z / 4, 0.0f);

		for (j; j < simParams.numBodies; j++){
			float theta = randFloat(0.0f, CL_M_PI);
			float phi = randFloat(0.0f, 2 * CL_M_PI);
			float r = TEST_SETUP_RADIUS;

			float x = CELL_SIZE_X * simParams.gridSize.x / 2 + randFloat(0.0f, r * CELL_SIZE_X) * sin(theta) * cos(phi);
			float y = CELL_SIZE_Y * simParams.gridSize.y / 2 + randFloat(0.0f, r * CELL_SIZE_Y) * sin(theta) * sin(phi);
			float z = CELL_SIZE_Z * simParams.gridSize.z * 3 / 4 + randFloat(0.0f, r * CELL_SIZE_Z) * cos(theta);
			float w = 1.f;
			(*pos)[j] = Vec4(x, y, z, w);
			(*vel)[j] = Vec4(0.0f, 0.0f, randFloat(-simParams.maxVel, 0.0f), 0.0f);
			(*goal)[j] = Vec4(goalT.x, goalT.y, goalT.z, goalT.w);
			(*color)[j] = Vec4(0.69f, 0.12f, 0.12f, 1.0f);
		}
		break;

	case 3:
		for (int i = 0; i < simParams.numBodies; i += 4)
		{
			float theta = randFloat(0.0f, CL_M_PI);
			float phi = randFloat(0.0f, 2 * CL_M_PI);
			float r = TEST_SETUP_RADIUS / 2;

			float x = CELL_SIZE_X * simParams.gridSize.x / 2 + randFloat(0.0f, r * CELL_SIZE_X) * sin(theta) * cos(phi);
			float y = CELL_SIZE_Y * simParams.gridSize.y / 2 + randFloat(0.0f, r * CELL_SIZE_Y) * sin(theta) * sin(phi);
			float z = CELL_SIZE_Z * simParams.gridSize.z * 1 / 4 + randFloat(0.0f, r * CELL_SIZE_Z) * cos(theta);
			float w = 1.f;
			(*pos)[i] = Vec4(x, y, z, w);
			(*vel)[i] = Vec4(0.0f, 0.0f, randFloat(0.0f, simParams.maxVel), 0.0f);
			(*goal)[i + 1] = Vec4(x, y, z, 0.0f);
			(*color)[i] = Vec4(0.69f, 0.12f, 0.12f, 1.0f);

			theta = randFloat(0.0f, CL_M_PI);
			phi = randFloat(0.0f, 2 * CL_M_PI);

			x = CELL_SIZE_X * simParams.gridSize.x / 2 + randFloat(0.0f, r * CELL_SIZE_X) * sin(theta) * cos(phi);
			y = CELL_SIZE_Y * simParams.gridSize.y / 2 + randFloat(0.0f, r * CELL_SIZE_Y) * sin(theta) * sin(phi);
			z = CELL_SIZE_Z * simParams.gridSize.z * 3 / 4 + randFloat(0.0f, r * CELL_SIZE_Z) * cos(theta);
			w = 1.f;
			(*pos)[i + 1] = Vec4(x, y, z, w);
			(*vel)[i + 1] = Vec4(0.0f, 0.0f, randFloat(-simParams.maxVel, 0.0f), 0.0f);
			(*goal)[i] = Vec4(x, y, z, 0.0f);
			(*color)[i + 1] = Vec4(0.17f, 0.37f, 0.21f, 1.f);

			theta = randFloat(0.0f, CL_M_PI);
			phi = randFloat(0.0f, 2 * CL_M_PI);

			x = CELL_SIZE_X * simParams.gridSize.x / 4 + randFloat(0.0f, r * CELL_SIZE_X) * sin(theta) * cos(phi);
			y = CELL_SIZE_Y * simParams.gridSize.y / 2 + randFloat(0.0f, r * CELL_SIZE_Y) * sin(theta) * sin(phi);
			z = CELL_SIZE_Z * simParams.gridSize.z / 2 + randFloat(0.0f, r * CELL_SIZE_Z) * cos(theta);
			w = 1.f;
			(*pos)[i + 2] = Vec4(x, y, z, w);
			(*vel)[i + 2] = Vec4(randFloat( 0.0f, simParams.maxVel), 0.0f, 0.0f, 0.0f);
			(*goal)[i + 3] = Vec4(x, y, z, 0.0f);
			(*color)[i + 2] = Vec4(.77f, 0.59f, 0.09f, 1.0f);
			theta = randFloat(0.0f, CL_M_PI);
			phi = randFloat(0.0f, 2 * CL_M_PI);

			x = CELL_SIZE_X * simParams.gridSize.x * 3/ 4 + randFloat(0.0f, r * CELL_SIZE_X) * sin(theta) * cos(phi);
			y = CELL_SIZE_Y * simParams.gridSize.y / 2 + randFloat(0.0f, r * CELL_SIZE_Y) * sin(theta) * sin(phi);
			z = CELL_SIZE_Z * simParams.gridSize.z / 2 + randFloat(0.0f, r * CELL_SIZE_Z) * cos(theta);
			w = 1.f;
			(*pos)[i + 3] = Vec4(x, y, z, w);
			(*vel)[i + 3] = Vec4(randFloat(-simParams.maxVel, 0.0f), 0.0f, 0.0f , 0.0f);
			(*goal)[i + 2] = Vec4(x, y, z, 0.0f);
			(*color)[i + 3] = Vec4(0.09f, 0.59f, .77f, 1.0f);
		}
		break;
	case 4:
		for (int i = 0; i < simParams.numBodies; i += 2)
		{
			float theta = randFloat(0.0f, CL_M_PI);
			float phi = randFloat(0.0f, 2 * CL_M_PI);
			float r = TEST_SETUP_RADIUS / 2;

			float x = CELL_SIZE_X * simParams.gridSize.x / 2 + randFloat(0.0f, r * CELL_SIZE_X) * sin(theta) * cos(phi);
			float y = CELL_SIZE_Y * simParams.gridSize.y / 2 + randFloat(0.0f, r * CELL_SIZE_Y) * sin(theta) * sin(phi);
			float z = CELL_SIZE_Z * simParams.gridSize.z * 1 / 4 + randFloat(0.0f, r * CELL_SIZE_Z) * cos(theta);
			float w = 1.f;
			(*pos)[i] = Vec4(x, y, z, w);
			(*vel)[i] = Vec4(0.0f, 0.0f, randFloat(0.0f, simParams.maxVel), 0.0f);
			(*color)[i] = Vec4(0.17f, 0.37f, 0.21f, 1.f);

			theta = randFloat(0.0f, CL_M_PI);
			phi = randFloat(0.0f, 2 * CL_M_PI);

			x = CELL_SIZE_X * simParams.gridSize.x / 2 + randFloat(0.0f, r * CELL_SIZE_X) * sin(theta) * cos(phi);
			y = CELL_SIZE_Y * simParams.gridSize.y / 2 + randFloat(0.0f, r * CELL_SIZE_Y) * sin(theta) * sin(phi);
			z = CELL_SIZE_Z * simParams.gridSize.z * 3 / 4 + randFloat(0.0f, r * CELL_SIZE_Z) * cos(theta);
			w = 1.f;

			(*goal)[i] = Vec4(x, y, z, 0.0f);

			theta = randFloat(0.0f, CL_M_PI);
			phi = randFloat(0.0f, 2 * CL_M_PI);

			x = CELL_SIZE_X * simParams.gridSize.x * 3 / 4 + randFloat(0.0f, r * CELL_SIZE_X) * sin(theta) * cos(phi);
			y = CELL_SIZE_Y * simParams.gridSize.y / 2 + randFloat(0.0f, r * CELL_SIZE_Y) * sin(theta) * sin(phi);
			z = CELL_SIZE_Z * simParams.gridSize.z / 2 + randFloat(0.0f, r * CELL_SIZE_Z) * cos(theta);
			w = 1.f;
			(*pos)[i + 1] = Vec4(x, y, z, w);
			(*vel)[i + 1] = Vec4(randFloat(-simParams.maxVel,0.0f), 0.0f, 0.0f, 0.0f);
			(*color)[i + 1] = Vec4(0.69f, 0.12f, 0.12f, 1.0f);

			theta = randFloat(0.0f, CL_M_PI);
			phi = randFloat(0.0f, 2 * CL_M_PI);

			x = CELL_SIZE_X * simParams.gridSize.x * 1 / 4 + randFloat(0.0f, r * CELL_SIZE_X) * sin(theta) * cos(phi);
			y = CELL_SIZE_Y * simParams.gridSize.y / 2 + randFloat(0.0f, r * CELL_SIZE_Y) * sin(theta) * sin(phi);
			z = CELL_SIZE_Z * simParams.gridSize.z / 2 + randFloat(0.0f, r * CELL_SIZE_Z) * cos(theta);

			(*goal)[i + 1] = Vec4(x, y, z, 0.0f);
		}
		break;
	}
}


float Simulation::randFloat(float mn, float mx)
{
	float r = std::rand() / (float)RAND_MAX;
	return mn + (mx - mn)*r;
}

long Simulation::getBoidModelSimulationTime(){
	return (long)(timeDiff * 1000);
	//return boidModel->getSimulationTime();
}

int Simulation::getBoidModelNumberOfBoids(){
	return simParams.numBodies;
}


std::vector<const char*> Simulation::getSimTimeDescriptions(){

	std::vector<const char*> text = boidModel->getSimTimeDescriptions();
	
	std::stringstream strstream;
	strstream.str(std::string());
	strstream << "Total time: " << timeDiff * 1000 << "ms";
	simTimeAll = strstream.str();
	text[3] = simTimeAll.c_str();

	return text;
}

void Simulation::getPosVelOfBoid(unsigned int* boidIndex, float* posX, float* posY, float* posZ, float* velX, float* velY, float* velZ){
	Vec4 pos, vel;
	boidModel->getFollowedBoid(boidIndex, &pos, &vel);

	*posX = pos.x;
	*posY = pos.y;
	*posZ = pos.z;

	*velX = vel.x;
	*velY = vel.y;
	*velZ = vel.z;
}