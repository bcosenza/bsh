#include "common.h"
#include "Simulation.h"
#ifdef USE_SYCL
#include "BoidModelSimpleSYCL.h"   // SYCL: Simple (brute force) scene
#include "BoidGridBase.h"          // SYCL: Grid (uniform-grid acceleration) scene
#else
#include "BoidModelCL.h"           // concrete OpenCL models constructed in createModel()
#endif
#include "vectorTypes.h"
#include "gfx.h"
#include <math.h>

Simulation *Simulation::pInstance = NULL;

/*
	Static description of every scene: simulation weights, world dimensions,
	camera preset and which obstacles are part of it. switchToModel(), restart()
	and the overlay all read from this table, so adding a scene means adding
	one row here plus a case in createModel().
*/
struct SceneConfig {
	int id;
	const char* name;
	unsigned int numBoids;
	float wAlignment, wCohesion, wSeparation, wOwn, wPath;
	float maxVel, maxVelCor;
	unsigned int gridX, gridY, gridZ;
	int cameraPreset;
	bool flatGround;	// draw the flat ground used by the 2D worlds
	bool showColumns;
	bool showTunnel;
};

static const SceneConfig sceneTable[] = {
	{ BOID_SIMPLE, "1: Simple (brute force)", NUM_BOIDS_SIMPLE,
	  WEIGHT_ALIGNMENT, WEIGHT_COHESION, WEIGHT_SEPARATION, WEIGHT_OWN, 0.f,
	  MAX_VEL_SIMPLE, MAX_VEL_COR_SIMPLE,
	  GRID_SIZE_X, GRID_SIZE_Y, GRID_SIZE_Z,
	  CAMERA_PRESET_STANDARD, false, false, false },

	{ BOID_GRID, "2: Grid", NUM_BOIDS_GRID,
	  WEIGHT_ALIGNMENT_GRID, WEIGHT_COHESION_GRID, WEIGHT_SEPARATION_GRID, WEIGHT_OWN_GRID, 0.f,
	  MAX_VEL_GRID, MAX_VEL_COR_GRID,
	  GRID_SIZE_X, GRID_SIZE_Y, GRID_SIZE_Z,
	  CAMERA_PRESET_STANDARD, false, false, false },

	{ BOID_SH, "3: SH", NUM_BOIDS_SH,
	  WEIGHT_ALIGNMENT_SH, WEIGHT_COHESION_SH, WEIGHT_SEPARATION_SH, WEIGHT_OWN_SH, 0.f,
	  MAX_VEL_GRID, MAX_VEL_COR_GRID,
	  GRID_SIZE_X_SH, GRID_SIZE_Y_SH, GRID_SIZE_Z_SH,
	  CAMERA_PRESET_SH, false, false, false },

	{ BOID_GRID_2D, "4: Grid 2D", NUM_BOIDS_GRID_2D,
	  WEIGHT_ALIGNMENT_GRID_2D, WEIGHT_COHESION_GRID_2D, WEIGHT_SEPARATION_GRID_2D, WEIGHT_OWN_GRID_2D, 0.f,
	  MAX_VEL_GRID, MAX_VEL_COR_GRID,
	  GRID_SIZE_X_GRID_2D, GRID_SIZE_Y_GRID_2D, GRID_SIZE_Z_GRID_2D,
	  CAMERA_PRESET_2D_FAR, true, false, false },

	{ BOID_SH_2D, "5: SH 2D", NUM_BOIDS_SH_2D,
	  WEIGHT_ALIGNMENT_SH_2D, WEIGHT_COHESION_SH_2D, WEIGHT_SEPARATION_SH_2D, WEIGHT_OWN_SH_2D, 0.f,
	  MAX_VEL_GRID, MAX_VEL_COR_GRID,
	  GRID_SIZE_X_SH_2D, GRID_SIZE_Y_SH_2D, GRID_SIZE_Z_SH_2D,
	  CAMERA_PRESET_2D, true, false, false },

	{ BOID_SH_WAY1, "6: SH Way 1", NUM_BOIDS_SH,
	  WEIGHT_ALIGNMENT_SH_WAY1, WEIGHT_COHESION_SH_WAY1, WEIGHT_SEPARATION_SH_WAY1, WEIGHT_OWN_SH_WAY1, WEIGHT_GOAL_SH_WAY1,
	  MAX_VEL_GRID, MAX_VEL_COR_GRID,
	  GRID_SIZE_X_SH_WAY1, GRID_SIZE_Y_SH_WAY1, GRID_SIZE_Z_SH_WAY1,
	  CAMERA_PRESET_SH_SMALL, false, false, false },

	{ BOID_SH_WAY2, "7: SH Way 2", NUM_BOIDS_SH,
	  WEIGHT_ALIGNMENT_SH_WAY1, WEIGHT_COHESION_SH_WAY1, WEIGHT_SEPARATION_SH_WAY1, WEIGHT_OWN_SH_WAY1, WEIGHT_GOAL_SH_WAY1,
	  MAX_VEL_GRID, MAX_VEL_COR_GRID,
	  GRID_SIZE_X_SH_WAY1, GRID_SIZE_Y_SH_WAY1, GRID_SIZE_Z_SH_WAY1,
	  CAMERA_PRESET_SH_SMALL, false, false, false },

	{ BOID_SH_OBSTACLE, "8: SH Obstacle", NUM_BOIDS_SH,
	  WEIGHT_ALIGNMENT_SH_OBSTACLE, WEIGHT_COHESION_SH_OBSTACLE, WEIGHT_SEPARATION_SH_OBSTACLE, WEIGHT_OWN_SH_OBSTACLE, WEIGHT_GOAL_SH_OBSTACLE,
	  MAX_VEL_GRID, MAX_VEL_COR_GRID,
	  GRID_SIZE_X_SH_OBSTACLE, GRID_SIZE_Y_SH_OBSTACLE, GRID_SIZE_Z_SH_OBSTACLE,
	  CAMERA_PRESET_SH_SMALL, false, true, false },

	{ BOID_SH_OBSTACLE_COMBINED, "9: SH Obstacle + Way", NUM_BOIDS_SH,
	  WEIGHT_ALIGNMENT_SH_OBSTACLE, WEIGHT_COHESION_SH_OBSTACLE, WEIGHT_SEPARATION_SH_OBSTACLE, WEIGHT_OWN_SH_OBSTACLE, WEIGHT_GOAL_SH_OBSTACLE,
	  MAX_VEL_GRID, MAX_VEL_COR_GRID,
	  GRID_SIZE_X_SH_OBSTACLE, GRID_SIZE_Y_SH_OBSTACLE, GRID_SIZE_Z_SH_OBSTACLE,
	  CAMERA_PRESET_SH_SMALL, false, true, false },

	{ BOID_SH_OBSTACLE_TUNNEL, "0: SH Tunnel", NUM_BOIDS_SH,
	  WEIGHT_ALIGNMENT_SH_OBSTACLE, WEIGHT_COHESION_SH_OBSTACLE, WEIGHT_SEPARATION_SH_OBSTACLE, WEIGHT_OWN_SH_OBSTACLE, WEIGHT_GOAL_SH_OBSTACLE,
	  MAX_VEL_GRID, MAX_VEL_COR_GRID,
	  GRID_SIZE_X_SH_OBSTACLE, GRID_SIZE_Y_SH_OBSTACLE, GRID_SIZE_Z_SH_OBSTACLE,
	  CAMERA_PRESET_SH_SMALL, false, false, true },
};

static const SceneConfig* findScene(int id){
	for (size_t i = 0; i < sizeof(sceneTable) / sizeof(sceneTable[0]); i++)
		if (sceneTable[i].id == id)
			return &sceneTable[i];
	return NULL;
}

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
#ifndef USE_SYCL
	clHelper = new CLHelper(logFile);
#endif

	currentModel = BOID_SIMPLE;
	pendingModel = BOID_SIMPLE;
	currentSceneName = findScene(BOID_SIMPLE)->name;
#ifdef USE_SYCL
	boidModel = new BoidModelSimpleSYCL(pos, vel, &simParams);
#else
	boidModel = new BoidModelSimple(clHelper, pos, vel, &simParams);
#endif
	
	worldBox = new WorldBox(simParams.gridSize.x, TRUE, simParams.gridSize.x, simParams.gridSize.y, simParams.gridSize.z);
	worldGround = new WorldGround(FALSE, simParams.gridSize.x, simParams.gridSize.y, simParams.gridSize.z);
	overlayText = new OverlayText();
	skybox = new Skybox("", "textures/posx.tga", "textures/negx.tga", "textures/negz.tga", "textures/posz.tga", "textures/negy.tga", "textures/posy.tga");

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
	const SceneConfig* scene = findScene(modelNum);
	if (scene == NULL)
		return;

	delete boidModel;
	delete worldBox;
	delete worldGround;

	pos.resize(simParams.numBodies);
	vel.resize(simParams.numBodies);
	goal.resize(simParams.numBodies);
	color.resize(simParams.numBodies);
	createData(&pos, &vel, &goal, &color);

	column1->setVisibility(scene->showColumns);
	column2->setVisibility(scene->showColumns);
	column3->setVisibility(scene->showColumns);
	tunnel->setVisibility(scene->showTunnel);

	boidModel = createModel(modelNum);
	worldGround = new WorldGround(scene->flatGround, simParams.gridSize.x, simParams.gridSize.y, simParams.gridSize.z);
	worldBox = new WorldBox(simParams.gridSize.x, TRUE, simParams.gridSize.x, simParams.gridSize.y, simParams.gridSize.z);

	renderList[3] = worldGround;
	renderList[1] = boidModel;
	renderList[0] = worldBox;
}

void Simulation::columnObstacleData(std::vector<Vec4>* cor, std::vector<unsigned int>* start, std::vector<unsigned int>* end, std::vector<Vec4>* posObst){
	unsigned int segments = column1->obstacleSegmentCount();
	cor->resize(3 * column1->obstacleCornerCount());
	start->resize(3 * segments);
	end->resize(3 * segments);
	posObst->resize(3 * segments);
	column1->getObstacleForce(cor, start, end, posObst, 0);
	column2->getObstacleForce(cor, start, end, posObst, segments);
	column3->getObstacleForce(cor, start, end, posObst, 2 * segments);
}

BoidModel* Simulation::createModel(int modelNum){
#ifdef USE_SYCL
	// SYCL build: Simple (brute force) and Grid (uniform-grid acceleration)
	switch (modelNum){
	case BOID_GRID:	return new BoidGridBase(pos, vel, &simParams);
	default:		return new BoidModelSimpleSYCL(pos, vel, &simParams);
	}
#else
	switch (modelNum){
	case BOID_SIMPLE:	return new BoidModelSimple(clHelper, pos, vel, &simParams);
	case BOID_GRID:		return new BoidModelGrid(clHelper, pos, vel, &simParams);
	case BOID_SH:		return new BoidModelSH(clHelper, pos, vel, &simParams);
	case BOID_GRID_2D:	return new BoidModelGrid_2D(clHelper, pos, vel, &simParams);
	case BOID_SH_2D:	return new BoidModelSH_2D(clHelper, pos, vel, &simParams);
	case BOID_SH_WAY1:	return new BoidModelSHWay1(clHelper, pos, vel, goal, color, &simParams);
	case BOID_SH_WAY2:	return new BoidModelSHWay2(clHelper, pos, vel, goal, color, &simParams);
	case BOID_SH_OBSTACLE: {
		std::vector<Vec4> cor, posObst; std::vector<unsigned int> start, end;
		columnObstacleData(&cor, &start, &end, &posObst);
		return new BoidModelSHObstacle(clHelper, pos, vel, goal, &simParams, cor, start, end, posObst);
	}
	case BOID_SH_OBSTACLE_COMBINED: {
		std::vector<Vec4> cor, posObst; std::vector<unsigned int> start, end;
		columnObstacleData(&cor, &start, &end, &posObst);
		return new BoidModelSHCombined(clHelper, pos, vel, goal, color, &simParams, cor, start, end, posObst);
	}
	case BOID_SH_OBSTACLE_TUNNEL: {
		std::vector<Vec4> cor(tunnel->obstacleCornerCount()), posObst(tunnel->obstacleSegmentCount());
		std::vector<unsigned int> start(tunnel->obstacleSegmentCount()), end(tunnel->obstacleSegmentCount());
		tunnel->getObstacleForce(&cor, &start, &end, &posObst, 0);
		return new BoidModelSHObstacleTunnel(clHelper, pos, vel, goal, color, &simParams, cor, start, end, posObst);
	}
	}
	return NULL;
#endif
}


void Simulation::start(int initialModel){
	if (initOk){
		pendingModel = initialModel;
		timeLast = GetTickCount64();
		GFX::getInstance().startRendering();
	}
}

void Simulation::simulationStep(){
	if (pendingModel != currentModel){
		switchToModel(pendingModel);
		return;
	}
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

void Simulation::switchToModel(int modelNum){
#ifdef USE_SYCL
	// the SYCL build ports the Simple and Grid models
	if (modelNum != BOID_SIMPLE && modelNum != BOID_GRID)
		return;
#endif
	const SceneConfig* scene = findScene(modelNum);
	if (scene == NULL)
		return;

	currentModel = modelNum;
	pendingModel = modelNum;	// keep in sync or simulationStep() switches back
	currentSceneName = scene->name;

	simParams.numBodies = scene->numBoids;
	simParams.wAlignment = scene->wAlignment;
	simParams.wCohesion = scene->wCohesion;
	simParams.wSeparation = scene->wSeparation;
	simParams.wOwn = scene->wOwn;
	simParams.wPath = scene->wPath;
	simParams.maxVel = scene->maxVel;
	simParams.maxVelCor = scene->maxVelCor;
	simParams.gridSize = make_uint3(scene->gridX, scene->gridY, scene->gridZ);
	simParams.numCells = scene->gridX * scene->gridY * scene->gridZ;

	restart(currentModel);
	GFX::getInstance().setCam(scene->cameraPreset);
}

void Simulation::keyPress(unsigned char key){
	switch (key)
	{
	case '1':	switchToModel(BOID_SIMPLE);              break;
	case '2':	switchToModel(BOID_GRID);                break;
	case '3':	switchToModel(BOID_SH);                  break;
	case '4':	switchToModel(BOID_GRID_2D);             break;
	case '5':	switchToModel(BOID_SH_2D);               break;
	case '6':	switchToModel(BOID_SH_WAY1);             break;
	case '7':	switchToModel(BOID_SH_WAY2);             break;
	case '8':	switchToModel(BOID_SH_OBSTACLE);         break;
	case '9':	switchToModel(BOID_SH_OBSTACLE_COMBINED);break;
	case '0':	switchToModel(BOID_SH_OBSTACLE_TUNNEL);  break;
	case 'R':	//restart model
	case 'r': 
		restart(currentModel);
		break;
	case '+':	//increase number of boids
		simParams.numBodies = simParams.numBodies * 2;
		if (simParams.numBodies >= NUM_BOIDS_MAX)
			simParams.numBodies = NUM_BOIDS_MAX;
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

//boid group colors used by the test placements
static const Vec4 COLOR_GREEN(0.17f, 0.37f, 0.21f, 1.f);
static const Vec4 COLOR_RED(0.69f, 0.12f, 0.12f, 1.f);
static const Vec4 COLOR_YELLOW(.77f, 0.59f, 0.09f, 1.f);
static const Vec4 COLOR_BLUE(0.09f, 0.59f, .77f, 1.f);

//turn a position into a goal entry (w = 0)
static inline Vec4 goalAt(const Vec4& p){
	return Vec4(p.x, p.y, p.z, 0.0f);
}

Vec4 Simulation::clusterPos(float fx, float fy, float fz, float r){
	float theta = randFloat(0.0f, CL_M_PI);
	float phi = randFloat(0.0f, 2 * CL_M_PI);
	float x = CELL_SIZE_X * simParams.gridSize.x * fx + randFloat(0.0f, r * CELL_SIZE_X) * sin(theta) * cos(phi);
	float y = CELL_SIZE_Y * simParams.gridSize.y * fy + randFloat(0.0f, r * CELL_SIZE_Y) * sin(theta) * sin(phi);
	float z = CELL_SIZE_Z * simParams.gridSize.z * fz + randFloat(0.0f, r * CELL_SIZE_Z) * cos(theta);
	return Vec4(x, y, z, 1.f);
}

void Simulation::createData(std::vector<Vec4> *pos, std::vector<Vec4> *vel, std::vector<Vec4> *goal, std::vector<Vec4> *color){
	const float r = TEST_SETUP_RADIUS / 2;
	const float maxVel = simParams.maxVel;

	switch(currentInitPlacement){
	case 0:		//random placement inside the cube, goal = own start position
		for (int i = 0; i < (int)simParams.numBodies; i++)
		{
			float x = randFloat(2.f * CELL_SIZE_X, simParams.gridSize.x * CELL_SIZE_X - CELL_SIZE_X * 2.f);
			float z = randFloat(2.f * CELL_SIZE_Z, simParams.gridSize.z * CELL_SIZE_Z - CELL_SIZE_Z * 2.f);
			float y = randFloat(2.f * CELL_SIZE_Y, simParams.gridSize.y * CELL_SIZE_Y - CELL_SIZE_Y * 2.f);
			(*pos)[i] = Vec4(x, y, z, 1.f);
			(*vel)[i] = Vec4(randFloat(-maxVel, maxVel), randFloat(-maxVel, maxVel), randFloat(-maxVel, maxVel), 0.f);
			(*goal)[i] = Vec4(x, y, z, 0.0f);
			(*color)[i] = BOID_COLOR;
		}
		break;
	case 1:		//two groups moving towards each other, goals swapped pairwise
		for (int i = 0; i < (int)simParams.numBodies; i += 2)
		{
			(*pos)[i] = clusterPos(.5f, .5f, .25f, r);
			(*vel)[i] = Vec4(0.0f, 0.0f, randFloat(0.0f, maxVel), 0.0f);
			(*color)[i] = COLOR_GREEN;

			(*pos)[i + 1] = clusterPos(.5f, .5f, .75f, r);
			(*vel)[i + 1] = Vec4(0.0f, 0.0f, randFloat(-maxVel, 0.0f), 0.0f);
			(*color)[i + 1] = COLOR_RED;

			(*goal)[i] = goalAt((*pos)[i + 1]);
			(*goal)[i + 1] = goalAt((*pos)[i]);
		}
		break;
	case 2:		//small group and large group moving towards each other's center
	{
		Vec4 goalFar = goalAt(clusterPos(.5f, .5f, .75f, 0.f));
		Vec4 goalNear = goalAt(clusterPos(.5f, .5f, .25f, 0.f));

		for (int j = 0; j < (int)simParams.numBodies; j++){
			bool small_ = j < (int)simParams.numBodies / 8;
			(*pos)[j] = small_ ? clusterPos(.5f, .5f, .25f, TEST_SETUP_RADIUS / 4)
			                   : clusterPos(.5f, .5f, .75f, TEST_SETUP_RADIUS);
			(*vel)[j] = small_ ? Vec4(0.0f, 0.0f, randFloat(0.0f, maxVel), 0.0f)
			                   : Vec4(0.0f, 0.0f, randFloat(-maxVel, 0.0f), 0.0f);
			(*goal)[j] = small_ ? goalFar : goalNear;
			(*color)[j] = small_ ? COLOR_GREEN : COLOR_RED;
		}
		break;
	}
	case 3:		//four groups converging at the center, goals swapped pairwise
		for (int i = 0; i < (int)simParams.numBodies; i += 4)
		{
			(*pos)[i] = clusterPos(.5f, .5f, .25f, r);
			(*vel)[i] = Vec4(0.0f, 0.0f, randFloat(0.0f, maxVel), 0.0f);
			(*color)[i] = COLOR_RED;

			(*pos)[i + 1] = clusterPos(.5f, .5f, .75f, r);
			(*vel)[i + 1] = Vec4(0.0f, 0.0f, randFloat(-maxVel, 0.0f), 0.0f);
			(*color)[i + 1] = COLOR_GREEN;

			(*pos)[i + 2] = clusterPos(.25f, .5f, .5f, r);
			(*vel)[i + 2] = Vec4(randFloat(0.0f, maxVel), 0.0f, 0.0f, 0.0f);
			(*color)[i + 2] = COLOR_YELLOW;

			(*pos)[i + 3] = clusterPos(.75f, .5f, .5f, r);
			(*vel)[i + 3] = Vec4(randFloat(-maxVel, 0.0f), 0.0f, 0.0f, 0.0f);
			(*color)[i + 3] = COLOR_BLUE;

			(*goal)[i] = goalAt((*pos)[i + 1]);
			(*goal)[i + 1] = goalAt((*pos)[i]);
			(*goal)[i + 2] = goalAt((*pos)[i + 3]);
			(*goal)[i + 3] = goalAt((*pos)[i + 2]);
		}
		break;
	case 4:		//two groups crossing at the center
		for (int i = 0; i < (int)simParams.numBodies; i += 2)
		{
			(*pos)[i] = clusterPos(.5f, .5f, .25f, r);
			(*vel)[i] = Vec4(0.0f, 0.0f, randFloat(0.0f, maxVel), 0.0f);
			(*goal)[i] = goalAt(clusterPos(.5f, .5f, .75f, r));
			(*color)[i] = COLOR_GREEN;

			(*pos)[i + 1] = clusterPos(.75f, .5f, .5f, r);
			(*vel)[i + 1] = Vec4(randFloat(-maxVel, 0.0f), 0.0f, 0.0f, 0.0f);
			(*goal)[i + 1] = goalAt(clusterPos(.25f, .5f, .5f, r));
			(*color)[i + 1] = COLOR_RED;
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