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
	boidModel = new BoidModelSimpleSYCL(pos, vel, color, &simParams);
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
	case BOID_GRID:	return new BoidGridBase(pos, vel, color, &simParams);
	default:		return new BoidModelSimpleSYCL(pos, vel, color, &simParams);
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
static const Vec4 COLOR_ORANGE(0.85f, 0.42f, 0.09f, 1.f);
static const Vec4 COLOR_PURPLE(0.45f, 0.15f, 0.55f, 1.f);

//turn a position into a goal entry (w = 0)
static inline Vec4 goalAt(const Vec4& p){
	return Vec4(p.x, p.y, p.z, 0.0f);
}

//velocity of magnitude 'speed' pointing from 'from' toward 'to' (w = 0)
static inline Vec4 dirTo(const Vec4& from, const Vec4& to, float speed){
	float dx = to.x - from.x, dy = to.y - from.y, dz = to.z - from.z;
	float len = sqrtf(dx * dx + dy * dy + dz * dz);
	if (len < 1e-6f)
		return Vec4(0.f, 0.f, 0.f, 0.f);
	float s = speed / len;
	return Vec4(dx * s, dy * s, dz * s, 0.f);
}

Vec4 Simulation::clusterPos(float fx, float fy, float fz, float r){
	float theta = randFloat(0.0f, CL_M_PI);
	float phi = randFloat(0.0f, 2 * CL_M_PI);
	float x = CELL_SIZE_X * simParams.gridSize.x * fx + randFloat(0.0f, r * CELL_SIZE_X) * sin(theta) * cos(phi);
	float y = CELL_SIZE_Y * simParams.gridSize.y * fy + randFloat(0.0f, r * CELL_SIZE_Y) * sin(theta) * sin(phi);
	float z = CELL_SIZE_Z * simParams.gridSize.z * fz + randFloat(0.0f, r * CELL_SIZE_Z) * cos(theta);
	return Vec4(x, y, z, 1.f);
}

//every boid at a uniform-random position inside the world cube (inset two cells
//from each face) with a random velocity; goal = its own start position.
void Simulation::initRandom(std::vector<Vec4> *pos, std::vector<Vec4> *vel, std::vector<Vec4> *goal, std::vector<Vec4> *color){
	const float maxVel = simParams.maxVel;
	for (int i = 0; i < (int)simParams.numBodies; i++)
	{
		float x = randFloat(2.f * CELL_SIZE_X, simParams.gridSize.x * CELL_SIZE_X - CELL_SIZE_X * 2.f);
		float z = randFloat(2.f * CELL_SIZE_Z, simParams.gridSize.z * CELL_SIZE_Z - CELL_SIZE_Z * 2.f);
		float y = randFloat(2.f * CELL_SIZE_Y, simParams.gridSize.y * CELL_SIZE_Y - CELL_SIZE_Y * 2.f);
		(*pos)[i] = Vec4(x, y, z, 1.f);
		(*vel)[i] = Vec4(randFloat(-maxVel, maxVel), randFloat(-maxVel, maxVel), randFloat(-maxVel, maxVel), 0.f);
		(*goal)[i] = Vec4(x, y, z, 0.0f);
		(*color)[i] = COLOR_RED;
	}
}

//two spherical groups: the first half of the boids are spread in a sphere around
//P1, the rest around P2. P1 and P2 sit at 1/3 and 2/3 of the world along Z (X and
//Y centred). Every boid heads toward the opposite group's centre.
void Simulation::initTwoGroups(std::vector<Vec4> *pos, std::vector<Vec4> *vel, std::vector<Vec4> *goal, std::vector<Vec4> *color){
	const float maxVel = simParams.maxVel;
	const float r = TEST_SETUP_RADIUS / 2;   // sphere radius (in cells), as in the other test setups

	// group centres at 1/3 and 2/3 of the world extent along Z
	const Vec4 P1(CELL_SIZE_X * simParams.gridSize.x * 0.5f,
	              CELL_SIZE_Y * simParams.gridSize.y * 0.5f,
	              CELL_SIZE_Z * simParams.gridSize.z * (1.f / 3.f), 1.f);
	const Vec4 P2(CELL_SIZE_X * simParams.gridSize.x * 0.5f,
	              CELL_SIZE_Y * simParams.gridSize.y * 0.5f,
	              CELL_SIZE_Z * simParams.gridSize.z * (2.f / 3.f), 1.f);

	const int n = (int)simParams.numBodies;
	const int half = n / 2;
	for (int i = 0; i < n; i++)
	{
		const bool group1 = (i < half);
		// spread in a sphere around this group's centre (fz = 1/3 or 2/3)
		(*pos)[i]   = clusterPos(0.5f, 0.5f, group1 ? (1.f / 3.f) : (2.f / 3.f), r);
		// head toward the opposite group's centre
		const Vec4& target = group1 ? P2 : P1;
		(*vel)[i]   = dirTo((*pos)[i], target, maxVel);
		(*goal)[i]  = goalAt(target);
		(*color)[i] = group1 ? COLOR_GREEN : COLOR_RED;
	}
}

//display name for each initial placement (index == currentInitPlacement)
static const char* const kPlacementNames[MODEL_INIT_PLACEMENT_MAX] = {
	"one group",
	"two groups",
	"two groups asymmetric",
	"four groups",
	"three groups",
};

const char* Simulation::getPlacementName() const {
	if (currentInitPlacement >= 0 && currentInitPlacement < MODEL_INIT_PLACEMENT_MAX)
		return kPlacementNames[currentInitPlacement];
	return "";
}

void Simulation::createData(std::vector<Vec4> *pos, std::vector<Vec4> *vel, std::vector<Vec4> *goal, std::vector<Vec4> *color){
	const float r = TEST_SETUP_RADIUS / 2;
	const float maxVel = simParams.maxVel;

	switch(currentInitPlacement){
	case 0:		//random placement inside the cube, goal = own start position
		initRandom(pos, vel, goal, color);
		break;
	case 1:		//"two groups": head-on along Z, contiguous halves (goals swapped)
	{
		const int n = (int)simParams.numBodies;
		const int half = n / 2;
		for (int i = 0; i < half; i++)
		{
			const int j = i + half;
			(*pos)[i] = clusterPos(.5f, .5f, .25f, r);
			(*vel)[i] = Vec4(0.0f, 0.0f, randFloat(0.0f, maxVel), 0.0f);
			(*color)[i] = COLOR_RED;

			(*pos)[j] = clusterPos(.5f, .5f, .75f, r);
			(*vel)[j] = Vec4(0.0f, 0.0f, randFloat(-maxVel, 0.0f), 0.0f);
			(*color)[j] = COLOR_BLUE;

			(*goal)[i] = goalAt((*pos)[j]);
			(*goal)[j] = goalAt((*pos)[i]);
		}
		break;
	}
	case 2:		//"two groups asymmetric": small (first 1/8) vs large group, already contiguous
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
			(*color)[j] = small_ ? COLOR_RED : COLOR_BLUE;
		}
		break;
	}
	case 3:		//"four groups": converging at the centre, contiguous fourths (goals swapped)
	{
		const int n = (int)simParams.numBodies;
		const int q = n / 4;
		for (int i = 0; i < q; i++)
		{
			const int a = i, b = i + q, c = i + 2 * q, d = i + 3 * q;
			(*pos)[a] = clusterPos(.5f, .5f, .25f, r); (*vel)[a] = Vec4(0.0f, 0.0f, randFloat(0.0f, maxVel), 0.0f);  (*color)[a] = COLOR_RED;
			(*pos)[b] = clusterPos(.5f, .5f, .75f, r); (*vel)[b] = Vec4(0.0f, 0.0f, randFloat(-maxVel, 0.0f), 0.0f); (*color)[b] = COLOR_BLUE;
			(*pos)[c] = clusterPos(.25f, .5f, .5f, r); (*vel)[c] = Vec4(randFloat(0.0f, maxVel), 0.0f, 0.0f, 0.0f);  (*color)[c] = COLOR_ORANGE;
			(*pos)[d] = clusterPos(.75f, .5f, .5f, r); (*vel)[d] = Vec4(randFloat(-maxVel, 0.0f), 0.0f, 0.0f, 0.0f); (*color)[d] = COLOR_PURPLE;

			(*goal)[a] = goalAt((*pos)[b]); (*goal)[b] = goalAt((*pos)[a]);
			(*goal)[c] = goalAt((*pos)[d]); (*goal)[d] = goalAt((*pos)[c]);
		}
		break;
	}
	case 4:		//"three groups": converging at the centre, contiguous thirds
	{
		const int n = (int)simParams.numBodies;
		const int t = n / 3;
		for (int i = 0; i < t; i++)
		{
			const int a = i, b = i + t, c = i + 2 * t;
			(*pos)[a] = clusterPos(.5f, .5f, .25f, r); (*vel)[a] = Vec4(0.0f, 0.0f, randFloat(0.0f, maxVel), 0.0f);  (*color)[a] = COLOR_RED;
			(*pos)[b] = clusterPos(.5f, .5f, .75f, r); (*vel)[b] = Vec4(0.0f, 0.0f, randFloat(-maxVel, 0.0f), 0.0f); (*color)[b] = COLOR_BLUE;
			(*pos)[c] = clusterPos(.25f, .5f, .5f, r); (*vel)[c] = Vec4(randFloat(0.0f, maxVel), 0.0f, 0.0f, 0.0f);  (*color)[c] = COLOR_GREEN;

			(*goal)[a] = goalAt((*pos)[b]); (*goal)[b] = goalAt((*pos)[c]); (*goal)[c] = goalAt((*pos)[a]);
		}
		// numBodies may not divide evenly by 3; park any leftover boids in the first group
		for (int i = 3 * t; i < n; i++)
		{
			(*pos)[i] = clusterPos(.5f, .5f, .25f, r);
			(*vel)[i] = Vec4(0.0f, 0.0f, randFloat(0.0f, maxVel), 0.0f);
			(*goal)[i] = goalAt(clusterPos(.5f, .5f, .75f, 0.f));
			(*color)[i] = COLOR_RED;
		}
		break;
	}
	/* old placement 4 ("two groups crossing at the centre") -- disabled
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

	   old placement 5 ("two spherical groups, each aiming at the other's centre") -- disabled
	   initTwoGroups(pos, vel, goal, color);
	*/
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