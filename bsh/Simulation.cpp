#include "common.h"
#include "Simulation.h"
#ifdef USE_SYCL
#include "BoidModelSimpleSYCL.h"   // SYCL: Simple (brute force) scene
#include "BoidGridBase.h"          // SYCL: Grid (uniform-grid acceleration) scene
#include "BoidGroupAvoidance.h"    // SYCL: Two groups avoidance scene
#include "BoidGroupAvoidanceSH.h"  // SYCL: Two groups avoidance with SH scene
#include "Boid4GroupsAvoidanceSH.h" // SYCL: Four groups avoidance with SH scene
#include "BoidThreeColumnsSH.h"     // SYCL: Three columns avoidance with SH scene
#include "BoidTunnelSH.h"           // SYCL: Tunnel SH scene
#include "BoidCrossingLanesSH.h"    // SYCL: SH accuracy, crossing lanes scene
#include "BoidPlateLanesSH.h"       // SYCL: SH accuracy, plate lanes scene
#else
#include "BoidModelCL.h"           // concrete OpenCL models constructed in createModel()
#endif
#include "vectorTypes.h"
#include "gfx.h"
#include "AccuracyLanes.h"
#include <math.h>
#include <fstream>
#include <algorithm>
#include <cstdio>
#include <iostream>

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
	bool showPlates;	// thin plates of the accuracy scene 9 (false when omitted)
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

#ifdef USE_SYCL
	// SYCL: scene 3 is the grid-accelerated "Two groups avoidance" model, so it
	// uses the same world/tuning as the grid scene (not the SH 20^3 world).
	{ BOID_SH, "3: Two groups avoidance", NUM_BOIDS_SH,
	  WEIGHT_ALIGNMENT, WEIGHT_COHESION, WEIGHT_SEPARATION, WEIGHT_OWN, 0.f,
	  MAX_VEL_SIMPLE, MAX_VEL_COR_SIMPLE,
	  GRID_SIZE_X, GRID_SIZE_Y, GRID_SIZE_Z,
	  CAMERA_PRESET_STANDARD, false, false, false },
#else
	{ BOID_SH, "3: SH", NUM_BOIDS_SH,
	  WEIGHT_ALIGNMENT_SH, WEIGHT_COHESION_SH, WEIGHT_SEPARATION_SH, WEIGHT_OWN_SH, 0.f,
	  MAX_VEL_GRID, MAX_VEL_COR_GRID,
	  GRID_SIZE_X_SH, GRID_SIZE_Y_SH, GRID_SIZE_Z_SH,
	  CAMERA_PRESET_SH, false, false, false },
#endif

#ifdef USE_SYCL
	// SYCL: scene 4 is the grid-based "Two groups avoidance with SH" model; it uses
	// the same 3-D world/tuning as the grid scene (not the 2-D grid world).
	{ BOID_GRID_2D, "4: Two groups avoidance with SH", NUM_BOIDS_SH,
	  WEIGHT_ALIGNMENT, WEIGHT_COHESION, WEIGHT_SEPARATION, WEIGHT_OWN, 0.f,
	  MAX_VEL_SIMPLE, MAX_VEL_COR_SIMPLE,
	  GRID_SIZE_X, GRID_SIZE_Y, GRID_SIZE_Z,
	  CAMERA_PRESET_STANDARD, false, false, false },
#else
	{ BOID_GRID_2D, "4: Grid 2D", NUM_BOIDS_GRID_2D,
	  WEIGHT_ALIGNMENT_GRID_2D, WEIGHT_COHESION_GRID_2D, WEIGHT_SEPARATION_GRID_2D, WEIGHT_OWN_GRID_2D, 0.f,
	  MAX_VEL_GRID, MAX_VEL_COR_GRID,
	  GRID_SIZE_X_GRID_2D, GRID_SIZE_Y_GRID_2D, GRID_SIZE_Z_GRID_2D,
	  CAMERA_PRESET_2D_FAR, true, false, false },
#endif

#ifdef USE_SYCL
	// SYCL: scene 5 is the grid-based "Four groups avoidance with SH" model; same
	// 3-D world/tuning as the grid scene (not the SH-2D world).
	{ BOID_SH_2D, "5: Four groups avoidance with SH", NUM_BOIDS_SH,
	  WEIGHT_ALIGNMENT, WEIGHT_COHESION, WEIGHT_SEPARATION, WEIGHT_OWN, 0.f,
	  MAX_VEL_SIMPLE, MAX_VEL_COR_SIMPLE,
	  GRID_SIZE_X, GRID_SIZE_Y, GRID_SIZE_Z,
	  CAMERA_PRESET_STANDARD, false, false, false },
#else
	{ BOID_SH_2D, "5: SH 2D", NUM_BOIDS_SH_2D,
	  WEIGHT_ALIGNMENT_SH_2D, WEIGHT_COHESION_SH_2D, WEIGHT_SEPARATION_SH_2D, WEIGHT_OWN_SH_2D, 0.f,
	  MAX_VEL_GRID, MAX_VEL_COR_GRID,
	  GRID_SIZE_X_SH_2D, GRID_SIZE_Y_SH_2D, GRID_SIZE_Z_SH_2D,
	  CAMERA_PRESET_2D, true, false, false },
#endif

#ifdef USE_SYCL
	// SYCL: scene 6 is the grid-based "Three columns avoidance with SH" model. It
	// uses the small (16^3) world so the columns (built at cells 6-9) sit centrally,
	// and shows the columns.
	{ BOID_SH_WAY1, "6: Three columns avoidance with SH", NUM_BOIDS_SH,
	  WEIGHT_ALIGNMENT, WEIGHT_COHESION, WEIGHT_SEPARATION, WEIGHT_OWN, 0.f,
	  MAX_VEL_SIMPLE, MAX_VEL_COR_SIMPLE,
	  GRID_SIZE_X_SH_WAY1, GRID_SIZE_Y_SH_WAY1, GRID_SIZE_Z_SH_WAY1,
	  CAMERA_PRESET_SH_SMALL, false, true, false },
#else
	{ BOID_SH_WAY1, "6: SH Way 1", NUM_BOIDS_SH,
	  WEIGHT_ALIGNMENT_SH_WAY1, WEIGHT_COHESION_SH_WAY1, WEIGHT_SEPARATION_SH_WAY1, WEIGHT_OWN_SH_WAY1, WEIGHT_GOAL_SH_WAY1,
	  MAX_VEL_GRID, MAX_VEL_COR_GRID,
	  GRID_SIZE_X_SH_WAY1, GRID_SIZE_Y_SH_WAY1, GRID_SIZE_Z_SH_WAY1,
	  CAMERA_PRESET_SH_SMALL, false, false, false },
#endif

#ifdef USE_SYCL
	// SYCL: scene 7 is the grid-based "Tunnel SH" model -- boids funnel through a
	// wall with openings. Small (16^3) world so the tunnel (built at cell 3,3,5)
	// sits centrally, and shows the tunnel.
	{ BOID_SH_WAY2, "7: Tunnel SH", NUM_BOIDS_SH,
	  WEIGHT_ALIGNMENT, WEIGHT_COHESION, WEIGHT_SEPARATION, WEIGHT_OWN, 0.f,
	  MAX_VEL_SIMPLE, MAX_VEL_COR_SIMPLE,
	  GRID_SIZE_X_SH_WAY1, GRID_SIZE_Y_SH_WAY1, GRID_SIZE_Z_SH_WAY1,
	  CAMERA_PRESET_SH_SMALL, false, false, true },
#else
	{ BOID_SH_WAY2, "7: SH Way 2", NUM_BOIDS_SH,
	  WEIGHT_ALIGNMENT_SH_WAY1, WEIGHT_COHESION_SH_WAY1, WEIGHT_SEPARATION_SH_WAY1, WEIGHT_OWN_SH_WAY1, WEIGHT_GOAL_SH_WAY1,
	  MAX_VEL_GRID, MAX_VEL_COR_GRID,
	  GRID_SIZE_X_SH_WAY1, GRID_SIZE_Y_SH_WAY1, GRID_SIZE_Z_SH_WAY1,
	  CAMERA_PRESET_SH_SMALL, false, false, false },
#endif

#ifdef USE_SYCL
	// SYCL: scenes 8 and 9 are the SH accuracy scenes (lanes stacked along y in the
	// 1200^3 world, see AccuracyLanes.h): crossing groups for the group SH term, and
	// groups facing thin plates for the obstacle SH term.
	{ BOID_SH_OBSTACLE, "8: SH accuracy - crossing lanes", NUM_BOIDS_ACC_CROSSING,
	  WEIGHT_ALIGNMENT, WEIGHT_COHESION, WEIGHT_SEPARATION, WEIGHT_OWN, 0.f,
	  MAX_VEL_SIMPLE, MAX_VEL_COR_SIMPLE,
	  GRID_SIZE_X, GRID_SIZE_Y, GRID_SIZE_Z,
	  CAMERA_PRESET_STANDARD, false, false, false },

	{ BOID_SH_OBSTACLE_COMBINED, "9: SH accuracy - plate lanes", NUM_BOIDS_ACC_PLATES,
	  WEIGHT_ALIGNMENT, WEIGHT_COHESION, WEIGHT_SEPARATION, WEIGHT_OWN, 0.f,
	  MAX_VEL_SIMPLE, MAX_VEL_COR_SIMPLE,
	  GRID_SIZE_X, GRID_SIZE_Y, GRID_SIZE_Z,
	  CAMERA_PRESET_STANDARD, false, false, false, true },
#else
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
#endif

	{ BOID_SH_OBSTACLE_TUNNEL, "0: SH Tunnel", NUM_BOIDS_SH,
	  WEIGHT_ALIGNMENT_SH_OBSTACLE, WEIGHT_COHESION_SH_OBSTACLE, WEIGHT_SEPARATION_SH_OBSTACLE, WEIGHT_OWN_SH_OBSTACLE, WEIGHT_GOAL_SH_OBSTACLE,
	  MAX_VEL_GRID, MAX_VEL_COR_GRID,
	  GRID_SIZE_X_SH_OBSTACLE, GRID_SIZE_Y_SH_OBSTACLE, GRID_SIZE_Z_SH_OBSTACLE,
	  CAMERA_PRESET_SH_SMALL, false, false, true },
};

// Wireframe of the uniform acceleration grid for the current scene's world:
// GRID_ACCEL_CELL_SIZE cells (matching sycl/BoidGridBase) covering gridSize*cellSize.
static Grid* makeAccelGrid(const simParams_t& sp, bool visible){
	return new Grid(GRID_ACCEL_CELL_SIZE,
		(unsigned int)ceil((sp.gridSize.x * sp.cellSize.x) / GRID_ACCEL_CELL_SIZE),
		(unsigned int)ceil((sp.gridSize.y * sp.cellSize.y) / GRID_ACCEL_CELL_SIZE),
		(unsigned int)ceil((sp.gridSize.z * sp.cellSize.z) / GRID_ACCEL_CELL_SIZE),
		visible);
}

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

	// createData() consults the current scene and the tunnel, which are only set up
	// further down; give them safe values for this first call
	currentModel = BOID_SIMPLE;
	tunnel = NULL;

	currentInitPlacement = MODEL_INIT_PLACEMENT;
	createData(&pos, &vel, &goal, &color);

	logFile = new LogFile("OCL Boid ");
#ifndef USE_SYCL
	clHelper = new CLHelper(logFile);
#endif

	currentModel = BOID_SIMPLE;
	pendingModel = BOID_SIMPLE;
	currentSceneName = findScene(BOID_SIMPLE)->name;
	shMode = SH_PROJECTION_DEFAULT;
	vmfK = SH_VMF_KAPPA;
	timingSteps = 0;
	timingWarmup = 0;
	timingHaveFrame = false;
	accuracySteps = 0;
	accuracyCount = 0;
	runSeed = -1;
	headingSpread = 0.0f;
#ifdef USE_SYCL
	boidModel = new BoidModelSimpleSYCL(pos, vel, color, &simParams);
#else
	boidModel = new BoidModelSimple(clHelper, pos, vel, &simParams);
#endif
	
	worldBox = new WorldBox(simParams.gridSize.x, TRUE, simParams.gridSize.x, simParams.gridSize.y, simParams.gridSize.z);
	// wireframe of the acceleration grid, hidden by default; toggle with 'l'
	grid = makeAccelGrid(simParams, false);
	worldGround = new WorldGround(FALSE, simParams.gridSize.x, simParams.gridSize.y, simParams.gridSize.z);
	overlayText = new OverlayText();
	skybox = new Skybox("", "textures/posx.tga", "textures/negx.tga", "textures/negz.tga", "textures/posz.tga", "textures/negy.tga", "textures/posy.tga");

	column1 = new Column(false, simParams.cellSize.x, simParams.cellSize.y, simParams.cellSize.z, 8, 0, 6, 10);
	column2 = new Column(false, simParams.cellSize.x, simParams.cellSize.y, simParams.cellSize.z, 8, 0, 9, 10);
	column3 = new Column(false, simParams.cellSize.x, simParams.cellSize.y, simParams.cellSize.z, 6, 0, 7, 10);

	tunnel = new Tunnel(false, simParams.cellSize.x, simParams.cellSize.y, simParams.cellSize.z, 3, 3, 5, 5);
	// plates of accuracy scene 9, which uses the GRID_SIZE world
	plates = new PlateLanes(false, GRID_SIZE_X * CELL_SIZE_X, GRID_SIZE_Z * CELL_SIZE_Z);

	renderList = std::vector<Renderable*>(11);
	renderList[0] = worldBox;
	renderList[1] = boidModel;
	renderList[2] = overlayText;
	renderList[3] = worldGround;
	renderList[4] = skybox;
	renderList[5] = column1;
	renderList[6] = column2;
	renderList[7] = column3;
	renderList[8] = tunnel;
	renderList[9] = grid;
	renderList[10] = plates;
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
	plates->setVisibility(scene->showPlates);

	boidModel = createModel(modelNum);
	applySHProjection();
	if (accuracySteps > 0)
		boidModel->setAccuracyRecording(true);
	worldGround = new WorldGround(scene->flatGround, simParams.gridSize.x, simParams.gridSize.y, simParams.gridSize.z);
	worldBox = new WorldBox(simParams.gridSize.x, TRUE, simParams.gridSize.x, simParams.gridSize.y, simParams.gridSize.z);

	// the world size depends on the scene, so rebuild the grid overlay to match it
	// (keeping whether the user has it switched on)
	const bool gridVisible = grid->isVisible();
	delete grid;
	grid = makeAccelGrid(simParams, gridVisible);

	renderList[3] = worldGround;
	renderList[1] = boidModel;
	renderList[0] = worldBox;
	renderList[9] = grid;
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
	// SYCL build: Simple (brute force), Grid, and Two-groups-avoidance (grid-based)
	switch (modelNum){
	case BOID_GRID:		return new BoidGridBase(pos, vel, color, &simParams);
	case BOID_SH:		return new BoidGroupAvoidance(pos, vel, color, &simParams);
	case BOID_GRID_2D:	return new BoidGroupAvoidanceSH(pos, vel, color, &simParams);
	case BOID_SH_2D:	return new Boid4GroupsAvoidanceSH(pos, vel, color, &simParams);
	case BOID_SH_WAY1: {
		std::vector<Vec4> cor, posObst;
		std::vector<unsigned int> start, end;
		columnObstacleData(&cor, &start, &end, &posObst);
		return new BoidThreeColumnsSH(pos, vel, color, &simParams, cor, start, end, posObst);
	}
	case BOID_SH_WAY2: {
		std::vector<Vec4> cor(tunnel->obstacleCornerCount()), posObst(tunnel->obstacleSegmentCount());
		std::vector<unsigned int> start(tunnel->obstacleSegmentCount()), end(tunnel->obstacleSegmentCount());
		tunnel->getObstacleForce(&cor, &start, &end, &posObst, 0);
		return new BoidTunnelSH(pos, vel, color, &simParams, cor, start, end, posObst, goal);
	}
	case BOID_SH_OBSTACLE:	return new BoidCrossingLanesSH(pos, vel, color, &simParams, goal);
	case BOID_SH_OBSTACLE_COMBINED: {
		std::vector<Vec4> cor, posObst;
		std::vector<unsigned int> start, end;
		plates->getObstacleForce(&cor, &start, &end, &posObst);
		return new BoidPlateLanesSH(pos, vel, color, &simParams, cor, start, end, posObst,
		                            plates->pointsPerPlate(), plates->plateCentres(), plates->plateHalfExtent(), goal);
	}
	default:			return new BoidModelSimpleSYCL(pos, vel, color, &simParams);
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

void Simulation::start(const RunOptions& options){
	shMode = options.shMode;
	vmfK = options.vmfK;
	timingSteps = (options.timingSteps > 0) ? options.timingSteps : 0;
	timingWarmup = timingSteps ? TIMING_WARMUP_STEPS : 0;
	accuracySteps = (options.accuracySteps > 0) ? options.accuracySteps : 0;
	accuracyOut = options.accuracyOut;
	headingSpread = options.headingSpread;
	// placements are reproducible when seeded; accuracy runs always are
	runSeed = options.seed;
	if (runSeed < 0 && accuracySteps > 0) runSeed = ACCURACY_DEFAULT_SEED;
	if (runSeed >= 0) srand((unsigned int)runSeed);
	applySHProjection();
	start(options.model);
}

void Simulation::applySHProjection(){
	if (boidModel)
		boidModel->setSHProjection(shMode, vmfK);
}

const char* Simulation::getSHProjectionName(){
	return boidModel->getSHProjectionName();
}

void Simulation::simulationStep(){
	if (pendingModel != currentModel){
		switchToModel(pendingModel);
		// a scene the build does not have leaves currentModel unchanged; stop
		// retrying it every frame
		pendingModel = currentModel;
		return;
	}
	if (accuracySteps > 0){
		// accuracy mode: fixed time step, the model records every step
		timeDiff = TIMING_FIXED_DT;
		boidModel->simulate(timeDiff);
		if (!boidModel->hasAccuracyMetrics() || ++accuracyCount >= accuracySteps)
			accuracyFinish();
		return;
	}
	if (timingSteps == 0){
		timeDiff = getTimeDiff();
		boidModel->simulate(timeDiff);
		return;
	}

	// timing mode: fixed time step, so every run of a scene integrates the same
	// trajectory whatever the frame rate
	timeDiff = TIMING_FIXED_DT;
	auto t0 = std::chrono::steady_clock::now();
	boidModel->simulate(timeDiff);
	auto t1 = std::chrono::steady_clock::now();
	timingRecord(std::chrono::duration<double, std::micro>(t1 - t0).count());
}

void Simulation::timingRecord(double stepUs){
	// frame time: between the ends of consecutive steps (step + render + swap)
	const auto now = std::chrono::steady_clock::now();
	const double frameUs = timingHaveFrame
		? std::chrono::duration<double, std::micro>(now - timingLastFrame).count() : 0.0;
	const bool haveFrame = timingHaveFrame;
	timingLastFrame = now;
	timingHaveFrame = true;

	if (timingWarmup > 0){ timingWarmup--; return; }   // JIT, allocation, first-touch

	const PhaseTimes pt = boidModel->getPhaseTimes();
	statProject.add(pt.projectUs);
	statGrid.add(pt.gridUs);
	statInteract.add(pt.interactUs);
	statUpload.add(pt.uploadUs);
	statStep.add(stepUs);
	if (haveFrame) statFrame.add(frameUs);

	if (statStep.count >= timingSteps){
		timingReport();
		timingSteps = 0;
		GFX::getInstance().appDestroyHandler();
	}
}

void Simulation::timingReport(){
	std::stringstream s;
	s << "timing: scene " << currentSceneName << ", " << simParams.numBodies << " boids, "
	  << (boidModel->getSHProjectionName()[0] ? boidModel->getSHProjectionName() : "no SH")
	  << ", " << statStep.count << " steps (+" << TIMING_WARMUP_STEPS << " warm-up), dt "
	  << TIMING_FIXED_DT << "\n";
	s << "  phase              mean us      min us      max us\n";
	auto row = [&s](const char* name, const TimingStat& t){
		char line[128];
		snprintf(line, sizeof(line), "  %-14s %11.1f %11.1f %11.1f\n", name, t.mean(), t.count ? t.min : 0.0, t.max);
		s << line;
	};
	row("SH projection", statProject);
	row("grid build", statGrid);
	row("interaction", statInteract);
	row("upload (host)", statUpload);
	row("simulate()", statStep);
	row("frame (wall)", statFrame);
	s << "  GPU phases are device profiling times; simulate() is the host-side time of\n"
	     "  the whole step including upload and synchronization; frame includes rendering.\n";

	std::cout << s.str() << std::flush;
	logFile->writeLog(s.str());
}

void Simulation::accuracyFinish(){
	accuracySteps = 0;
	if (!boidModel->hasAccuracyMetrics()){
		std::cerr << "accuracy: scene " << currentSceneName
		          << " has no accuracy metrics (use the SYCL scenes 8 or 9)\n";
		GFX::getInstance().appDestroyHandler();
		return;
	}

	std::stringstream s;
	s << boidModel->getAccuracySummary()
	  << "  seed " << runSeed << ", heading spread " << headingSpread
	  << ", dt " << TIMING_FIXED_DT << "; rows appended to " << accuracyOut << "\n";
	std::cout << s.str() << std::flush;
	logFile->writeLog(s.str());

	// append one row per lane; write the header only into a new / empty file
	bool header = true;
	{
		std::ifstream in(accuracyOut.c_str(), std::ios::ate);
		if (in.good() && in.tellg() > 0) header = false;
	}
	std::stringstream extra;
	extra << currentModel << "," << runSeed << "," << headingSpread;
	std::ofstream out(accuracyOut.c_str(), std::ios::app);
	if (out)
		out << boidModel->getAccuracyCSV("scene,seed,heading_spread", extra.str(), header);
	else
		std::cerr << "accuracy: cannot write " << accuracyOut << "\n";

	GFX::getInstance().appDestroyHandler();
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
	// the SYCL build ports the Simple, Grid, the group-avoidance and the obstacle models
	if (modelNum != BOID_SIMPLE && modelNum != BOID_GRID &&
	    modelNum != BOID_SH && modelNum != BOID_GRID_2D && modelNum != BOID_SH_2D &&
	    modelNum != BOID_SH_WAY1 && modelNum != BOID_SH_WAY2 &&
	    modelNum != BOID_SH_OBSTACLE && modelNum != BOID_SH_OBSTACLE_COMBINED)
		return;
	// the accuracy scenes have their own lane placements; leaving them, drop a lane
	// placement the other scenes cannot use
	if (modelNum == BOID_SH_OBSTACLE)
		currentInitPlacement = PLACEMENT_CROSSING_LANES;
	else if (modelNum == BOID_SH_OBSTACLE_COMBINED)
		currentInitPlacement = PLACEMENT_PLATE_LANES;
	else if (currentInitPlacement >= MODEL_INIT_PLACEMENT_MAX)
		currentInitPlacement = MODEL_INIT_PLACEMENT;
	// two-group scenes default to the two-groups placement (the tunnel too, so the
	// groups funnel through the wall); the four-group and three-columns scenes to
	// the four-groups placement
	if (modelNum == BOID_SH || modelNum == BOID_GRID_2D || modelNum == BOID_SH_WAY2)
		currentInitPlacement = 1;
	else if (modelNum == BOID_SH_2D || modelNum == BOID_SH_WAY1)
		currentInitPlacement = 3;
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
	case 'L':
	case 'l':	//toggle visibility of the acceleration grid
		grid->toggleVisibility();
		break;
	case 't':
	case 'T':	//switch to a different initial boid placement (the lane scenes keep theirs)
		if (currentInitPlacement >= MODEL_INIT_PLACEMENT_MAX)
			break;
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
	case 'p':
	case 'P':	//cycle the SH projection: Dirac -> vMF (-> exact reference in scenes 8, 9)
		if (shMode == SH_PROJECTION_DIRAC)
			shMode = SH_PROJECTION_VMF;
		else if (shMode == SH_PROJECTION_VMF && boidModel->supportsExactReference())
			shMode = SH_PROJECTION_EXACT;
		else
			shMode = SH_PROJECTION_DIRAC;
		applySHProjection();
		break;
	case '[':	//halve / double the vMF concentration k
		vmfK = std::max(vmfK * 0.5f, 1.0f / 64.0f);
		applySHProjection();
		break;
	case ']':
		vmfK = std::min(vmfK * 2.0f, 1024.0f);
		applySHProjection();
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
static const char* const kPlacementNames[MODEL_PLACEMENT_COUNT] = {
	"one group",
	"two groups",
	"two groups asymmetric",
	"four groups",
	"three groups",
	"crossing lanes",
	"plate lanes",
};

const char* Simulation::getPlacementName() const {
	if (currentInitPlacement >= 0 && currentInitPlacement < MODEL_PLACEMENT_COUNT)
		return kPlacementNames[currentInitPlacement];
	return "";
}

void Simulation::createData(std::vector<Vec4> *pos, std::vector<Vec4> *vel, std::vector<Vec4> *goal, std::vector<Vec4> *color){
	const float r = TEST_SETUP_RADIUS;   // group start-spread radius (2x the old TEST_SETUP_RADIUS/2)
	const float maxVel = simParams.maxVel;

	switch(currentInitPlacement){
	case 0:		//random placement inside the cube, goal = own start position
		initRandom(pos, vel, goal, color);
		break;
	case 1:		//"two groups": both groups head toward the world centre, contiguous halves
	{
		const int n = (int)simParams.numBodies;
		const int half = n / 2;
		Vec4 centre = clusterPos(.5f, .5f, .5f, 0.f);   // world centre (no jitter)
		float fx = .5f, fy = .5f, fz0 = .25f, fz1 = .75f;

		// tunnel scene: the default 1/4 and 3/4 spheres overlap the wall, so start each
		// group centred in the free space in front of / behind the wall, level with the
		// openings, and aim the initial heading at the openings
		const SceneConfig* sc = findScene(currentModel);
		if (sc != NULL && sc->showTunnel && tunnel != NULL){
			const float wx = CELL_SIZE_X * simParams.gridSize.x;
			const float wy = CELL_SIZE_Y * simParams.gridSize.y;
			const float wz = CELL_SIZE_Z * simParams.gridSize.z;
			centre = tunnel->openingsCentre();
			fx  = centre.x / wx;
			fy  = centre.y / wy;
			fz0 = 0.5f * tunnel->wallZMin() / wz;
			fz1 = 0.5f * (tunnel->wallZMax() + wz) / wz;
		}

		for (int i = 0; i < half; i++)
		{
			const int j = i + half;
			(*pos)[i] = clusterPos(fx, fy, fz0, r);
			(*vel)[i] = dirTo((*pos)[i], centre, maxVel);   // group 0 heads to centre
			(*color)[i] = COLOR_RED;

			(*pos)[j] = clusterPos(fx, fy, fz1, r);
			(*vel)[j] = dirTo((*pos)[j], centre, maxVel);   // group 1 heads to centre
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
	case 3:		//"four groups": all four groups head toward the world centre, contiguous fourths
	{
		const int n = (int)simParams.numBodies;
		const int q = n / 4;
		const Vec4 centre = clusterPos(.5f, .5f, .5f, 0.f);
		for (int i = 0; i < q; i++)
		{
			const int a = i, b = i + q, c = i + 2 * q, d = i + 3 * q;
			(*pos)[a] = clusterPos(.5f, .5f, .25f, r); (*vel)[a] = dirTo((*pos)[a], centre, maxVel); (*color)[a] = COLOR_RED;
			(*pos)[b] = clusterPos(.5f, .5f, .75f, r); (*vel)[b] = dirTo((*pos)[b], centre, maxVel); (*color)[b] = COLOR_BLUE;
			(*pos)[c] = clusterPos(.25f, .5f, .5f, r); (*vel)[c] = dirTo((*pos)[c], centre, maxVel); (*color)[c] = COLOR_ORANGE;
			(*pos)[d] = clusterPos(.75f, .5f, .5f, r); (*vel)[d] = dirTo((*pos)[d], centre, maxVel); (*color)[d] = COLOR_PURPLE;

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
	case PLACEMENT_CROSSING_LANES:	//accuracy scene 8
		initCrossingLanes(pos, vel, goal, color);
		break;
	case PLACEMENT_PLATE_LANES:		//accuracy scene 9
		initPlateLanes(pos, vel, goal, color);
		break;
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


Vec4 Simulation::noisyHeading(const Vec4& dir, float speed){
	float mx = dir.x, my = dir.y, mz = dir.z;
	const float kh = headingSpread;
	if (kh > 0.0f){
		// sample a vMF direction of concentration kh around dir (Wood's method on S^2):
		// cosine w to the mean, uniform azimuth phi in the tangent plane
		const float u = randFloat(0.0f, 1.0f);
		float w = 1.0f + logf(u + (1.0f - u) * expf(-2.0f * kh)) / kh;
		w = std::max(-1.0f, std::min(1.0f, w));
		const float phi = randFloat(0.0f, 2.0f * (float)CL_M_PI);
		// tangent basis (e1, e2) of the unit mean direction
		float ax = 0.f, ay = 1.f, az = 0.f;
		if (fabsf(my) > 0.9f){ ax = 1.f; ay = 0.f; }
		float e1x = ay * mz - az * my, e1y = az * mx - ax * mz, e1z = ax * my - ay * mx;
		const float l1 = sqrtf(e1x * e1x + e1y * e1y + e1z * e1z);
		e1x /= l1; e1y /= l1; e1z /= l1;
		const float e2x = my * e1z - mz * e1y, e2y = mz * e1x - mx * e1z, e2z = mx * e1y - my * e1x;
		const float s = sqrtf(std::max(0.0f, 1.0f - w * w));
		const float c = cosf(phi), sn = sinf(phi);
		mx = w * mx + s * (c * e1x + sn * e2x);
		my = w * my + s * (c * e1y + sn * e2y);
		mz = w * mz + s * (c * e1z + sn * e2z);
	}
	return Vec4(mx * speed, my * speed, mz * speed, 0.f);
}

//accuracy scene 8: per lane L, groups 2L (heading +x) and 2L+1 (at CROSS_DEG[L] from
//it) start APPROACH before the lane centre and aim at it, so they meet there at the
//lane's angle; their goals lie APPROACH beyond it on the same line. In the 0-degree
//lane the groups fly side by side (parallel control).
//Groups are the contiguous buckets of groupOf(), as the models expect.
void Simulation::initCrossingLanes(std::vector<Vec4> *pos, std::vector<Vec4> *vel, std::vector<Vec4> *goal, std::vector<Vec4> *color){
	const int n = (int)simParams.numBodies;
	const int ng = 2 * AccLanes::LANES;
	const float wx = CELL_SIZE_X * simParams.gridSize.x;
	const float wy = CELL_SIZE_Y * simParams.gridSize.y;
	const float wz = CELL_SIZE_Z * simParams.gridSize.z;
	for (int i = 0; i < n; i++){
		const int g = (int)(((long long)i * ng) / n);
		const int lane = g / 2;
		const bool second = (g & 1) != 0;
		const float a = (second ? AccLanes::CROSS_DEG[lane] : 0.0f) * (float)CL_M_PI / 180.f;
		const Vec4 dir(cosf(a), 0.f, sinf(a), 0.f);
		Vec4 target(0.5f * wx, AccLanes::laneY(lane), 0.5f * wz, 1.f);
		if (second && AccLanes::CROSS_DEG[lane] == 0.0f)
			target.z += AccLanes::PARALLEL_OFFSET;
		const Vec4 startC(target.x - AccLanes::APPROACH * dir.x, target.y, target.z - AccLanes::APPROACH * dir.z, 1.f);

		(*pos)[i] = clusterPos(startC.x / wx, startC.y / wy, startC.z / wz, TEST_SETUP_RADIUS);
		const Vec4 aim = dirTo((*pos)[i], target, 1.0f);
		(*vel)[i] = noisyHeading(aim, simParams.maxVel);
		// goal: APPROACH beyond the lane centre on the group's line (keeps the course)
		(*goal)[i] = Vec4(target.x + AccLanes::APPROACH * dir.x, target.y, target.z + AccLanes::APPROACH * dir.z, 0.f);
		(*color)[i] = second ? COLOR_BLUE : COLOR_RED;
	}
}

//accuracy scene 9: per lane L, one group starts APPROACH before the lane's plate on
//its -x side, at PLATE_DEG[L] from the plate normal, and aims at the plate centre
void Simulation::initPlateLanes(std::vector<Vec4> *pos, std::vector<Vec4> *vel, std::vector<Vec4> *goal, std::vector<Vec4> *color){
	const int n = (int)simParams.numBodies;
	const int ng = AccLanes::LANES;
	const float wx = CELL_SIZE_X * simParams.gridSize.x;
	const float wy = CELL_SIZE_Y * simParams.gridSize.y;
	const float wz = CELL_SIZE_Z * simParams.gridSize.z;
	for (int i = 0; i < n; i++){
		const int lane = (int)(((long long)i * ng) / n);
		const float a = AccLanes::PLATE_DEG[lane] * (float)CL_M_PI / 180.f;
		const Vec4 dir(cosf(a), 0.f, sinf(a), 0.f);
		const Vec4 target(0.5f * wx, AccLanes::laneY(lane), 0.5f * wz, 1.f);
		const Vec4 startC(target.x - AccLanes::APPROACH * dir.x, target.y, target.z - AccLanes::APPROACH * dir.z, 1.f);

		(*pos)[i] = clusterPos(startC.x / wx, startC.y / wy, startC.z / wz, TEST_SETUP_RADIUS);
		const Vec4 aim = dirTo((*pos)[i], target, 1.0f);
		(*vel)[i] = noisyHeading(aim, simParams.maxVel);
		// goal: APPROACH behind the plate on the group's line, so the plate field
		// has to hold the group off (as the tunnel's goals behind the wall)
		(*goal)[i] = Vec4(target.x + AccLanes::APPROACH * dir.x, target.y, target.z + AccLanes::APPROACH * dir.z, 0.f);
		(*color)[i] = (lane % 2) ? COLOR_BLUE : COLOR_RED;
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