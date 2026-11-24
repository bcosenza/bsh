// Copyright (c) 2015-2026, Biagio Cosenza.
// Technische Universitaet Berlin (2015-2019). University of Salerno (2019-2026). All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.


#ifndef _SIMULATION_H_
#define _SIMULATION_H_

#include "common.h"
#include "vector_types.h"
#ifndef USE_SYCL
#include "CLHelper.h"
#endif
#include "logFile.h"
#include "SimParam.h"
#include "BoidModel.h"
#include "WorldBox.h"
#include "Grid.h"
#include "WorldGround.h"
#include "OverlayText.h"
#include "SkyBox.h"
#include "Column.h"
#include "Tunnel.h"
#include "PlateLanes.h"
#include <chrono>

/*
	Command-line options (see main_linux.cpp)
*/
struct RunOptions {
	int   model       = BOID_SIMPLE;			// scene to start in
	int   timingSteps = 0;						// >0: measure that many steps, report, quit
	int   shMode      = SH_PROJECTION_DEFAULT;	// SH projection kernel (SH_PROJECTION_*)
	float vmfK        = SH_VMF_KAPPA;			// vMF concentration k
	int   accuracySteps = 0;					// >0: record SH accuracy that many steps (scenes 8, 9), report, quit
	std::string accuracyOut = "accuracy.csv";	// CSV the accuracy rows are appended to
	int   seed        = -1;						// random seed of the placements (-1: time based)
	float headingSpread = 0.0f;				// lane scenes: vMF concentration of the initial heading noise (0 = none)
	std::vector<int> snapshotFrames;		// write a PNG after these steps (fixed dt, no overlay), then quit
	std::string snapshotDir = ".";			// where the snapshots are written
};

/*
	Boid simulation controler. Handles interaction between view and model.
*/
class Simulation
{
private:
	GLenum error;
#ifndef USE_SYCL
	CLHelper* clHelper;
#endif
	BoidModel* boidModel;
	LogFile* logFile;
	simParams_t simParams;
	WorldBox* worldBox;
	Grid* grid;
	WorldGround* worldGround;
	OverlayText* overlayText;
	Skybox* skybox;
	Column* column1;
	Column* column2;
	Column* column3;
	Tunnel* tunnel;
	PlateLanes* plates;

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

	//SH projection kernel and vMF concentration, kept across restarts and scene switches
	int shMode;
	float vmfK;
	//hand the SH projection settings to the current model
	void applySHProjection();

	//timing mode (-timing t): steps still to measure (0 = off), warm-up steps left,
	//and the accumulated per-step measurements
	int timingSteps;
	int timingWarmup;
	struct TimingStat {
		double sum = 0.0, min = 1e300, max = 0.0;
		long count = 0;
		void add(double v){ sum += v; if (v < min) min = v; if (v > max) max = v; count++; }
		double mean() const { return count ? sum / count : 0.0; }
	};
	TimingStat statProject, statGrid, statInteract, statUpload, statStep, statFrame;
	std::chrono::steady_clock::time_point timingLastFrame;
	bool timingHaveFrame;
	//record one measured step; print the report and quit after the last one
	void timingRecord(double stepUs);
	void timingReport();

	//accuracy mode (-accuracy t, scenes 8 and 9): steps still to record (0 = off),
	//steps recorded, CSV path, seed used; then report, append the CSV and quit
	int accuracySteps;
	int accuracyCount;
	std::string accuracyOut;
	int runSeed;
	void accuracyFinish();
	//lane scenes: vMF concentration of the initial heading noise (0 = none)
	float headingSpread;

	//snapshot mode (-snapshot f1 f2 ...): frames still to write (sorted), the next
	//one, steps simulated in the scene (-1 before its first frame), output folder
	std::vector<int> snapFrames;
	size_t snapNext;
	int snapStep;
	std::string snapDir;

	//create position and velocity data for boids dependend on currentInitPlacement
	void createData(std::vector<Vec4> *pos, std::vector<Vec4> *vel, std::vector<Vec4> *goal, std::vector<Vec4> *color);
	//placement: every boid at a uniform-random position in the world cube with a random velocity
	void initRandom(std::vector<Vec4> *pos, std::vector<Vec4> *vel, std::vector<Vec4> *goal, std::vector<Vec4> *color);
	//placement: two spherical groups (centres P1, P2 at 1/3 and 2/3 of the world),
	//each boid heading toward the opposite group's centre
	void initTwoGroups(std::vector<Vec4> *pos, std::vector<Vec4> *vel, std::vector<Vec4> *goal, std::vector<Vec4> *color);
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
	//accuracy scene 9: lanes of one group facing a plate
	void initPlateLanes(std::vector<Vec4> *pos, std::vector<Vec4> *vel, std::vector<Vec4> *goal, std::vector<Vec4> *color);
	//velocity 'speed' along unit 'dir', perturbed by vMF noise of concentration headingSpread
	Vec4 noisyHeading(const Vec4& dir, float speed);

	Simulation();
	~Simulation();

public:
	//initialize simulation
	void init();
	//start simulation, optionally with a specific model number
	void start(int initialModel = BOID_SIMPLE);
	//start simulation with command-line options (scene, SH projection, timing mode)
	void start(const RunOptions& options);
	//"SH: Dirac" / "SH: vMF (k=..)" for SH models, "" otherwise (for the overlay)
	const char* getSHProjectionName();
	//snapshot mode: true for the whole run (the overlay is hidden); snapshotDue
	//returns the file to write for the frame just rendered, snapshotsDone once all are
	bool isSnapshotRun() const { return !snapFrames.empty(); }
	bool snapshotDue(std::string* path);
	bool snapshotsDone() const { return !snapFrames.empty() && snapNext >= snapFrames.size(); }
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
	//display name of the current initial placement (for the overlay)
	const char* getPlacementName() const;
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