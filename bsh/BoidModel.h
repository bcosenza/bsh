// Copyright (c) 2015, Biagio Cosenza.
// Technische Universitaet Berlin. All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#ifndef _BOIDMODEL_H_
#define _BOIDMODEL_H_

#include "stdafx.h"
#include "CLHelper.h"
#include "SimParam.h"
#include "vector_types.h"
#include "vectorTypes.h"
#include "Shader.h"
#include "Renderable.h"

/*
	Simulation parameters used in OpenCL kernels
*/
typedef struct simParams_t{
	uint3 gridSize;				// number of cells per axis
	unsigned int numCells;		// pre calculated number of cells
	float3 worldOrigin;			// origin of the world in object space (currently not used)
	float3 cellSize;			// size of cells

	unsigned int numBodies;		// number of boids used in the simulation
	unsigned int localSize;

	float wSeparation;			// weight of separation in the simulation
	float wAlignment;			// weight of alignment in the simulation
	float wCohesion;			// weight of cohesion in the simulation
	float wOwn;					// weight of own velocity in simulation
	float wPath;

	float maxVel;				// maximum velocity
	float maxVelCor;			// maximum correction velocity

} simParams_t;

/*
	Virtual base class for boids, implements interface Renderable.
*/
class BoidModel : public Renderable
{
public:
	simParams_t simParams;
	CLHelper* clHelper;

	BoidModel(CLHelper* clHlpr) { clHelper = clHlpr; };
	virtual ~BoidModel() {};

	/* Execute all simulation steps for the boid model
	dt - delta time */
	virtual void simulate(float dt) = 0;

	/* Returns the index of Vertex Buffer Object of positions */
	virtual GLuint getPosVBO() = 0;

	/* Returns index of Vertex Buffer Object of velocities */
	virtual GLuint getVelVBO() = 0;

	/* Returns index of Vertex Array Object */
	virtual GLuint getPosVAO() = 0;

	/* Returns number of boids */
	virtual int getNumBoid() = 0;

	/* Returns execution time of the simulate kernel */
	virtual long getSimulationTime() = 0;

	/* Returns std::vector with pointers to text which is used to display text in the interface */
	virtual std::vector<const char*> getSimTimeDescriptions() = 0;

	/* Get position data of a specific Boid, boidIndex passed in may be changed to new Index after reordering */
	virtual void getFollowedBoid(unsigned int* boidIndex, Vec4 *pos, Vec4 *vel) = 0;

	/* Helper method to write to the log file */
	inline void log(std::string entry){
		clHelper->log(entry);
	};
};

/*
	Naive boid model, which is non optimized for current GPU architecture
*/
class BoidModelSimple : public BoidModel
{
public:
	BoidModelSimple(CLHelper* clHlpr, std::vector<Vec4> pos, std::vector<Vec4> vel, simParams_t* simP);
	~BoidModelSimple();

	// Inheritate from BoidModel
	void simulate(float dt);
	GLuint getPosVBO();
	GLuint getVelVBO();
	GLuint getPosVAO();
	int getNumBoid();
	long getSimulationTime();
	std::vector<const char*> getSimTimeDescriptions();
	void getFollowedBoid(unsigned int* boidIndex, Vec4 *pos, Vec4 *vel);

	// Inheritate from Renderable
	void render();
	Shader* getShader();
	void bindShader();
	void unbindShader();

private:
	/* Create Vertex Array Object and Vertex Buffer Object
	pos - vector of Vec4 which contains boid positions */
	void createVboBindShader(std::vector<Vec4> pos, std::vector<Vec4> vel);

	/* Load the openCL program file */
	void loadProgram(const std::string &filename);

	/* Load the kernel from the program file */
	void loadKernel();

	/* Create buffer and link position buffer with VBO
	pos - vector of Vec4 which contains boid positions */
	void createBuffer(std::vector<Vec4> pos, std::vector<Vec4> vel);

	/* Load data from host to openCL device
	vel - vector of Vec4 which contains boid velocities */
	void loadData();

	// helper is used to switch between input and output position buffer
	int helper = 0;
	GLuint pos_vbo[1];
	GLuint pos_vao[1];
	GLuint pos_out_vbo[1];
	GLuint pos_out_vao[1];

	GLuint vel_vbo[1];
	GLuint vel_out_vbo[1];

	// number of boids
	int num;

	// Pointer to simulation time discription strings
	std::vector<const char*> simTimeDisc;

	// Simulation time as string for overlay text
	std::string stringSimTime;

	// Attributes used in the shader
	std::vector<std::string> attribName;
	Shader* shader;

	cl::Context context;
	cl::CommandQueue queue;
	cl::Program program;
	std::vector<cl::Device> devices;
	cl::Kernel kernel;
	cl::Event event;
	cl::Event eventSim;

	// boid position shared with OpenGL
	std::vector<cl::Memory> cl_pos_vbos;
	std::vector<cl::Memory> cl_pos_vbos_out;

	std::vector<cl::Memory> cl_vel_vbos;
	std::vector<cl::Memory> cl_vel_vbos_out;

	// plain CL buffers used when GL interop is unavailable
	bool useGLInterop = false;
	cl::Buffer cl_pos_plain, cl_pos_out_plain, cl_vel_plain, cl_vel_out_plain;

	cl::Buffer cl_simParams;

	cl_int err;
};

/*
	Common host-side machinery shared by all grid based boid models:
	shader/VBO setup, the CL buffers of the uniform grid, kernel loading,
	the hash -> sort -> reorder pipeline, bitonic sort, per-stage profiling
	and the Renderable plumbing. Subclasses provide the scene specific
	kernels and the tail of simulate().

	All entries of times[] are kept in microseconds.
*/
class BoidModelGridBase : public BoidModel
{
public:
	virtual ~BoidModelGridBase();

	// BoidModel interface
	GLuint getPosVBO();
	GLuint getVelVBO();
	GLuint getPosVAO();
	int getNumBoid();
	long getSimulationTime();
	std::vector<const char*> getSimTimeDescriptions();
	void getFollowedBoid(unsigned int* boidIndex, Vec4 *pos, Vec4 *vel);

	// Renderable interface
	void render();
	Shader* getShader();
	void bindShader();
	void unbindShader();

protected:
	/* doubleBuffered - true for models that reorder into a second VBO set and
	   ping-pong between the two every frame */
	BoidModelGridBase(CLHelper* clHlpr, simParams_t* simP, const char* modelName, bool doubleBuffered);

	// ---- setup helpers (call order: createCommonBuffers, [createGoalBuffers,]
	// ---- uploadSimParams, loadProgram(s), loadCommonKernels) ----

	/* Compile an OpenCL program from a source file */
	cl::Program loadProgram(const std::string &filename);

	/* Compile a boid kernel file with kernels/common.cl prepended, which
	   provides the shared simParams_t/memSet/getGridHash declarations */
	cl::Program loadBoidProgram(const std::string &filename);

	/* Load the kernels every grid model uses from programBoid/programBitonic
	   (getGridHash, findGridEdgeAndReorder, simulate, memSet, bitonic sort) */
	void loadCommonKernels();

	/* Create the shader, the pos/vel VBO set(s) and the uniform grid CL buffers.
	   color - per boid colors; pass an empty vector for a constant BOID_COLOR.
	   reorderedColor - true to create CL-shared color VBOs which the model
	   reorders every frame (goal seeking scenes with multiple groups). */
	void createCommonBuffers(const std::vector<Vec4>& pos, const std::vector<Vec4>& vel,
		const std::vector<Vec4>& color, bool reorderedColor);

	/* Create the goal double buffer and fill it with the initial goals */
	void createGoalBuffers(const std::vector<Vec4>& goal);

	/* Upload simParams to the device */
	void uploadSimParams();

	/* Add a row to the timing overlay; timesIndex selects the times[] slot */
	void addTimingRow(const char* label, int timesIndex);

	// ---- per frame pipeline steps, called from the subclass' simulate() ----

	/* Wait for GL and acquire all GL-shared buffers for CL */
	void acquireGLBuffers();
	/* Release all GL-shared buffers back to GL */
	void releaseGLBuffers();

	/* Compute the grid hash of every boid in posIn -> times[0] */
	void runGridHash(const cl::Memory& posIn);

	/* Reset the per-cell start/end indices */
	void clearGridEdges();

	/* Sort the grid hash -> times[1] */
	void sortGridHash();

	/* Enqueue findGridEdgeAndReorder -> times[2]. The pos/vel (and model
	   specific goal/color) buffer arguments starting at index 2/3 and 6/7 must
	   be set by the caller beforehand; the common arguments and the trailing
	   local memory/boid count are set here. */
	void runReorder();

	/* Profiled execution time of an event in microseconds */
	long eventTimeUs(cl::Event& e);

	// in/out buffer selection for double buffered models
	bool useInBuffers() const { return !doubleBuffered || counter; }

	// ---- members ----
	const char* modelName;
	bool doubleBuffered;
	// flipped at the start of every simulate() of a double buffered model
	bool counter;

	// number of boids
	int num;

	GLuint pos_vbo[1], pos_vbo_out[1];
	GLuint pos_vao[1], pos_vao_out[1];
	GLuint vel_vbo[1], vel_vbo_out[1];
	GLuint color_vbo[1], color_vbo_out[1];

	Shader* shader;

	cl::Context context;
	cl::CommandQueue queue;
	std::vector<cl::Device> devices;

	cl::Program programBoid;
	cl::Program programBitonic;

	cl::Kernel kernel_getGridHash;
	cl::Kernel kernel_findGridEdgeAndReorder;
	cl::Kernel kernel_simulate;
	cl::Kernel kernel_memSet;
	cl::Kernel kernel_bitonicSortLocal;
	cl::Kernel kernel_bitonicSortLocal1;
	cl::Kernel kernel_bitonicMergeGlobal;
	cl::Kernel kernel_bitonicMergeLocal;

	cl::Event event;
	cl::Event eventSim;

	// boid buffers shared with OpenGL
	std::vector<cl::Memory> cl_pos_vbos, cl_pos_vbos_out;
	std::vector<cl::Memory> cl_vel_vbos, cl_vel_vbos_out;
	std::vector<cl::Memory> cl_color_vbos, cl_color_vbos_out;

	// goal double buffer (only when createGoalBuffers was called)
	cl::Buffer cl_goal_in, cl_goal_out;

	// uniform grid buffers
	cl::Buffer cl_gridHash_unsorted, cl_gridHash_sorted;
	cl::Buffer cl_gridIndex_unsorted, cl_gridIndex_sorted;
	cl::Buffer cl_gridStartIndex, cl_gridEndIndex;
	cl::Buffer cl_range;
	cl::Buffer cl_simParams;

	// per stage execution times in microseconds
	long times[8];

	cl_int err;

private:
	void bitonicSort(cl::Buffer d_DstKey, cl::Buffer d_DstVal, cl::Buffer d_SrcKey,
		cl::Buffer d_SrcVal, unsigned int batch, unsigned int arrayLength, unsigned int dir);
	cl_uint factorRadix2(cl_uint& log2L, cl_uint L);

	std::string readKernelFile(const std::string &filename);
	cl::Program buildProgram(const std::string &kernelSource);

	/* create one VAO with pos/vel/color VBOs; out parameters receive the ids */
	void createVboSet(const std::vector<Vec4>& pos, const std::vector<Vec4>& vel,
		const std::vector<Vec4>& color, bool reorderedColor,
		GLuint* vao, GLuint* posVbo, GLuint* velVbo, GLuint* colorVbo);

	// rows of the timing overlay: label and times[] index
	std::vector<std::pair<const char*, int> > timingRows;
	std::vector<std::string> timingStrings;
	std::vector<const char*> simTimeDisc;
};

/*
	Improved boid model using the GPU. It sorts the boids by their position in
	cells. The velocity and position buffer are reordered, according to the
	sorted index, for coalesced memory access.
*/
class BoidModelGrid : public BoidModelGridBase
{
public:
	BoidModelGrid(CLHelper* clHlpr, std::vector<Vec4> pos, std::vector<Vec4> vel, simParams_t* simP);
	void simulate(float dt);

protected:
	BoidModelGrid(CLHelper* clHlpr, std::vector<Vec4> pos, std::vector<Vec4> vel, simParams_t* simP,
		const std::string& kernelFile, const char* modelName);

	// reordered pos/vel, input of the simulation kernel
	cl::Buffer cl_pos_out;
	cl::Buffer cl_velocities_out;
};

/* The same as BoidModelGrid with a 2D kernel: velocity on the Y axis is 0. */
class BoidModelGrid_2D : public BoidModelGrid
{
public:
	BoidModelGrid_2D(CLHelper* clHlpr, std::vector<Vec4> pos, std::vector<Vec4> vel, simParams_t* simP);
};

/* Boid model with Spherical Harmonics long-range collision avoidance.
   Basically BoidModelGrid extended with SH. */
class BoidModelSH : public BoidModelGridBase
{
public:
	BoidModelSH(CLHelper* clHlpr, std::vector<Vec4> pos, std::vector<Vec4> vel, simParams_t* simP);
	void simulate(float dt);

protected:
	BoidModelSH(CLHelper* clHlpr, std::vector<Vec4> pos, std::vector<Vec4> vel, simParams_t* simP,
		const std::string& kernelFile, const char* modelName);

	/* set the trailing useSH arguments; the 2D variant inserts the fixed Y height */
	virtual void setUseSHFinalArgs(float dt);

	// summed velocity per cell, input of the SH evaluation
	cl::Buffer cl_sumVel;

	cl::Kernel kernel_sumVelSH;
	cl::Kernel kernel_useSH;
};

/* The same as BoidModelSH with a 2D kernel: the boids are placed on a fixed Y
   plane and their useSH kernel keeps them there. */
class BoidModelSH_2D : public BoidModelSH
{
public:
	BoidModelSH_2D(CLHelper* clHlpr, std::vector<Vec4> pos, std::vector<Vec4> vel, simParams_t* simP);

protected:
	void setUseSHFinalArgs(float dt);

private:
	// Y position all boids are kept on
	float yAxisFixed;
};

/* BoidModelSH with path finding. The SH representation is aggregated per cell. */
class BoidModelSHWay1 : public BoidModelGridBase
{
public:
	BoidModelSHWay1(CLHelper* clHlpr, std::vector<Vec4> pos, std::vector<Vec4> vel,
		std::vector<Vec4> goal, std::vector<Vec4> color, simParams_t* simP);
	void simulate(float dt);

private:
	// per cell SH coefficients of the aggregated velocities
	cl::Buffer cl_shEvalX, cl_shEvalY, cl_shEvalZ;
	cl::Buffer cl_coef0X, cl_coef0Y, cl_coef0Z;

	cl::Kernel kernel_evalSH;
	cl::Kernel kernel_useSH;
};

/* BoidModelSH with path finding. The SH representation is kept per boid,
   the unaggregated (and much more expensive) reference of Way1. */
class BoidModelSHWay2 : public BoidModelGridBase
{
public:
	BoidModelSHWay2(CLHelper* clHlpr, std::vector<Vec4> pos, std::vector<Vec4> vel,
		std::vector<Vec4> goal, std::vector<Vec4> color, simParams_t* simP);
	void simulate(float dt);

private:
	// per boid SH coefficients
	cl::Buffer cl_shEvalX, cl_shEvalY, cl_shEvalZ;
	cl::Buffer cl_coef0X, cl_coef0Y, cl_coef0Z;

	cl::Kernel kernel_evalSH;
	cl::Kernel kernel_useSH;
};

/* SH obstacle avoidance: the obstacle geometry is projected once into SH
   coefficients which repel the goal seeking boids. */
class BoidModelSHObstacle : public BoidModelGridBase
{
public:
	BoidModelSHObstacle(CLHelper* clHlpr, std::vector<Vec4> pos, std::vector<Vec4> vel,
		std::vector<Vec4> goal, simParams_t* simP, std::vector<Vec4> cor,
		std::vector<unsigned int> start, std::vector<unsigned int> end, std::vector<Vec4> posObst);
	void simulate(float dt);

private:
	// per boid SH coefficients
	cl::Buffer cl_shEvalX, cl_shEvalY, cl_shEvalZ;
	cl::Buffer cl_coef0X, cl_coef0Y, cl_coef0Z;

	// SH representation of the static obstacles, computed once at setup
	cl::Buffer cl_shEvalOX, cl_shEvalOY, cl_shEvalOZ;
	cl::Buffer cl_coef0OX, cl_coef0OY, cl_coef0OZ;
	cl::Buffer cl_cor, cl_startCor, cl_endCor, cl_posObst;
	unsigned int numObst;

	cl::Kernel kernel_evalSH;
	cl::Kernel kernel_useSH;
	cl::Kernel kernel_obstacle;

	void createAndLoadObstacleSH(const std::vector<Vec4>& cor, const std::vector<unsigned int>& start,
		const std::vector<unsigned int>& end, const std::vector<Vec4>& posObst);
};

/* Way1 path finding combined with SH obstacle avoidance. */
class BoidModelSHCombined : public BoidModelGridBase
{
public:
	BoidModelSHCombined(CLHelper* clHlpr, std::vector<Vec4> pos, std::vector<Vec4> vel,
		std::vector<Vec4> goal, std::vector<Vec4> color, simParams_t* simP, std::vector<Vec4> cor,
		std::vector<unsigned int> start, std::vector<unsigned int> end, std::vector<Vec4> posObst);
	void simulate(float dt);

protected:
	BoidModelSHCombined(CLHelper* clHlpr, std::vector<Vec4> pos, std::vector<Vec4> vel,
		std::vector<Vec4> goal, std::vector<Vec4> color, simParams_t* simP, std::vector<Vec4> cor,
		std::vector<unsigned int> start, std::vector<unsigned int> end, std::vector<Vec4> posObst,
		const std::string& kernelFile, const char* modelName);

private:
	// per boid SH coefficients
	cl::Buffer cl_shEvalX, cl_shEvalY, cl_shEvalZ;
	cl::Buffer cl_coef0X, cl_coef0Y, cl_coef0Z;

	// SH representation of the static obstacles, computed once at setup
	cl::Buffer cl_shEvalOX, cl_shEvalOY, cl_shEvalOZ;
	cl::Buffer cl_coef0OX, cl_coef0OY, cl_coef0OZ;
	cl::Buffer cl_cor, cl_startCor, cl_endCor, cl_posObst;
	unsigned int numObst;

	cl::Kernel kernel_evalSH;
	cl::Kernel kernel_useSH;
	cl::Kernel kernel_obstacle;

	void createAndLoadObstacleSH(const std::vector<Vec4>& cor, const std::vector<unsigned int>& start,
		const std::vector<unsigned int>& end, const std::vector<Vec4>& posObst);
};

/* The tunnel test case: BoidModelSHCombined with the tunnel kernel. */
class BoidModelSHObstacleTunnel : public BoidModelSHCombined
{
public:
	BoidModelSHObstacleTunnel(CLHelper* clHlpr, std::vector<Vec4> pos, std::vector<Vec4> vel,
		std::vector<Vec4> goal, std::vector<Vec4> color, simParams_t* simP, std::vector<Vec4> cor,
		std::vector<unsigned int> start, std::vector<unsigned int> end, std::vector<Vec4> posObst);
};

#endif // _BOIDMODEL_H_
