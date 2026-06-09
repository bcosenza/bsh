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
#include "simParam.h"
#include "vector_types.h"
#include "vectorTypes.h"
#include "shader.h"
#include "renderable.h"

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

	cl::Buffer cl_simParams;

	cl_int err;
};

/*
	Improvde boid model using the GPU. It sorts the boids by their position in cells.
	The velocity and position buffer are reordered, according to the sorted index, for coalesced memory access.
*/
class BoidModelGrid : public BoidModel
{
public:
	BoidModelGrid(CLHelper* clHlpr, std::vector<Vec4> pos, std::vector<Vec4> vel, simParams_t* simP);
	~BoidModelGrid();

	// override BoidModel
	void simulate(float dt);
	GLuint getPosVBO();
	GLuint getVelVBO();
	GLuint getPosVAO();
	int getNumBoid();
	long getSimulationTime();
	std::vector<const char*> getSimTimeDescriptions();
	void getFollowedBoid(unsigned int* boidIndex, Vec4 *pos, Vec4 *vel);
	
	// override Renderable
	void render();
	Shader* getShader();
	void bindShader();
	void unbindShader();


private:
	/* loads openCL program from file
	filename - Name of file from which the program is loaded */
	cl::Program loadProgram(const std::string &filename);

	// load kernel from program file
	void loadKernel();

	/* create buffer
	pos - vector of Vec4 with position data for boids */
	void createBuffer(std::vector<Vec4> pos, std::vector<Vec4> vel);

	/* load data from host to device
	vel - vector of Vec4 with velocity data for boids */
	void loadData(std::vector<Vec4> vel);

	/* bitonic sort for key-value pairs (NVIDIA implementation)
	-d_DstKey Destination for output keys
	-d_DstVal Destination for output value
	-d_SrcKey Source key which is used for sorting
	-d_SrcVal Source value which is sorted
	-batch size
	-arrayLength number of elements to be sorted
	-dir sort direction (ascending or descending)*/
	void bitonicSort(cl::Buffer d_DstKey, cl::Buffer d_DstVal, cl::Buffer d_SrcKey, cl::Buffer d_SrcVal, unsigned int batch, unsigned int arrayLength, unsigned int dir);

	// create the Vertex Buffer Object and
	void createVboBindShader(std::vector<Vec4> pos, std::vector<Vec4> vel);

	static cl_uint factorRadix2(cl_uint& log2L, cl_uint L);

	// index of VBO
	GLuint pos_vbo[1];
	GLuint vel_vbo[1];
	// index of VAO
	GLuint pos_vao[1];
	// number of boids
	int num;

	// vector of simulation time discription strings
	std::vector<const char*> simTimeDisc;
	// string with time of simulate kernel
	std::string stringSimTime;
	// string with time of calculate cell hash kernel
	std::string stringHashTime;
	// string with time of sorting (all bitonic sort/merge steps)
	std::string stringSortTime;
	// string with time for edge detection and memory reordering
	std::string stringEdgeTime;
	// array with times which cast into string for overlay text
	long times[4];

	cl::Context context;
	cl::CommandQueue queue;
	cl::Program programBoid;
	cl::Program programBitonic;
	std::vector<cl::Device> devices;

	// kernel to calculate the grid hash value dependent on position of a boid
	cl::Kernel kernel_getGridHash;
	/* kernel to find start and end index of cells and reorder velocity and position
	of boids in memory */
	cl::Kernel kernel_findGridEdgeAndReorder;
	// simulation kernel
	cl::Kernel kernel_simulate;
	// bitonic sort kernels
	cl::Kernel kernel_bitonicSortLocal;
	cl::Kernel kernel_bitonicSortLocal1;
	cl::Kernel kernel_bitonicMergeGlobal;
	cl::Kernel kernel_bitonicMergeLocal;
	/* kernel to set the start and end index of edges to 0
	without this number of elemnts in cell is undefined */
	cl::Kernel kernel_memSet;

	cl::Event event;
	cl::Event eventSim;

	std::vector<cl::Memory> cl_pos_vbos;
	std::vector<cl::Memory> cl_vel_vbos;
	//
	cl::Buffer cl_pos_out;
	cl::Buffer cl_range;
	cl::Buffer cl_velocities_out;
	cl::Buffer cl_gridHash_unsorted;
	cl::Buffer cl_gridHash_sorted;
	cl::Buffer cl_gridIndex_unsorted;
	cl::Buffer cl_gridIndex_sorted;
	cl::Buffer cl_simParams;
	cl::Buffer cl_gridStartIndex;
	cl::Buffer cl_gridEndIndex;

	cl_int err;

	std::vector<std::string> attribName;
	Shader* shader;
	// Note: logically shared with BitonicSort_b.cl!
	// static const unsigned int LOCAL_SIZE_LIMIT = 512
};

/* Boid model with Spherical Harmonics long-range collision avoidance. Basically BoidModelGrid extended with SH. */
class BoidModelSH : public BoidModel
{
public:
	BoidModelSH(CLHelper* clHlpr, std::vector<Vec4> pos, std::vector<Vec4> vel, simParams_t* simP);
	~BoidModelSH();

	// override BoidModel
	void simulate(float dt);
	GLuint getPosVBO();
	GLuint getVelVBO();
	GLuint getPosVAO();
	int getNumBoid();
	long getSimulationTime();
	std::vector<const char*> getSimTimeDescriptions();
	void getFollowedBoid(unsigned int* boidIndex, Vec4 *pos, Vec4 *vel);

	// override Renderable
	void render();
	Shader* getShader();
	void bindShader();
	void unbindShader();

private:
	cl::Program loadProgram(const std::string &filename);
	void loadKernel();
	void createBuffer(std::vector<Vec4> pos, std::vector<Vec4> vel);
	void loadData();
	void bitonicSort(cl::Buffer d_DstKey, cl::Buffer d_DstVal, cl::Buffer d_SrcKey, cl::Buffer d_SrcVal, unsigned int batch, unsigned int arrayLength, unsigned int dir);
	void createVboBindShader(std::vector<Vec4> pos, std::vector<Vec4> vel);

	static cl_uint factorRadix2(cl_uint& log2L, cl_uint L);

	int helper = 0;
	GLuint pos_vbo[1];
	GLuint vel_vbo[1];
	GLuint pos_vao[1];
	
	GLuint vel_vbo_out[1];
	GLuint pos_vbo_out[1];
	GLuint pos_vao_out[1];
	int num;

	bool counter = false;

	std::vector<const char*> simTimeDisc;
	std::string stringSimTime;
	std::string stringHashTime;
	std::string stringSortTime;
	std::string stringEdgeTime;
	// string of the spherical harmonics step
	std::string stringSHTime;
	// string of the reduction of the velocities
	std::string stringSumTime;
	long times[6];

	cl::Context context;
	cl::CommandQueue queue;
	cl::Program programBoid;
	cl::Program programBitonic;
	std::vector<cl::Device> devices;

	cl::Kernel kernel_getGridHash;
	cl::Kernel kernel_getGridEdge;
	cl::Kernel kernel_findGridEdgeAndReorder;
	cl::Kernel kernel_simulate;
	cl::Kernel kernel_bitonicSortLocal;
	cl::Kernel kernel_bitonicSortLocal1;
	cl::Kernel kernel_bitonicMergeGlobal;
	cl::Kernel kernel_bitonicMergeLocal;
	cl::Kernel kernel_memSet;

	// evalute spherical harmonics coefficients (implementation from P.P. Sloan)
	cl::Kernel kernel_SHEval3;
	// simple reduction to get sum of velocities per cell
	cl::Kernel kernel_sumVelSH;
	// extra step to apply SH to boid simulation
	cl::Kernel kernel_useSH;

	cl::Event event;
	cl::Event eventSim;

	std::vector<cl::Memory> cl_pos_vbos;
	std::vector<cl::Memory> cl_pos_vbos_out;
	std::vector<cl::Memory> cl_vel_vbos;
	std::vector<cl::Memory> cl_vel_vbos_out;


	cl::Buffer cl_range;
	cl::Buffer cl_gridHash_unsorted;
	cl::Buffer cl_gridHash_sorted;
	cl::Buffer cl_gridIndex_unsorted;
	cl::Buffer cl_gridIndex_sorted;
	cl::Buffer cl_simParams;
	cl::Buffer cl_gridStartIndex;
	cl::Buffer cl_gridEndIndex;
	// sum of velocities
	cl::Buffer cl_sumVel;

	cl_int err;

	std::vector<std::string> attribName;
	Shader* shader;
};

/* Currently exactly the same as BoidModelGrid. Velocity on Y axis is set to 0. */
class BoidModelGrid_2D : public BoidModel
{
public:
	BoidModelGrid_2D(CLHelper* clHlpr, std::vector<Vec4> pos, std::vector<Vec4> vel, simParams_t* simP);
	~BoidModelGrid_2D();
	void simulate(float dt);
	GLuint getPosVBO();
	GLuint getVelVBO();
	GLuint getPosVAO();
	int getNumBoid();
	long getSimulationTime();
	void getFollowedBoid(unsigned int* boidIndex, Vec4 *pos, Vec4 *vel);

	void render();
	Shader* getShader();
	void bindShader();
	void unbindShader();
	std::vector<const char*> getSimTimeDescriptions();

private:
	cl::Program loadProgram(const std::string &filename);
	void loadKernel();
	void createBuffer(std::vector<Vec4> pos, std::vector<Vec4> vel);
	void loadData(std::vector<Vec4> vel);
	void bitonicSort(cl::Buffer d_DstKey, cl::Buffer d_DstVal, cl::Buffer d_SrcKey, cl::Buffer d_SrcVal, unsigned int batch, unsigned int arrayLength, unsigned int dir);
	void createVboBindShader(std::vector<Vec4> pos, std::vector<Vec4> vel);

	static cl_uint factorRadix2(cl_uint& log2L, cl_uint L);

	float Y_AxisFixed;

	int helper = 0;
	GLuint pos_vbo[1];
	GLuint vel_vbo[1];
	GLuint pos_vao[1];
	int num;

	std::vector<const char*> simTimeDisc;
	std::string stringSimTime;
	std::string stringHashTime;
	std::string stringSortTime;
	std::string stringEdgeTime;
	long times[4];

	cl::Context context;
	cl::CommandQueue queue;
	cl::Program programBoid;
	cl::Program programBitonic;
	std::vector<cl::Device> devices;

	cl::Kernel kernel_getGridHash;
	cl::Kernel kernel_getGridEdge;
	cl::Kernel kernel_findGridEdgeAndReorder;
	cl::Kernel kernel_simulate;
	cl::Kernel kernel_bitonicSortLocal;
	cl::Kernel kernel_bitonicSortLocal1;
	cl::Kernel kernel_bitonicMergeGlobal;
	cl::Kernel kernel_bitonicMergeLocal;
	cl::Kernel kernel_memSet;

	cl::Event event;
	cl::Event eventSim;

	std::vector<cl::Memory> cl_pos_vbos;
	std::vector<cl::Memory> cl_vel_vbos;

	cl::Buffer cl_pos_out;
	cl::Buffer cl_range;
	cl::Buffer cl_velocities_out;
	cl::Buffer cl_gridHash_unsorted;
	cl::Buffer cl_gridHash_sorted;
	cl::Buffer cl_gridIndex_unsorted;
	cl::Buffer cl_gridIndex_sorted;
	cl::Buffer cl_simParams;
	cl::Buffer cl_gridStartIndex;
	cl::Buffer cl_gridEndIndex;

	cl_int err;

	std::vector<std::string> attribName;
	Shader* shader;
};

/* Currently exactly the same as BoidModelSH. Velocity on Y axis is set to 0.*/
class BoidModelSH_2D : public BoidModel
{
public:
	BoidModelSH_2D(CLHelper* clHlpr, std::vector<Vec4> pos, std::vector<Vec4> vel, simParams_t* simP);
	~BoidModelSH_2D();
	void simulate(float dt);
	GLuint getPosVBO();
	GLuint getVelVBO();
	GLuint getPosVAO();
	int getNumBoid();
	long getSimulationTime();
	std::vector<const char*> getSimTimeDescriptions();
	void getFollowedBoid(unsigned int* boidIndex, Vec4 *pos, Vec4 *vel);

	void render();
	Shader* getShader();
	void bindShader();
	void unbindShader();

private:
	cl::Program loadProgram(const std::string &filename);
	void loadKernel();
	void createBuffer(std::vector<Vec4> pos, std::vector<Vec4> vel);
	void loadData();
	void bitonicSort(cl::Buffer d_DstKey, cl::Buffer d_DstVal, cl::Buffer d_SrcKey, cl::Buffer d_SrcVal, unsigned int batch, unsigned int arrayLength, unsigned int dir);
	void createVboBindShader(std::vector<Vec4> pos, std::vector<Vec4> vel);

	static cl_uint factorRadix2(cl_uint& log2L, cl_uint L);

	GLuint pos_vbo[1];
	GLuint vel_vbo[1];
	GLuint pos_vao[1];

	GLuint vel_vbo_out[1];
	GLuint pos_vbo_out[1];
	GLuint pos_vao_out[1];
	int num;

	bool counter = false;

	float Y_AxisFixed;

	std::vector<const char*> simTimeDisc;
	std::string stringSimTime;
	std::string stringHashTime;
	std::string stringSortTime;
	std::string stringEdgeTime;
	std::string stringSHTime;
	std::string stringSumTime;
	long times[6];

	cl::Context context;
	cl::CommandQueue queue;
	cl::Program programBoid;
	cl::Program programBitonic;
	std::vector<cl::Device> devices;

	cl::Kernel kernel_getGridHash;
	cl::Kernel kernel_getGridEdge;
	cl::Kernel kernel_findGridEdgeAndReorder;
	cl::Kernel kernel_simulate;
	cl::Kernel kernel_bitonicSortLocal;
	cl::Kernel kernel_bitonicSortLocal1;
	cl::Kernel kernel_bitonicMergeGlobal;
	cl::Kernel kernel_bitonicMergeLocal;
	cl::Kernel kernel_memSet;

	cl::Kernel kernel_SHEval3;
	cl::Kernel kernel_sumVelSH;
	cl::Kernel kernel_useSH;

	cl::Event event;
	cl::Event eventSim;

	std::vector<cl::Memory> cl_pos_vbos;
	std::vector<cl::Memory> cl_pos_vbos_out;
	std::vector<cl::Memory> cl_vel_vbos;
	std::vector<cl::Memory> cl_vel_vbos_out;

	cl::Buffer cl_range;
	cl::Buffer cl_gridHash_unsorted;
	cl::Buffer cl_gridHash_sorted;
	cl::Buffer cl_gridIndex_unsorted;
	cl::Buffer cl_gridIndex_sorted;
	cl::Buffer cl_simParams;
	cl::Buffer cl_gridStartIndex;
	cl::Buffer cl_gridEndIndex;
	cl::Buffer cl_sumVel;

	cl_int err;

	std::vector<std::string> attribName;
	Shader* shader;
};

/* BoidModelSH with path finding. */
class BoidModelSHWay1 : public BoidModel
{
public:
	BoidModelSHWay1(CLHelper* clHlpr, std::vector<Vec4> pos, std::vector<Vec4> vel, std::vector<Vec4> goal, std::vector<Vec4> color, simParams_t* simP);
	~BoidModelSHWay1();

	// override from super class BoidModel
	void simulate(float dt);
	GLuint getPosVBO();
	GLuint getVelVBO();
	GLuint getPosVAO();
	int getNumBoid();
	long getSimulationTime();
	std::vector<const char*> getSimTimeDescriptions();
	void getFollowedBoid(unsigned int* boidIndex, Vec4 *pos, Vec4 *vel);

	// override from interface Renderable
	void render();
	Shader* getShader();
	void bindShader();
	void unbindShader();

private:
	cl::Program loadProgram(const std::string &filename);
	void loadKernel();
	void createBuffer(std::vector<Vec4> pos, std::vector<Vec4> vel, std::vector<Vec4> goal, std::vector<Vec4> color);
	void loadData(std::vector<Vec4> goal);
	void bitonicSort(cl::Buffer d_DstKey, cl::Buffer d_DstVal, cl::Buffer d_SrcKey, cl::Buffer d_SrcVal, unsigned int batch, unsigned int arrayLength, unsigned int dir);
	void createVboBindShader(std::vector<Vec4> pos, std::vector<Vec4> vel, std::vector<Vec4> color);

	static cl_uint factorRadix2(cl_uint& log2L, cl_uint L);

	int helper = 0;
	GLuint pos_vbo[1];
	GLuint vel_vbo[1];
	GLuint color_vbo[1];
	GLuint pos_vao[1];

	GLuint vel_vbo_out[1];
	GLuint pos_vbo_out[1];
	GLuint color_vbo_out[1];
	GLuint pos_vao_out[1];
	int num;

	bool counter = false;

	std::vector<const char*> simTimeDisc;
	std::string stringSimTime;
	std::string stringHashTime;
	std::string stringSortTime;
	std::string stringEdgeTime;
	// string of the spherical harmonics step
	std::string stringSHTime;
	// string of the reduction of the velocities
	std::string stringSumTime;
	long times[6];

	cl::Context context;
	cl::CommandQueue queue;
	cl::Program programBoid;
	cl::Program programBitonic;
	std::vector<cl::Device> devices;

	cl::Kernel kernel_getGridHash;
	cl::Kernel kernel_getGridEdge;
	cl::Kernel kernel_findGridEdgeAndReorder;
	cl::Kernel kernel_simulate;
	cl::Kernel kernel_bitonicSortLocal;
	cl::Kernel kernel_bitonicSortLocal1;
	cl::Kernel kernel_bitonicMergeGlobal;
	cl::Kernel kernel_bitonicMergeLocal;
	cl::Kernel kernel_memSet;

	//evalute spherical harmonics coefficients (implementation from P.P. Sloan)
	cl::Kernel kernel_SHEval3;
	//simple reduction to get sum of velocities per cell
	cl::Kernel kernel_evalSH;
	//extra step to apply SH to boid simulation
	cl::Kernel kernel_useSH;

	cl::Event event;
	cl::Event eventSim;

	std::vector<cl::Memory> cl_pos_vbos;
	std::vector<cl::Memory> cl_pos_vbos_out;
	std::vector<cl::Memory> cl_vel_vbos;
	std::vector<cl::Memory> cl_vel_vbos_out;
	std::vector<cl::Memory> cl_color_vbos;
	std::vector<cl::Memory> cl_color_vbos_out;

	cl::Buffer cl_shEvalX;
	cl::Buffer cl_shEvalY;
	cl::Buffer cl_shEvalZ;
	cl::Buffer cl_coef0X;
	cl::Buffer cl_coef0Y;
	cl::Buffer cl_coef0Z;
	cl::Buffer cl_goal_in;
	cl::Buffer cl_goal_out;
	cl::Buffer cl_range;
	cl::Buffer cl_gridHash_unsorted;
	cl::Buffer cl_gridHash_sorted;
	cl::Buffer cl_gridIndex_unsorted;
	cl::Buffer cl_gridIndex_sorted;
	cl::Buffer cl_simParams;
	cl::Buffer cl_gridStartIndex;
	cl::Buffer cl_gridEndIndex;
	// sum of velocities
	cl::Buffer cl_sumVel;

	cl_int err;

	std::vector<std::string> attribName;
	Shader* shader;
};


/*
	Draw the boidModelSH with way finding.
*/
class BoidModelSHWay2 : public BoidModel
{
public:
	BoidModelSHWay2(CLHelper* clHlpr, std::vector<Vec4> pos, std::vector<Vec4> vel, std::vector<Vec4> goal, std::vector<Vec4> color, simParams_t* simP);
	~BoidModelSHWay2();

	//inherited from super class BoidModel
	void simulate(float dt);
	GLuint getPosVBO();
	GLuint getVelVBO();
	GLuint getPosVAO();
	int getNumBoid();
	long getSimulationTime();
	std::vector<const char*> getSimTimeDescriptions();
	void getFollowedBoid(unsigned int* boidIndex, Vec4 *pos, Vec4 *vel);

	//inherited from interface Renderable
	void render();
	Shader* getShader();
	void bindShader();
	void unbindShader();

private:
	cl::Program loadProgram(const std::string &filename);
	void loadKernel();
	void createBuffer(std::vector<Vec4> pos, std::vector<Vec4> vel, std::vector<Vec4> goal, std::vector<Vec4> color);
	void loadData(std::vector<Vec4> goal);
	void bitonicSort(cl::Buffer d_DstKey, cl::Buffer d_DstVal, cl::Buffer d_SrcKey, cl::Buffer d_SrcVal, unsigned int batch, unsigned int arrayLength, unsigned int dir);
	void createVboBindShader(std::vector<Vec4> pos, std::vector<Vec4> vel, std::vector<Vec4> color);

	static cl_uint factorRadix2(cl_uint& log2L, cl_uint L);

	int helper = 0;
	GLuint pos_vbo[1];
	GLuint vel_vbo[1];
	GLuint color_vbo[1];
	GLuint pos_vao[1];

	GLuint vel_vbo_out[1];
	GLuint pos_vbo_out[1];
	GLuint color_vbo_out[1];
	GLuint pos_vao_out[1];
	int num;

	bool counter = false;

	std::vector<const char*> simTimeDisc;
	std::string stringSimTime;
	std::string stringHashTime;
	std::string stringSortTime;
	std::string stringEdgeTime;
	//string of the spherical harmonics step
	std::string stringSHTime;
	//string of the reduction of the velocities
	std::string stringSumTime;
	long times[6];

	cl::Context context;
	cl::CommandQueue queue;
	cl::Program programBoid;
	cl::Program programBitonic;
	std::vector<cl::Device> devices;

	cl::Kernel kernel_getGridHash;
	cl::Kernel kernel_getGridEdge;
	cl::Kernel kernel_findGridEdgeAndReorder;
	cl::Kernel kernel_simulate;
	cl::Kernel kernel_bitonicSortLocal;
	cl::Kernel kernel_bitonicSortLocal1;
	cl::Kernel kernel_bitonicMergeGlobal;
	cl::Kernel kernel_bitonicMergeLocal;
	cl::Kernel kernel_memSet;

	//evalute spherical harmonics coefficients (implementation from P.P. Sloan)
	cl::Kernel kernel_SHEval3;
	//simple reduction to get sum of velocities per cell
	cl::Kernel kernel_evalSH;
	//extra step to apply SH to boid simulation
	cl::Kernel kernel_useSH;

	cl::Event event;
	cl::Event eventSim;

	std::vector<cl::Memory> cl_pos_vbos;
	std::vector<cl::Memory> cl_pos_vbos_out;
	std::vector<cl::Memory> cl_vel_vbos;
	std::vector<cl::Memory> cl_vel_vbos_out;
	std::vector<cl::Memory> cl_color_vbos;
	std::vector<cl::Memory> cl_color_vbos_out;

	cl::Buffer cl_shEvalX;
	cl::Buffer cl_shEvalY;
	cl::Buffer cl_shEvalZ;
	cl::Buffer cl_coef0X;
	cl::Buffer cl_coef0Y;
	cl::Buffer cl_coef0Z;
	cl::Buffer cl_goal_in;
	cl::Buffer cl_goal_out;
	cl::Buffer cl_range;
	cl::Buffer cl_gridHash_unsorted;
	cl::Buffer cl_gridHash_sorted;
	cl::Buffer cl_gridIndex_unsorted;
	cl::Buffer cl_gridIndex_sorted;
	cl::Buffer cl_simParams;
	cl::Buffer cl_gridStartIndex;
	cl::Buffer cl_gridEndIndex;
	//sum of velocities
	cl::Buffer cl_sumVel;

	cl_int err;

	std::vector<std::string> attribName;
	Shader* shader;
};

/*SH obstacle avoidance*/
class BoidModelSHObstacle : public BoidModel
{
public:
	BoidModelSHObstacle(CLHelper* clHlpr, std::vector<Vec4> pos, std::vector<Vec4> vel, std::vector<Vec4> goal, simParams_t* simP, std::vector<Vec4> cor, std::vector<unsigned int> start, std::vector<unsigned int> end, std::vector<Vec4> posObst);
	~BoidModelSHObstacle();

	//inherited from super class BoidModel
	void simulate(float dt);
	GLuint getPosVBO();
	GLuint getVelVBO();
	GLuint getPosVAO();
	int getNumBoid();
	long getSimulationTime();
	std::vector<const char*> getSimTimeDescriptions();
	void getFollowedBoid(unsigned int* boidIndex, Vec4 *pos, Vec4 *vel);

	//inherited from interface Renderable
	void render();
	Shader* getShader();
	void bindShader();
	void unbindShader();

private:
	cl::Program loadProgram(const std::string &filename);
	void loadKernel();
	void createBuffer(std::vector<Vec4> pos, std::vector<Vec4> vel, std::vector<Vec4> goal);
	void loadData(std::vector<Vec4> goal);
	void bitonicSort(cl::Buffer d_DstKey, cl::Buffer d_DstVal, cl::Buffer d_SrcKey, cl::Buffer d_SrcVal, unsigned int batch, unsigned int arrayLength, unsigned int dir);
	void createVboBindShader(std::vector<Vec4> pos, std::vector<Vec4> vel);
	void createAndLoadObstacleSH(std::vector<Vec4> cor, std::vector<unsigned int> start, std::vector<unsigned int> end, std::vector<Vec4> posObst);

	static cl_uint factorRadix2(cl_uint& log2L, cl_uint L);

	int helper = 0;
	GLuint pos_vbo[1];
	GLuint vel_vbo[1];
	GLuint pos_vao[1];

	GLuint vel_vbo_out[1];
	GLuint pos_vbo_out[1];
	GLuint pos_vao_out[1];
	int num;

	bool counter = false;

	std::vector<const char*> simTimeDisc;
	std::string stringSimTime;
	std::string stringHashTime;
	std::string stringSortTime;
	std::string stringEdgeTime;
	//string of the spherical harmonics step
	std::string stringSHTime;
	//string of the reduction of the velocities
	std::string stringSumTime;
	long times[6];

	cl::Context context;
	cl::CommandQueue queue;
	cl::Program programBoid;
	cl::Program programBitonic;
	std::vector<cl::Device> devices;

	cl::Kernel kernel_getGridHash;
	cl::Kernel kernel_getGridEdge;
	cl::Kernel kernel_findGridEdgeAndReorder;
	cl::Kernel kernel_simulate;
	cl::Kernel kernel_bitonicSortLocal;
	cl::Kernel kernel_bitonicSortLocal1;
	cl::Kernel kernel_bitonicMergeGlobal;
	cl::Kernel kernel_bitonicMergeLocal;
	cl::Kernel kernel_memSet;

	//evalute spherical harmonics coefficients (implementation from P.P. Sloan)
	cl::Kernel kernel_SHEval3;
	//simple reduction to get sum of velocities per cell
	cl::Kernel kernel_evalSH;
	//extra step to apply SH to boid simulation
	cl::Kernel kernel_useSH;
	//kernel to create SH representation of obstacles
	cl::Kernel kernel_obstacle;

	cl::Event event;
	cl::Event eventSim;

	std::vector<cl::Memory> cl_pos_vbos;
	std::vector<cl::Memory> cl_pos_vbos_out;
	std::vector<cl::Memory> cl_vel_vbos;
	std::vector<cl::Memory> cl_vel_vbos_out;

	cl::Buffer cl_shEvalX;
	cl::Buffer cl_shEvalY;
	cl::Buffer cl_shEvalZ;
	cl::Buffer cl_coef0X;
	cl::Buffer cl_coef0Y;
	cl::Buffer cl_coef0Z;

	cl::Buffer cl_shEvalOX;
	cl::Buffer cl_shEvalOY;
	cl::Buffer cl_shEvalOZ;
	cl::Buffer cl_coef0OX;
	cl::Buffer cl_coef0OY;
	cl::Buffer cl_coef0OZ;
	cl::Buffer cl_cor;
	cl::Buffer cl_startCor;
	cl::Buffer cl_endCor;
	cl::Buffer cl_posObst;

	cl::Buffer cl_goal_in;
	cl::Buffer cl_goal_out;
	cl::Buffer cl_range;
	cl::Buffer cl_gridHash_unsorted;
	cl::Buffer cl_gridHash_sorted;
	cl::Buffer cl_gridIndex_unsorted;
	cl::Buffer cl_gridIndex_sorted;
	cl::Buffer cl_simParams;
	cl::Buffer cl_gridStartIndex;
	cl::Buffer cl_gridEndIndex;
	//sum of velocities
	cl::Buffer cl_sumVel;

	cl_int err;

	std::vector<std::string> attribName;
	Shader* shader;
};


/*Boid model way1 combined with obstacle avoidance*/

class BoidModelSHCombined : public BoidModel
{
public:
	BoidModelSHCombined(CLHelper* clHlpr, std::vector<Vec4> pos, std::vector<Vec4> vel, std::vector<Vec4> goal, std::vector<Vec4> color, simParams_t* simP, std::vector<Vec4> cor, std::vector<unsigned int> start, std::vector<unsigned int> end, std::vector<Vec4> posObst);
	~BoidModelSHCombined();

	//inherited from super class BoidModel
	void simulate(float dt);
	GLuint getPosVBO();
	GLuint getVelVBO();
	GLuint getPosVAO();
	int getNumBoid();
	long getSimulationTime();
	std::vector<const char*> getSimTimeDescriptions();
	void getFollowedBoid(unsigned int* boidIndex, Vec4 *pos, Vec4 *vel);

	//inherited from interface Renderable
	void render();
	Shader* getShader();
	void bindShader();
	void unbindShader();

private:
	cl::Program loadProgram(const std::string &filename);
	void loadKernel();
	void createBuffer(std::vector<Vec4> pos, std::vector<Vec4> vel, std::vector<Vec4> goal, std::vector<Vec4> color);
	void loadData(std::vector<Vec4> goal);
	void bitonicSort(cl::Buffer d_DstKey, cl::Buffer d_DstVal, cl::Buffer d_SrcKey, cl::Buffer d_SrcVal, unsigned int batch, unsigned int arrayLength, unsigned int dir);
	void createVboBindShader(std::vector<Vec4> pos, std::vector<Vec4> vel, std::vector<Vec4> color);
	void createAndLoadObstacleSH(std::vector<Vec4> cor, std::vector<unsigned int> start, std::vector<unsigned int> end, std::vector<Vec4> posObst);

	static cl_uint factorRadix2(cl_uint& log2L, cl_uint L);

	int helper = 0;
	GLuint pos_vbo[1];
	GLuint vel_vbo[1];
	GLuint color_vbo[1];
	GLuint pos_vao[1];

	GLuint vel_vbo_out[1];
	GLuint pos_vbo_out[1];
	GLuint color_vbo_out[1];
	GLuint pos_vao_out[1];
	int num;

	bool counter = false;

	std::vector<const char*> simTimeDisc;
	std::string stringSimTime;
	std::string stringHashTime;
	std::string stringSortTime;
	std::string stringEdgeTime;
	//string of the spherical harmonics step
	std::string stringSHTime;
	//string of the reduction of the velocities
	std::string stringSumTime;
	long times[6];

	cl::Context context;
	cl::CommandQueue queue;
	cl::Program programBoid;
	cl::Program programBitonic;
	std::vector<cl::Device> devices;

	cl::Kernel kernel_getGridHash;
	cl::Kernel kernel_getGridEdge;
	cl::Kernel kernel_findGridEdgeAndReorder;
	cl::Kernel kernel_simulate;
	cl::Kernel kernel_bitonicSortLocal;
	cl::Kernel kernel_bitonicSortLocal1;
	cl::Kernel kernel_bitonicMergeGlobal;
	cl::Kernel kernel_bitonicMergeLocal;
	cl::Kernel kernel_memSet;

	//evalute spherical harmonics coefficients (implementation from P.P. Sloan)
	cl::Kernel kernel_SHEval3;
	//simple reduction to get sum of velocities per cell
	cl::Kernel kernel_evalSH;
	//extra step to apply SH to boid simulation
	cl::Kernel kernel_useSH;
	//kernel to create SH representation of obstacles
	cl::Kernel kernel_obstacle;

	cl::Event event;
	cl::Event eventSim;

	std::vector<cl::Memory> cl_pos_vbos;
	std::vector<cl::Memory> cl_pos_vbos_out;
	std::vector<cl::Memory> cl_vel_vbos;
	std::vector<cl::Memory> cl_vel_vbos_out;
	std::vector<cl::Memory> cl_color_vbos;
	std::vector<cl::Memory> cl_color_vbos_out;

	cl::Buffer cl_shEvalX;
	cl::Buffer cl_shEvalY;
	cl::Buffer cl_shEvalZ;
	cl::Buffer cl_coef0X;
	cl::Buffer cl_coef0Y;
	cl::Buffer cl_coef0Z;

	cl::Buffer cl_shEvalOX;
	cl::Buffer cl_shEvalOY;
	cl::Buffer cl_shEvalOZ;
	cl::Buffer cl_coef0OX;
	cl::Buffer cl_coef0OY;
	cl::Buffer cl_coef0OZ;
	cl::Buffer cl_cor;
	cl::Buffer cl_startCor;
	cl::Buffer cl_endCor;
	cl::Buffer cl_posObst;

	cl::Buffer cl_goal_in;
	cl::Buffer cl_goal_out;
	cl::Buffer cl_range;
	cl::Buffer cl_gridHash_unsorted;
	cl::Buffer cl_gridHash_sorted;
	cl::Buffer cl_gridIndex_unsorted;
	cl::Buffer cl_gridIndex_sorted;
	cl::Buffer cl_simParams;
	cl::Buffer cl_gridStartIndex;
	cl::Buffer cl_gridEndIndex;
	//sum of velocities
	cl::Buffer cl_sumVel;

	cl_int err;

	std::vector<std::string> attribName;
	Shader* shader;
};



/*Tunnel test case*/
class BoidModelSHObstacleTunnel : public BoidModel
{
public:
	BoidModelSHObstacleTunnel(CLHelper* clHlpr, std::vector<Vec4> pos, std::vector<Vec4> vel, std::vector<Vec4> goal, std::vector<Vec4> color, simParams_t* simP, std::vector<Vec4> cor, std::vector<unsigned int> start, std::vector<unsigned int> end, std::vector<Vec4> posObst);
	~BoidModelSHObstacleTunnel();

	//inherited from super class BoidModel
	void simulate(float dt);
	GLuint getPosVBO();
	GLuint getVelVBO();
	GLuint getPosVAO();
	int getNumBoid();
	long getSimulationTime();
	std::vector<const char*> getSimTimeDescriptions();
	void getFollowedBoid(unsigned int* boidIndex, Vec4 *pos, Vec4 *vel);

	//inherited from interface Renderable
	void render();
	Shader* getShader();
	void bindShader();
	void unbindShader();

private:
	cl::Program loadProgram(const std::string &filename);
	void loadKernel();
	void createBuffer(std::vector<Vec4> pos, std::vector<Vec4> vel, std::vector<Vec4> goal, std::vector<Vec4> color);
	void loadData(std::vector<Vec4> goal);
	void bitonicSort(cl::Buffer d_DstKey, cl::Buffer d_DstVal, cl::Buffer d_SrcKey, cl::Buffer d_SrcVal, unsigned int batch, unsigned int arrayLength, unsigned int dir);
	void createVboBindShader(std::vector<Vec4> pos, std::vector<Vec4> vel, std::vector<Vec4> color);
	void createAndLoadObstacleSH(std::vector<Vec4> cor, std::vector<unsigned int> start, std::vector<unsigned int> end, std::vector<Vec4> posObst);

	static cl_uint factorRadix2(cl_uint& log2L, cl_uint L);

	int helper = 0;
	GLuint pos_vbo[1];
	GLuint vel_vbo[1];
	GLuint color_vbo[1];
	GLuint pos_vao[1];

	GLuint vel_vbo_out[1];
	GLuint pos_vbo_out[1];
	GLuint color_vbo_out[1];
	GLuint pos_vao_out[1];
	int num;

	bool counter = false;

	std::vector<const char*> simTimeDisc;
	std::string stringSimTime;
	std::string stringHashTime;
	std::string stringSortTime;
	std::string stringEdgeTime;
	//string of the spherical harmonics step
	std::string stringSHTime;
	//string of the reduction of the velocities
	std::string stringSumTime;
	long times[6];

	cl::Context context;
	cl::CommandQueue queue;
	cl::Program programBoid;
	cl::Program programBitonic;
	std::vector<cl::Device> devices;

	cl::Kernel kernel_getGridHash;
	cl::Kernel kernel_getGridEdge;
	cl::Kernel kernel_findGridEdgeAndReorder;
	cl::Kernel kernel_simulate;
	cl::Kernel kernel_bitonicSortLocal;
	cl::Kernel kernel_bitonicSortLocal1;
	cl::Kernel kernel_bitonicMergeGlobal;
	cl::Kernel kernel_bitonicMergeLocal;
	cl::Kernel kernel_memSet;

	//evalute spherical harmonics coefficients (implementation from P.P. Sloan)
	cl::Kernel kernel_SHEval3;
	//simple reduction to get sum of velocities per cell
	cl::Kernel kernel_evalSH;
	//extra step to apply SH to boid simulation
	cl::Kernel kernel_useSH;
	//kernel to create SH representation of obstacles
	cl::Kernel kernel_obstacle;

	cl::Event event;
	cl::Event eventSim;

	std::vector<cl::Memory> cl_pos_vbos;
	std::vector<cl::Memory> cl_pos_vbos_out;
	std::vector<cl::Memory> cl_vel_vbos;
	std::vector<cl::Memory> cl_vel_vbos_out;
	std::vector<cl::Memory> cl_color_vbos;
	std::vector<cl::Memory> cl_color_vbos_out;

	cl::Buffer cl_shEvalX;
	cl::Buffer cl_shEvalY;
	cl::Buffer cl_shEvalZ;
	cl::Buffer cl_coef0X;
	cl::Buffer cl_coef0Y;
	cl::Buffer cl_coef0Z;

	cl::Buffer cl_shEvalOX;
	cl::Buffer cl_shEvalOY;
	cl::Buffer cl_shEvalOZ;
	cl::Buffer cl_coef0OX;
	cl::Buffer cl_coef0OY;
	cl::Buffer cl_coef0OZ;
	cl::Buffer cl_cor;
	cl::Buffer cl_startCor;
	cl::Buffer cl_endCor;
	cl::Buffer cl_posObst;

	cl::Buffer cl_goal_in;
	cl::Buffer cl_goal_out;
	cl::Buffer cl_range;
	cl::Buffer cl_gridHash_unsorted;
	cl::Buffer cl_gridHash_sorted;
	cl::Buffer cl_gridIndex_unsorted;
	cl::Buffer cl_gridIndex_sorted;
	cl::Buffer cl_simParams;
	cl::Buffer cl_gridStartIndex;
	cl::Buffer cl_gridEndIndex;
	//sum of velocities
	cl::Buffer cl_sumVel;

	cl_int err;

	std::vector<std::string> attribName;
	Shader* shader;
};

#endif //_BOIDMODEL_H_