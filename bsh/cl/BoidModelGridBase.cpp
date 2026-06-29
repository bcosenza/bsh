// Copyright (c) 2015, Biagio Cosenza.
// Technische Universitaet Berlin. All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#include "common.h"
#include "BoidModel.h"

BoidModelGridBase::BoidModelGridBase(CLHelper* clHlpr, simParams_t* simP, const char* name, bool dblBuffered)
	: BoidModel(clHlpr)
{
	modelName = name;
	doubleBuffered = dblBuffered;
	counter = false;
	shader = NULL;

	log("start setup - " + std::string(modelName));

	context = clHelper->getContext();
	queue = clHelper->getCmdQueue();
	devices = clHelper->getDevices();

	simParams = *simP;
	num = simParams.numBodies;

	for (int i = 0; i < 8; i++)
		times[i] = 0;

	addTimingRow("Grid hash time", 0);
	addTimingRow("Sorting time", 1);
	addTimingRow("Edge detect./reorder time", 2);
	addTimingRow("Simulation time", 3);
}

BoidModelGridBase::~BoidModelGridBase(){
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glDeleteBuffers(1, pos_vbo);
	glBindVertexArray(0);
	glDeleteVertexArrays(1, pos_vao);

	delete shader;
}

// ---- Renderable ----

void BoidModelGridBase::render(){
	shader->bind();
	glBindVertexArray(getPosVAO());
	glDrawArrays(GL_POINTS, 0, num);	//draw boids as points
	glBindVertexArray(0);
	shader->unbind();
}

Shader* BoidModelGridBase::getShader(){
	return shader;
}

void BoidModelGridBase::bindShader(){
	shader->bind();
}

void BoidModelGridBase::unbindShader(){
	shader->unbind();
}

// ---- BoidModel ----

GLuint BoidModelGridBase::getPosVBO(){
	return useInBuffers() ? pos_vbo[0] : pos_vbo_out[0];
}

GLuint BoidModelGridBase::getVelVBO(){
	return useInBuffers() ? vel_vbo[0] : vel_vbo_out[0];
}

GLuint BoidModelGridBase::getPosVAO(){
	return useInBuffers() ? pos_vao[0] : pos_vao_out[0];
}

int BoidModelGridBase::getNumBoid(){
	return num;
}

long BoidModelGridBase::getSimulationTime(){
	return times[3] / 1000;
}

void BoidModelGridBase::addTimingRow(const char* label, int timesIndex){
	timingRows.push_back(std::make_pair(label, timesIndex));
}

std::vector<const char*> BoidModelGridBase::getSimTimeDescriptions(){
	timingStrings.resize(timingRows.size());
	simTimeDisc.assign(4 + timingRows.size(), "");
	simTimeDisc[0] = modelName;
	simTimeDisc[1] = "OpenCL Simulation Times:";
	// simTimeDisc[3] is filled with the total time by the Simulation

	for (size_t i = 0; i < timingRows.size(); i++){
		std::stringstream strstream;
		strstream << timingRows[i].first << ": " << times[timingRows[i].second] / 1000.0 << "ms";
		timingStrings[i] = strstream.str();
		simTimeDisc[4 + i] = timingStrings[i].c_str();
	}

	return simTimeDisc;
}

void BoidModelGridBase::getFollowedBoid(unsigned int* boidIndex, Vec4* pos, Vec4* vel){
	size_t size = sizeof(unsigned int) * num;
	std::vector<unsigned int> sortedHash(num);
	queue.enqueueReadBuffer(cl_gridIndex_sorted, CL_TRUE, 0, size, sortedHash.data());
	queue.finish();

	for (int i = 0; i < num; i++){
		if (sortedHash[i] == *boidIndex){
			*boidIndex = i;
			break;
		}
	}

	Vec4 v;
	GLuint vbo = getVelVBO();
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glGetBufferSubData(GL_ARRAY_BUFFER, sizeof(Vec4) * *boidIndex, sizeof(Vec4), &v);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	(*vel).set(v.x, v.y, v.z, 0.0);

	Vec4 p;
	vbo = getPosVBO();
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glGetBufferSubData(GL_ARRAY_BUFFER, sizeof(Vec4) * *boidIndex, sizeof(Vec4), &p);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	(*pos).set(p.x, p.y, p.z, 0.0);
}

// ---- setup ----

std::string BoidModelGridBase::readKernelFile(const std::string &filename){
	std::string kernelSource;

	std::ifstream in(filename, std::ios::in | std::ios::binary);
	if (in)
	{
		in.seekg(0, std::ios::end);
		kernelSource.resize(in.tellg());
		in.seekg(0, std::ios::beg);
		in.read(&kernelSource[0], kernelSource.size());
		in.close();
	}
	else
	{
		log("could not open " + filename);
		throw(errno);
	}

	return kernelSource;
}

cl::Program BoidModelGridBase::loadBoidProgram(const std::string &filename){
	// the shared declarations and kernels are prepended to every boid program
	return buildProgram(readKernelFile(kernel_path + "common.cl") + "\n" + readKernelFile(filename));
}

cl::Program BoidModelGridBase::loadProgram(const std::string &filename){
	return buildProgram(readKernelFile(filename));
}

cl::Program BoidModelGridBase::buildProgram(const std::string &kernelSource){
	log("load program");

	cl::Program program;
	try
	{
		cl::Program::Sources source;
		source.push_back(kernelSource);
		program = cl::Program(context, source);
	}
	catch (cl::Error er)
	{
		log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
	}

	try
	{
		err = program.build(devices);
	}
	catch (cl::Error er) {
		log("program build: " + clHelper->oclErrorString(er.err()));
		log("\n----------------------buildLog start--------------------\n");
		std::string buildLog = program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(devices[0]);
		log(buildLog);
		log("\n----------------------buildLog end--------------------\n");
	}

	return program;
}

void BoidModelGridBase::loadCommonKernels(){
	log("loading kernels");
	try{
		kernel_getGridHash = cl::Kernel(programBoid, "getGridHash", &err);
		kernel_findGridEdgeAndReorder = cl::Kernel(programBoid, "findGridEdgeAndReorder", &err);
		kernel_simulate = cl::Kernel(programBoid, "simulate", &err);
		kernel_memSet = cl::Kernel(programBoid, "memSet", &err);
		kernel_bitonicSortLocal = cl::Kernel(programBitonic, "bitonicSortLocal", &err);
		kernel_bitonicSortLocal1 = cl::Kernel(programBitonic, "bitonicSortLocal1", &err);
		kernel_bitonicMergeGlobal = cl::Kernel(programBitonic, "bitonicMergeGlobal", &err);
		kernel_bitonicMergeLocal = cl::Kernel(programBitonic, "bitonicMergeLocal", &err);
	}
	catch (cl::Error er) {
		log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
	}
}

void BoidModelGridBase::createVboSet(const std::vector<Vec4>& pos, const std::vector<Vec4>& vel,
	const std::vector<Vec4>& color, bool reorderedColor,
	GLuint* vao, GLuint* posVbo, GLuint* velVbo, GLuint* colorVbo){

	size_t array_size = num * sizeof(Vec4);

	GLint vertLoc = glGetAttribLocation(shader->id(), "coord3d");
	GLint colorLoc = glGetAttribLocation(shader->id(), "color");
	GLint velLoc = glGetAttribLocation(shader->id(), "vel3d");

	glGenVertexArrays(1, vao);
	glBindVertexArray(*vao);

	*posVbo = clHelper->createVBO(&pos[0], array_size, GL_ARRAY_BUFFER, GL_DYNAMIC_DRAW);
	glVertexAttribPointer(vertLoc, 4, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(vertLoc);

	*velVbo = clHelper->createVBO(&vel[0], array_size, GL_ARRAY_BUFFER, GL_DYNAMIC_DRAW);
	glVertexAttribPointer(velLoc, 4, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(velLoc);

	if (reorderedColor){
		*colorVbo = clHelper->createVBO(&color[0], array_size, GL_ARRAY_BUFFER, GL_DYNAMIC_DRAW);
	}
	else {
		glGenBuffers(1, colorVbo);
		glBindBuffer(GL_ARRAY_BUFFER, *colorVbo);
		glBufferData(GL_ARRAY_BUFFER, array_size, &color[0], GL_STATIC_DRAW);
	}
	glVertexAttribPointer(colorLoc, 4, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(colorLoc);

	glEnableVertexAttribArray(0);
	glBindVertexArray(0);
}

void BoidModelGridBase::createCommonBuffers(const std::vector<Vec4>& pos, const std::vector<Vec4>& vel,
	const std::vector<Vec4>& color, bool reorderedColor){
	log("Create buffer for usage");

	size_t array_size_simple = num * sizeof(unsigned int);
	size_t array_size_edges = simParams.numCells * sizeof(unsigned int);

	std::vector<Vec4> boidColor = color;
	if (boidColor.empty())
		boidColor.assign(num, BOID_COLOR);

	shader = new Shader("boidTri.v.glsl", "boidTri.f.glsl", "boidTri.g.glsl");

	createVboSet(pos, vel, boidColor, reorderedColor, &pos_vao[0], &pos_vbo[0], &vel_vbo[0], &color_vbo[0]);
	if (doubleBuffered)
		createVboSet(pos, vel, boidColor, reorderedColor, &pos_vao_out[0], &pos_vbo_out[0], &vel_vbo_out[0], &color_vbo_out[0]);

	log("GL VBO Buffer created");

	try{
		// create OpenCL buffers from the GL VBOs
		cl_pos_vbos.push_back(clHelper->createFromGLBuffer(pos_vbo[0], &err));
		cl_vel_vbos.push_back(clHelper->createFromGLBuffer(vel_vbo[0], &err));
		if (doubleBuffered){
			cl_pos_vbos_out.push_back(clHelper->createFromGLBuffer(pos_vbo_out[0], &err));
			cl_vel_vbos_out.push_back(clHelper->createFromGLBuffer(vel_vbo_out[0], &err));
		}
		if (reorderedColor){
			cl_color_vbos.push_back(clHelper->createFromGLBuffer(color_vbo[0], &err));
			cl_color_vbos_out.push_back(clHelper->createFromGLBuffer(color_vbo_out[0], &err));
		}

		//create the OpenCL only arrays
		cl_gridHash_unsorted = cl::Buffer(context, CL_MEM_READ_WRITE, array_size_simple, NULL, &err);
		cl_gridHash_sorted = cl::Buffer(context, CL_MEM_READ_WRITE, array_size_simple, NULL, &err);
		cl_gridIndex_sorted = cl::Buffer(context, CL_MEM_READ_WRITE, array_size_simple, NULL, &err);
		cl_gridIndex_unsorted = cl::Buffer(context, CL_MEM_READ_WRITE, array_size_simple, NULL, &err);
		cl_gridStartIndex = cl::Buffer(context, CL_MEM_READ_WRITE, array_size_edges, NULL, &err);
		cl_gridEndIndex = cl::Buffer(context, CL_MEM_READ_WRITE, array_size_edges, NULL, &err);
		cl_range = cl::Buffer(context, CL_MEM_READ_WRITE, array_size_edges, NULL, &err);
		cl_simParams = cl::Buffer(context, CL_MEM_READ_ONLY, sizeof(simParams_t), NULL, &err);
	}
	catch (cl::Error er) {
		log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
	}
}

void BoidModelGridBase::createGoalBuffers(const std::vector<Vec4>& goal){
	size_t array_size_fp4 = num * sizeof(Vec4);

	try{
		cl_goal_in = cl::Buffer(context, CL_MEM_READ_WRITE, array_size_fp4, NULL, &err);
		cl_goal_out = cl::Buffer(context, CL_MEM_READ_WRITE, array_size_fp4, NULL, &err);
	}
	catch (cl::Error er) {
		log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
	}

	err = queue.enqueueWriteBuffer(cl_goal_in, CL_TRUE, 0, array_size_fp4, &goal[0], NULL, &event);
	queue.finish();
}

void BoidModelGridBase::uploadSimParams(){
	err = queue.enqueueWriteBuffer(cl_simParams, CL_TRUE, 0, sizeof(simParams_t), &simParams, NULL, &event);
	queue.finish();
}

// ---- per frame pipeline ----

long BoidModelGridBase::eventTimeUs(cl::Event& e){
	cl_ulong startTime, endTime;
	e.wait();
	e.getProfilingInfo<cl_ulong>(CL_PROFILING_COMMAND_START, &startTime);
	e.getProfilingInfo<cl_ulong>(CL_PROFILING_COMMAND_END, &endTime);
	return (long)((endTime - startTime) / 1000);
}

void BoidModelGridBase::acquireGLBuffers(){
	//Make sure OpenGL is done using our VBOs
	glFinish();
	// map the GL buffer objects for writing from OpenCL
	err = clHelper->acquireGLObjects(queue, &cl_pos_vbos, &event);
	err = clHelper->acquireGLObjects(queue, &cl_vel_vbos, &event);
	if (doubleBuffered){
		err = clHelper->acquireGLObjects(queue, &cl_pos_vbos_out, &event);
		err = clHelper->acquireGLObjects(queue, &cl_vel_vbos_out, &event);
	}
	if (!cl_color_vbos.empty()){
		err = clHelper->acquireGLObjects(queue, &cl_color_vbos, &event);
		err = clHelper->acquireGLObjects(queue, &cl_color_vbos_out, &event);
	}
	queue.finish();
}

void BoidModelGridBase::releaseGLBuffers(){
	//Release the VBOs so OpenGL can play with them
	err = clHelper->releaseGLObjects(queue, &cl_pos_vbos, &event);
	err = clHelper->releaseGLObjects(queue, &cl_vel_vbos, &event);
	if (doubleBuffered){
		err = clHelper->releaseGLObjects(queue, &cl_pos_vbos_out, &event);
		err = clHelper->releaseGLObjects(queue, &cl_vel_vbos_out, &event);
	}
	if (!cl_color_vbos.empty()){
		err = clHelper->releaseGLObjects(queue, &cl_color_vbos, &event);
		err = clHelper->releaseGLObjects(queue, &cl_color_vbos_out, &event);
	}
}

void BoidModelGridBase::runGridHash(const cl::Memory& posIn){
	try
	{
		err = kernel_getGridHash.setArg(0, posIn);
		err = kernel_getGridHash.setArg(1, cl_gridHash_unsorted);
		err = kernel_getGridHash.setArg(2, cl_gridIndex_unsorted);
		err = kernel_getGridHash.setArg(3, cl_simParams);
	}
	catch (cl::Error er) {
		log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
	}

	err = queue.enqueueNDRangeKernel(kernel_getGridHash, cl::NullRange, cl::NDRange(num), cl::NullRange, NULL, &event);
	times[0] = eventTimeUs(event);
}

void BoidModelGridBase::clearGridEdges(){
	unsigned int val = 0;
	try
	{
		err = kernel_memSet.setArg(0, cl_gridStartIndex);
		err = kernel_memSet.setArg(1, val);
		err = kernel_memSet.setArg(2, simParams.numCells);
		err = queue.enqueueNDRangeKernel(kernel_memSet, cl::NullRange, cl::NDRange(simParams.numCells), cl::NullRange, NULL, &event);
		queue.finish();

		err = kernel_memSet.setArg(0, cl_gridEndIndex);
		err = queue.enqueueNDRangeKernel(kernel_memSet, cl::NullRange, cl::NDRange(simParams.numCells), cl::NullRange, NULL, &event);
		queue.finish();
	}
	catch (cl::Error er) {
		log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
	}
}

void BoidModelGridBase::sortGridHash(){
	bitonicSort(cl_gridHash_sorted, cl_gridIndex_sorted, cl_gridHash_unsorted, cl_gridIndex_unsorted, 1, simParams.numBodies, 0);
	queue.finish();
}

void BoidModelGridBase::runReorder(){
	try
	{
		err = kernel_findGridEdgeAndReorder.setArg(0, cl_gridStartIndex);
		err = kernel_findGridEdgeAndReorder.setArg(1, cl_gridEndIndex);
		err = kernel_findGridEdgeAndReorder.setArg(4, cl_gridHash_sorted);
		err = kernel_findGridEdgeAndReorder.setArg(5, cl_gridIndex_sorted);

		// the local hash cache and boid count are the two trailing arguments,
		// after the model specific buffers set by the caller
		cl_uint numArgs = kernel_findGridEdgeAndReorder.getInfo<CL_KERNEL_NUM_ARGS>();
		err = kernel_findGridEdgeAndReorder.setArg(numArgs - 2, cl::Local(sizeof(cl_uint)*(LOCAL_PREF + 1))); //one bigger because of border case
		err = kernel_findGridEdgeAndReorder.setArg(numArgs - 1, num);
	}
	catch (cl::Error er) {
		log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
	}

	err = queue.enqueueNDRangeKernel(kernel_findGridEdgeAndReorder, cl::NullRange, cl::NDRange(num), cl::NDRange(LOCAL_PREF), NULL, &event);
	times[2] = eventTimeUs(event);
}

// ---- bitonic sort (adapted from the NVIDIA OpenCL samples) ----

void BoidModelGridBase::bitonicSort(
	cl::Buffer d_DstKey,
	cl::Buffer d_DstVal,
	cl::Buffer d_SrcKey,
	cl::Buffer d_SrcVal,
	unsigned int batch,
	unsigned int arrayLength,
	unsigned int dir
	){

	if (arrayLength < 2)
		return;

	//Only power-of-two array lengths are supported so far
	cl_uint log2L;
	cl_uint factorizationRemainder = factorRadix2(log2L, arrayLength);

	if (factorizationRemainder != 1){
		log("Array not a power of two");
		return;
	}

	dir = (dir != 0);

	size_t localWorkSize, globalWorkSize;

	unsigned long long timeNow = GetTickCount64();

	if (arrayLength <= LOCAL_SIZE_LIMIT)
	{
		try
		{
			err = kernel_bitonicSortLocal.setArg(0, d_DstKey);
			err = kernel_bitonicSortLocal.setArg(1, d_DstVal);
			err = kernel_bitonicSortLocal.setArg(2, d_SrcKey);
			err = kernel_bitonicSortLocal.setArg(3, d_SrcVal);
			err = kernel_bitonicSortLocal.setArg(4, arrayLength);
			err = kernel_bitonicSortLocal.setArg(5, dir);
		}
		catch (cl::Error er) {
			log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
		}

		localWorkSize = LOCAL_SIZE_LIMIT / 2;
		globalWorkSize = batch * arrayLength / 2;

		err = queue.enqueueNDRangeKernel(kernel_bitonicSortLocal, cl::NullRange, cl::NDRange(globalWorkSize), cl::NDRange(localWorkSize), NULL, NULL);
		queue.finish();
	}
	else
	{
		try
		{
			err = kernel_bitonicSortLocal1.setArg(0, d_DstKey);
			err = kernel_bitonicSortLocal1.setArg(1, d_DstVal);
			err = kernel_bitonicSortLocal1.setArg(2, d_SrcKey);
			err = kernel_bitonicSortLocal1.setArg(3, d_SrcVal);
		}
		catch (cl::Error er) {
			log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
		}

		localWorkSize = LOCAL_SIZE_LIMIT / 2;
		globalWorkSize = batch * arrayLength / 2;
		err = queue.enqueueNDRangeKernel(kernel_bitonicSortLocal1, cl::NullRange, cl::NDRange(globalWorkSize), cl::NDRange(localWorkSize), NULL, NULL);
		queue.finish();

		for (unsigned int size = 2 * LOCAL_SIZE_LIMIT; size <= arrayLength; size <<= 1)
		{
			for (unsigned stride = size / 2; stride > 0; stride >>= 1)
			{
				if (stride >= LOCAL_SIZE_LIMIT)
				{
					localWorkSize = LOCAL_SIZE_LIMIT / 4;
					globalWorkSize = batch * arrayLength / 2;
					//Launch bitonicMergeGlobal
					try
					{
						err = kernel_bitonicMergeGlobal.setArg(0, d_DstKey);
						err = kernel_bitonicMergeGlobal.setArg(1, d_DstVal);
						err = kernel_bitonicMergeGlobal.setArg(2, d_DstKey);
						err = kernel_bitonicMergeGlobal.setArg(3, d_DstVal);
						err = kernel_bitonicMergeGlobal.setArg(4, arrayLength);
						err = kernel_bitonicMergeGlobal.setArg(5, size);
						err = kernel_bitonicMergeGlobal.setArg(6, stride);
						err = kernel_bitonicMergeGlobal.setArg(7, dir);
					}
					catch (cl::Error er) {
						log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
					}

					err = queue.enqueueNDRangeKernel(kernel_bitonicMergeGlobal, cl::NullRange, cl::NDRange(globalWorkSize), cl::NDRange(localWorkSize), NULL, NULL);
					queue.finish();
				}
				else
				{
					//Launch bitonicMergeLocal
					localWorkSize = LOCAL_SIZE_LIMIT / 2;
					globalWorkSize = batch * arrayLength / 2;

					try
					{
						err = kernel_bitonicMergeLocal.setArg(0, d_DstKey);
						err = kernel_bitonicMergeLocal.setArg(1, d_DstVal);
						err = kernel_bitonicMergeLocal.setArg(2, d_DstKey);
						err = kernel_bitonicMergeLocal.setArg(3, d_DstVal);
						err = kernel_bitonicMergeLocal.setArg(4, arrayLength);
						err = kernel_bitonicMergeLocal.setArg(5, stride);
						err = kernel_bitonicMergeLocal.setArg(6, size);
						err = kernel_bitonicMergeLocal.setArg(7, dir);
					}
					catch (cl::Error er) {
						log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
					}

					err = queue.enqueueNDRangeKernel(kernel_bitonicMergeLocal, cl::NullRange, cl::NDRange(globalWorkSize), cl::NDRange(localWorkSize), NULL, NULL);
					queue.finish();
					break;
				}
			}
		}
	}

	times[1] = (long)(GetTickCount64() - timeNow) * 1000;	//ms -> us
}

cl_uint BoidModelGridBase::factorRadix2(cl_uint& log2L, cl_uint L){
	if (!L){
		log2L = 0;
		return 0;
	}
	else{
		for (log2L = 0; (L & 1) == 0; L >>= 1, log2L++);
		return L;
	}
}
