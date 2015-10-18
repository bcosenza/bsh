#include "stdafx.h"
#include "boidModel.h"

BoidModelSH_2D::BoidModelSH_2D(CLHelper* clHlpr, std::vector<Vec4> pos, std::vector<Vec4> vel, simParams_t* simP) : BoidModel(clHlpr)
{
	simTimeDisc = std::vector<const char*>(10);
	simTimeDisc[0] = "Boid Model SH 2D";
	simTimeDisc[1] = "OpenCL Simulation Times:";
	simTimeDisc[2] = "";
	simTimeDisc[3] = "";
	simTimeDisc[4] = "";
	simTimeDisc[5] = "";
	simTimeDisc[6] = "";
	simTimeDisc[7] = "";
	simTimeDisc[8] = "";
	simTimeDisc[9] = "";

	context = clHelper->getContext();
	queue = clHelper->getCmdQueue();
	devices = clHelper->getDevices();

	simParams = *simP;

	num = simParams.numBodies;
	Y_AxisFixed = CELL_SIZE_Y / 2;

	for (int i = 0; i < pos.size(); i++){
		pos[i].y = Y_AxisFixed;
		vel[i].y = 0.0f;
	}

	createBuffer(pos, vel);
	loadData();

	programBoid    = loadProgram(kernel_path + "boidModelSH_2D_kernel_v1.cl");
	programBitonic = loadProgram(kernel_path + "bitonic_sort.cl");

	loadKernel();
	log("setup complete - simulation is runable");
}

BoidModelSH_2D::~BoidModelSH_2D(){
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glDeleteBuffers(1, pos_vbo);

	glBindVertexArray(0);
	glDeleteVertexArrays(1, pos_vao);

	delete shader;
}

void BoidModelSH_2D::render(){
	shader->bind();
	glBindVertexArray(getPosVAO());
	glDrawArrays(GL_POINTS, 0, num);
	glBindVertexArray(0);
	shader->unbind();
}

Shader* BoidModelSH_2D::getShader(){
	return shader;
}

void BoidModelSH_2D::simulate(float dt){
	counter = !counter;

	cl_ulong startTime, endTime;
	//this will update our system by calculating new velocity and updating the positions of our particles
	//Make sure OpenGL is done using our VBOs
	glFinish();
	// map OpenGL buffer object for writing from OpenCL
	//this passes in the vector of VBO buffer objects (position and color)
	err = queue.enqueueAcquireGLObjects(&cl_pos_vbos, NULL, &event);
	err = queue.enqueueAcquireGLObjects(&cl_pos_vbos_out, NULL, &event);
	err = queue.enqueueAcquireGLObjects(&cl_vel_vbos, NULL, &event);
	err = queue.enqueueAcquireGLObjects(&cl_vel_vbos_out, NULL, &event);
	queue.finish();

	//Get grid hash value for every boid
	try
	{
		if (counter)
			err = kernel_getGridHash.setArg(0, cl_pos_vbos[0]);
		else
			err = kernel_getGridHash.setArg(0, cl_pos_vbos_out[0]); //position vbo

		err = kernel_getGridHash.setArg(1, cl_gridHash_unsorted);
		err = kernel_getGridHash.setArg(2, cl_gridIndex_unsorted);
		err = kernel_getGridHash.setArg(3, cl_simParams);
	}
	catch (cl::Error er) {
		log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
	}

	//std::vector<Vec4> test(num);
	//glGetBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(Vec4)* num, test.data());

	//create gridHash
	err = queue.enqueueNDRangeKernel(kernel_getGridHash, cl::NullRange, cl::NDRange(num), cl::NullRange, NULL, &event);
	queue.finish();

	event.wait();
	event.getProfilingInfo<cl_ulong>(CL_PROFILING_COMMAND_START, &startTime);
	event.getProfilingInfo<cl_ulong>(CL_PROFILING_COMMAND_END, &endTime);
	times[0] = (endTime - startTime) / 1000;

	//set start and end index to 0
	unsigned int val = 0;
	try
	{
		err = kernel_memSet.setArg(0, cl_gridStartIndex);
		err = kernel_memSet.setArg(1, val);
		err = kernel_memSet.setArg(2, simParams.numCells);
	}
	catch (cl::Error er) {
		log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
	}

	err = queue.enqueueNDRangeKernel(kernel_memSet, cl::NullRange, cl::NDRange(simParams.numCells), cl::NullRange, NULL, &event);
	queue.finish();

	try
	{
		err = kernel_memSet.setArg(0, cl_gridEndIndex);
		err = kernel_memSet.setArg(1, val);
		err = kernel_memSet.setArg(2, simParams.numCells);
	}
	catch (cl::Error er) {
		log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
	}

	err = queue.enqueueNDRangeKernel(kernel_memSet, cl::NullRange, cl::NDRange(simParams.numCells), cl::NullRange, NULL, &event);
	queue.finish();

	//sort gridHash
	bitonicSort(cl_gridHash_sorted, cl_gridIndex_sorted, cl_gridHash_unsorted, cl_gridIndex_unsorted, 1, simParams.numBodies, 0);
	queue.finish();



	//unsigned int E[NUM_BOIDS];
	//queue.enqueueReadBuffer(cl_gridHash_sorted, CL_TRUE, 0, (size_t)(NUM_BOIDS * sizeof(unsigned int)), &E);
	//queue.finish();

	try
	{
		if (counter){
			err = kernel_findGridEdgeAndReorder.setArg(2, cl_pos_vbos_out[0]);	//pos out ordered
			err = kernel_findGridEdgeAndReorder.setArg(3, cl_vel_vbos_out[0]);	//vel out ordered
			err = kernel_findGridEdgeAndReorder.setArg(6, cl_pos_vbos[0]);		//pos in unordered
			err = kernel_findGridEdgeAndReorder.setArg(7, cl_vel_vbos[0]);		//vel in unordered
		}
		else {
			err = kernel_findGridEdgeAndReorder.setArg(6, cl_pos_vbos_out[0]);	//pos in
			err = kernel_findGridEdgeAndReorder.setArg(7, cl_vel_vbos_out[0]);	//vel in
			err = kernel_findGridEdgeAndReorder.setArg(2, cl_pos_vbos[0]);		//pos out
			err = kernel_findGridEdgeAndReorder.setArg(3, cl_vel_vbos[0]);		//vel out
		}

		err = kernel_findGridEdgeAndReorder.setArg(0, cl_gridStartIndex);
		err = kernel_findGridEdgeAndReorder.setArg(1, cl_gridEndIndex);
		err = kernel_findGridEdgeAndReorder.setArg(4, cl_gridHash_sorted);
		err = kernel_findGridEdgeAndReorder.setArg(5, cl_gridIndex_sorted);
		err = kernel_findGridEdgeAndReorder.setArg(8, cl::__local(sizeof(cl_uint)*(LOCAL_PREF + 1)));
		err = kernel_findGridEdgeAndReorder.setArg(9, num);
	}
	catch (cl::Error er) {
		log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
	}

	err = queue.enqueueNDRangeKernel(kernel_findGridEdgeAndReorder, cl::NullRange, cl::NDRange(num), cl::NDRange(LOCAL_PREF), NULL, &event);


	unsigned int F[8000];
	queue.enqueueReadBuffer(cl_gridStartIndex, CL_TRUE, 0, (size_t)(8000 * sizeof(unsigned int)), &F);
	queue.finish();

	unsigned int G[8000];
	queue.enqueueReadBuffer(cl_gridEndIndex, CL_TRUE, 0, (size_t)(8000 * sizeof(unsigned int)), &G);
	queue.finish();

	queue.finish();
	event.wait();
	event.getProfilingInfo<cl_ulong>(CL_PROFILING_COMMAND_START, &startTime);
	event.getProfilingInfo<cl_ulong>(CL_PROFILING_COMMAND_END, &endTime);
	times[2] = (endTime - startTime) / 1000000;

	try
	{
		if (counter)
			err = kernel_sumVelSH.setArg(0, cl_vel_vbos_out[0]);
		else
			err = kernel_sumVelSH.setArg(0, cl_vel_vbos[0]);

		err = kernel_sumVelSH.setArg(1, cl_gridStartIndex);
		err = kernel_sumVelSH.setArg(2, cl_gridEndIndex);
		err = kernel_sumVelSH.setArg(3, cl_sumVel);
		err = kernel_sumVelSH.setArg(4, cl::__local(sizeof(cl_float4)*(2 * LOCAL_PREF)));
	}
	catch (cl::Error er){
		log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
	}



	int localWorkSize = LOCAL_PREF;
	int globalWorkSize = LOCAL_PREF * (simParams.numCells);
	err = queue.enqueueNDRangeKernel(kernel_sumVelSH, cl::NullRange, cl::NDRange(globalWorkSize), cl::NDRange(localWorkSize), NULL, &event);

	event.wait();
	event.getProfilingInfo<cl_ulong>(CL_PROFILING_COMMAND_START, &startTime);
	event.getProfilingInfo<cl_ulong>(CL_PROFILING_COMMAND_END, &endTime);
	times[4] = (endTime - startTime) / 1000000;

	//	std::vector<Vec4> C(simParams.numCells);
	//	queue.enqueueReadBuffer(cl_sumVel, CL_TRUE, 0, (size_t)simParams.numCells * sizeof(Vec4), C.data());
	//	queue.finish(); 

	try
	{
		if (counter){
			err = kernel_simulate.setArg(0, cl_pos_vbos_out[0]);	//pos in
			err = kernel_simulate.setArg(1, cl_pos_vbos[0]);		//pos out
			err = kernel_simulate.setArg(2, cl_vel_vbos_out[0]);	//vel in
			err = kernel_simulate.setArg(3, cl_vel_vbos[0]);		//vel out
		}
		else{
			err = kernel_simulate.setArg(1, cl_pos_vbos_out[0]);	//pos out
			err = kernel_simulate.setArg(0, cl_pos_vbos[0]);		//pos in
			err = kernel_simulate.setArg(3, cl_vel_vbos_out[0]);	//vel out
			err = kernel_simulate.setArg(2, cl_vel_vbos[0]);		//vel in
		}

		err = kernel_simulate.setArg(4, cl_gridStartIndex);
		err = kernel_simulate.setArg(5, cl_gridEndIndex);
		err = kernel_simulate.setArg(6, cl::__local(sizeof(cl_float4)*(LOCAL_PREF)));
		err = kernel_simulate.setArg(7, cl::__local(sizeof(cl_float4)*(LOCAL_PREF)));
		err = kernel_simulate.setArg(8, cl_simParams);
		err = kernel_simulate.setArg(9, cl_range);
		err = kernel_simulate.setArg(10, dt);
	}
	catch (cl::Error er){
		log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
	}

	queue.finish();


	/*
	unsigned int D[12500];
	queue.enqueueReadBuffer(cl_gridEndIndex, CL_TRUE, 0, (size_t)12500 * sizeof(unsigned int), &D);
	queue.finish();*/



	queue.finish();

	localWorkSize = LOCAL_PREF;
	globalWorkSize = LOCAL_PREF * (simParams.numCells);
	err = queue.enqueueNDRangeKernel(kernel_simulate, cl::NullRange, cl::NDRange(globalWorkSize), cl::NDRange(localWorkSize), NULL, &eventSim);

	eventSim.wait();
	eventSim.getProfilingInfo<cl_ulong>(CL_PROFILING_COMMAND_START, &startTime);
	eventSim.getProfilingInfo<cl_ulong>(CL_PROFILING_COMMAND_END, &endTime);
	times[3] = (endTime - startTime) / 1000000;

	try
	{
		if (counter){
			err = kernel_useSH.setArg(0, cl_vel_vbos[0]);		//vel in
			err = kernel_useSH.setArg(1, cl_vel_vbos_out[0]);	//vel out
			err = kernel_useSH.setArg(6, cl_pos_vbos[0]);		//pos in	
			err = kernel_useSH.setArg(7, cl_pos_vbos_out[0]);	//pos out
		}
		else {
			err = kernel_useSH.setArg(1, cl_vel_vbos[0]);		//vel out
			err = kernel_useSH.setArg(0, cl_vel_vbos_out[0]);	//vel in
			err = kernel_useSH.setArg(7, cl_pos_vbos[0]);		//pos out
			err = kernel_useSH.setArg(6, cl_pos_vbos_out[0]);	//pos in
		}

		err = kernel_useSH.setArg(2, cl_gridStartIndex);
		err = kernel_useSH.setArg(3, cl_gridEndIndex);
		err = kernel_useSH.setArg(4, cl_sumVel);
		err = kernel_useSH.setArg(5, cl_simParams);
		err = kernel_useSH.setArg(8, cl::__local(sizeof(cl_float4)*(LOCAL_PREF)));
		err = kernel_useSH.setArg(9, Y_AxisFixed);
		err = kernel_useSH.setArg(10, dt);
	}
	catch (cl::Error er){
		log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
	}

	localWorkSize = LOCAL_PREF;
	globalWorkSize = LOCAL_PREF * (simParams.numCells);
	err = queue.enqueueNDRangeKernel(kernel_useSH, cl::NullRange, cl::NDRange(globalWorkSize), cl::NDRange(localWorkSize), NULL, &event);
	queue.finish();

	event.wait();
	event.getProfilingInfo<cl_ulong>(CL_PROFILING_COMMAND_START, &startTime);
	event.getProfilingInfo<cl_ulong>(CL_PROFILING_COMMAND_END, &endTime);
	times[5] = (endTime - startTime) / 1000000;

	/*
	unsigned int A[8000];
	queue.enqueueReadBuffer(cl_range, CL_TRUE, 0, (size_t)(8000 * sizeof(unsigned int)), &A);
	queue.finish();
	*/
	/*std::vector<Vec4> X(simParams.numCells);
	queue.enqueueReadBuffer(cl_sumVel, CL_TRUE, 0, (size_t)simParams.numCells * sizeof(Vec4), X.data());
	queue.finish();
	*/

	//Release the VBOs so OpenGL can play with them
	err = queue.enqueueReleaseGLObjects(&cl_pos_vbos, NULL, &event);
	err = queue.enqueueReleaseGLObjects(&cl_pos_vbos_out, NULL, &event);
	err = queue.enqueueReleaseGLObjects(&cl_vel_vbos, NULL, &event);
	err = queue.enqueueReleaseGLObjects(&cl_vel_vbos_out, NULL, &event);

}

GLuint BoidModelSH_2D::getPosVBO(){
	if (counter)
		return pos_vbo[0];
	else
		return pos_vbo_out[0];
}

GLuint BoidModelSH_2D::getVelVBO(){
	if (counter)
		return vel_vbo[0];
	else
		return vel_vbo_out[0];
}

GLuint BoidModelSH_2D::getPosVAO(){
	if (counter)
		return pos_vao[0];
	else
		return pos_vao_out[0];
}

int BoidModelSH_2D::getNumBoid(){
	return num;
}

//Private Methods

cl::Program BoidModelSH_2D::loadProgram(const std::string &filename){
	log("load program");
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

	int pl;
	cl::Program program;

	pl = kernelSource.size();

	try
	{
		cl::Program::Sources source(1,
			std::make_pair(kernelSource.c_str(), pl));
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

void BoidModelSH_2D::loadKernel(){
	log("loading kernels");
	try{
		kernel_getGridHash = cl::Kernel(programBoid, "getGridHash", &err);
		kernel_findGridEdgeAndReorder = cl::Kernel(programBoid, "findGridEdgeAndReorder", &err);
		kernel_simulate = cl::Kernel(programBoid, "simulate", &err);
		kernel_bitonicSortLocal = cl::Kernel(programBitonic, "bitonicSortLocal", &err);
		kernel_bitonicSortLocal1 = cl::Kernel(programBitonic, "bitonicSortLocal1", &err);
		kernel_bitonicMergeGlobal = cl::Kernel(programBitonic, "bitonicMergeGlobal", &err);
		kernel_bitonicMergeLocal = cl::Kernel(programBitonic, "bitonicMergeLocal", &err);
		kernel_memSet = cl::Kernel(programBoid, "memSet", &err);
		kernel_sumVelSH = cl::Kernel(programBoid, "sumVelSH", &err);
		kernel_useSH = cl::Kernel(programBoid, "useSH", &err);
	}
	catch (cl::Error er) {
		log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
	}

}

void BoidModelSH_2D::createBuffer(std::vector<Vec4> pos, std::vector<Vec4> vel){
	log("Create buffer for usage");

	size_t array_size_fp4 = num * sizeof(Vec4);
	size_t array_size_simple = num * sizeof(unsigned int);
	size_t array_size_edges = simParams.numCells * sizeof(unsigned int);
	size_t array_size_fp4_cells = simParams.numCells * sizeof(Vec4);

	createVboBindShader(pos, vel);
	// create OpenCL buffer from GL VBO
	cl_pos_vbos.push_back(cl::BufferGL(context, CL_MEM_READ_WRITE, pos_vbo[0], &err));
	cl_pos_vbos_out.push_back(cl::BufferGL(context, CL_MEM_READ_WRITE, pos_vbo_out[0], &err));
	cl_vel_vbos.push_back(cl::BufferGL(context, CL_MEM_READ_WRITE, vel_vbo[0], &err));
	cl_vel_vbos_out.push_back(cl::BufferGL(context, CL_MEM_READ_WRITE, vel_vbo_out[0], &err));

	//create the OpenCL only arrays
	try
	{
		cl_gridHash_unsorted = cl::Buffer(context, CL_MEM_READ_WRITE, array_size_simple, NULL, &err);
		cl_gridHash_sorted = cl::Buffer(context, CL_MEM_READ_WRITE, array_size_simple, NULL, &err);
		cl_gridIndex_sorted = cl::Buffer(context, CL_MEM_READ_WRITE, array_size_simple, NULL, &err);
		cl_gridIndex_unsorted = cl::Buffer(context, CL_MEM_READ_WRITE, array_size_simple, NULL, &err);
		cl_gridStartIndex = cl::Buffer(context, CL_MEM_READ_WRITE, array_size_edges, NULL, &err);
		cl_gridEndIndex = cl::Buffer(context, CL_MEM_READ_WRITE, array_size_edges, NULL, &err);
		cl_range = cl::Buffer(context, CL_MEM_READ_WRITE, array_size_edges, NULL, &err);
		cl_simParams = cl::Buffer(context, CL_MEM_READ_ONLY, sizeof(simParams_t), NULL, &err);
		cl_sumVel = cl::Buffer(context, CL_MEM_READ_WRITE, array_size_fp4_cells, NULL, &err);
	}
	catch (cl::Error er) {
		log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
	}
}

void BoidModelSH_2D::createVboBindShader(std::vector<Vec4> pos, std::vector<Vec4> vel){
	std::vector<Vec4> newDataColor(num);

	for (int i = 0; i < num; i++){
		newDataColor[i] = BOID_COLOR;
	}

	GLuint id[1];
	size_t array_size = num * sizeof(Vec4);

	//create shader
	shader = new Shader("boidTri.v.glsl", "boidTri.f.glsl", "boidTri.g.glsl");
	GLint vertLoc = glGetAttribLocation(shader->id(), "coord3d");
	GLint colorLoc = glGetAttribLocation(shader->id(), "color");
	GLint velLoc = glGetAttribLocation(shader->id(), "vel3d");

	//------VBO 1--------- (in)
	glGenVertexArrays(1, &pos_vao[0]); // Create our Vertex Array Object  
	glBindVertexArray(pos_vao[0]); // Bind our Vertex Array Object so we can use it  

	pos_vbo[0] = clHelper->createVBO(&pos[0], array_size, GL_ARRAY_BUFFER, GL_DYNAMIC_DRAW);

	//std::vector<Vec4> test(num);
	//glGetBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(Vec4)* num, test.data());

	glVertexAttribPointer(vertLoc, 4, GL_FLOAT, GL_FALSE, 0, 0); // Set up our vertex attributes pointer
	glEnableVertexAttribArray(vertLoc);

	vel_vbo[0] = clHelper->createVBO(&vel[0], array_size, GL_ARRAY_BUFFER, GL_DYNAMIC_DRAW);

	glVertexAttribPointer(velLoc, 4, GL_FLOAT, GL_FALSE, 0, 0); // Set up our velocity attributes pointer
	glEnableVertexAttribArray(velLoc);

	glGenBuffers(1, &id[0]);
	glBindBuffer(GL_ARRAY_BUFFER, id[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Vec4)* num, &newDataColor[0], GL_STATIC_DRAW);

	glVertexAttribPointer(colorLoc, 4, GL_FLOAT, GL_FALSE, 0, 0); // Set up our vertex attributes pointer  
	glEnableVertexAttribArray(colorLoc);

	glEnableVertexAttribArray(0); // Disable our Vertex Array Object  
	glBindVertexArray(0);

	//------VBO 1--------- (in)
	glGenVertexArrays(1, &pos_vao_out[0]); // Create our Vertex Array Object  
	glBindVertexArray(pos_vao_out[0]); // Bind our Vertex Array Object so we can use it  

	pos_vbo_out[0] = clHelper->createVBO(&pos[0], array_size, GL_ARRAY_BUFFER, GL_DYNAMIC_DRAW);

	//std::vector<Vec4> test(num);
	//glGetBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(Vec4)* num, test.data());

	glVertexAttribPointer(vertLoc, 4, GL_FLOAT, GL_FALSE, 0, 0); // Set up our vertex attributes pointer
	glEnableVertexAttribArray(vertLoc);

	vel_vbo_out[0] = clHelper->createVBO(&vel[0], array_size, GL_ARRAY_BUFFER, GL_DYNAMIC_DRAW);

	glVertexAttribPointer(velLoc, 4, GL_FLOAT, GL_FALSE, 0, 0); // Set up our velocity attributes pointer
	glEnableVertexAttribArray(velLoc);

	glGenBuffers(1, &id[0]);
	glBindBuffer(GL_ARRAY_BUFFER, id[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Vec4)* num, &newDataColor[0], GL_STATIC_DRAW);

	glVertexAttribPointer(colorLoc, 4, GL_FLOAT, GL_FALSE, 0, 0); // Set up our vertex attributes pointer  
	glEnableVertexAttribArray(colorLoc);

	glEnableVertexAttribArray(0); // Disable our Vertex Array Object  
	glBindVertexArray(0);

	log("GL VBO Buffer created");
}

void BoidModelSH_2D::loadData(){
	err = queue.enqueueWriteBuffer(cl_simParams, CL_TRUE, 0, sizeof(simParams_t), &simParams, NULL, &event);
	queue.finish();
}

void BoidModelSH_2D::bitonicSort(
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
	times[1] = GetTickCount64() - timeNow;
}

cl_uint BoidModelSH_2D::factorRadix2(cl_uint& log2L, cl_uint L){
	if (!L){
		log2L = 0;
		return 0;
	}
	else{
		for (log2L = 0; (L & 1) == 0; L >>= 1, log2L++);
		return L;
	}
}

long BoidModelSH_2D::getSimulationTime(){
	cl_ulong startTime, endTime;
	eventSim.wait();
	eventSim.getProfilingInfo<cl_ulong>(CL_PROFILING_COMMAND_START, &startTime);
	eventSim.getProfilingInfo<cl_ulong>(CL_PROFILING_COMMAND_END, &endTime);
	return (endTime - startTime) / 1000000;
}

void BoidModelSH_2D::bindShader(){
	shader->bind();
}

void BoidModelSH_2D::unbindShader(){
	shader->unbind();
}

std::vector<const char*> BoidModelSH_2D::getSimTimeDescriptions(){
	std::stringstream strstream;

	strstream.str(std::string());
	strstream << "Calculate Grid Hash time: " << times[0] << "ns";
	stringHashTime = strstream.str();
	simTimeDisc[4] = stringHashTime.c_str();

	strstream.str(std::string());
	strstream << "Sorting time: " << times[1] << "ms";
	stringSortTime = strstream.str();
	simTimeDisc[5] = stringSortTime.c_str();

	strstream.str(std::string());
	strstream << "Edge detect./reorder time: " << times[2] << "ms";
	stringEdgeTime = strstream.str();
	simTimeDisc[6] = stringEdgeTime.c_str();

	strstream.str(std::string());
	strstream << "Simulation time: " << times[3] << "ms";
	stringSimTime = strstream.str();
	simTimeDisc[7] = stringSimTime.c_str();

	strstream.str(std::string());
	strstream << "Sum Vel. time: " << times[4] << "ms";
	stringSumTime = strstream.str();
	simTimeDisc[8] = stringSumTime.c_str();

	strstream.str(std::string());
	strstream << "SH total time: " << times[5] << "ms";
	stringSHTime = strstream.str();
	simTimeDisc[9] = stringSHTime.c_str();

	return simTimeDisc;
}


void BoidModelSH_2D::getFollowedBoid(unsigned int* boidIndex, Vec4* pos, Vec4* vel){
	size_t size = sizeof(unsigned int)* num;
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
	glGetBufferSubData(GL_ARRAY_BUFFER, sizeof(Vec4)* *boidIndex, sizeof(Vec4), &v);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	(*vel).set(v.x, v.y - 10.f, v.z, 0.0);

	Vec4 p;
	vbo = getPosVBO();
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glGetBufferSubData(GL_ARRAY_BUFFER, sizeof(Vec4)* *boidIndex, sizeof(Vec4), &p);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	(*pos).set(p.x, p.y, p.z, 0.0);
}


