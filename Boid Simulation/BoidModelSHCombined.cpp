#include "stdafx.h"
#include "boidModel.h"

BoidModelSHCombined::BoidModelSHCombined(CLHelper* clHlpr, std::vector<Vec4> pos, std::vector<Vec4> vel, std::vector<Vec4> goal, std::vector<Vec4> color, simParams_t* simP, std::vector<Vec4> cor, std::vector<unsigned int> start, std::vector<unsigned int> end, std::vector<Vec4> posObst) : BoidModel(clHlpr)
{
	simTimeDisc = std::vector<const char*>(10);
	simTimeDisc[0] = "SH obstacle avoidance";
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

	createBuffer(pos, vel, goal, color);
	loadData(goal);

	programBoid    = loadProgram(kernel_path + "BoidModelSHCombined_kernel_v1.cl");
	programBitonic = loadProgram(kernel_path + "bitonic_sort.cl");

	loadKernel();

	createAndLoadObstacleSH(cor, start, end, posObst);

	log("setup complete - simulation is runable");
}

BoidModelSHCombined::~BoidModelSHCombined(){
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glDeleteBuffers(1, pos_vbo);

	glBindVertexArray(0);
	glDeleteVertexArrays(1, pos_vao);

	delete shader;
}

void BoidModelSHCombined::render(){
	shader->bind();
	glBindVertexArray(getPosVAO());
	glDrawArrays(GL_POINTS, 0, num);
	glBindVertexArray(0);
	shader->unbind();
}

Shader* BoidModelSHCombined::getShader(){
	return shader;
}

void BoidModelSHCombined::simulate(float dt){
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
	err = queue.enqueueAcquireGLObjects(&cl_color_vbos, NULL, &event);
	err = queue.enqueueAcquireGLObjects(&cl_color_vbos_out, NULL, &event);
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
			err = kernel_findGridEdgeAndReorder.setArg(8, cl_goal_in);
			err = kernel_findGridEdgeAndReorder.setArg(9, cl_goal_out);
			err = kernel_findGridEdgeAndReorder.setArg(11, cl_color_vbos[0]);
			err = kernel_findGridEdgeAndReorder.setArg(10, cl_color_vbos_out[0]);
		}
		else {
			err = kernel_findGridEdgeAndReorder.setArg(6, cl_pos_vbos_out[0]);	//pos in
			err = kernel_findGridEdgeAndReorder.setArg(7, cl_vel_vbos_out[0]);	//vel in
			err = kernel_findGridEdgeAndReorder.setArg(2, cl_pos_vbos[0]);		//pos out
			err = kernel_findGridEdgeAndReorder.setArg(3, cl_vel_vbos[0]);		//vel out
			err = kernel_findGridEdgeAndReorder.setArg(9, cl_goal_in);
			err = kernel_findGridEdgeAndReorder.setArg(8, cl_goal_out);
			err = kernel_findGridEdgeAndReorder.setArg(10, cl_color_vbos[0]);
			err = kernel_findGridEdgeAndReorder.setArg(11, cl_color_vbos_out[0]);
		}

		err = kernel_findGridEdgeAndReorder.setArg(0, cl_gridStartIndex);
		err = kernel_findGridEdgeAndReorder.setArg(1, cl_gridEndIndex);
		err = kernel_findGridEdgeAndReorder.setArg(4, cl_gridHash_sorted);
		err = kernel_findGridEdgeAndReorder.setArg(5, cl_gridIndex_sorted);
		err = kernel_findGridEdgeAndReorder.setArg(12, cl::__local(sizeof(cl_uint)*(LOCAL_PREF + 1)));
		err = kernel_findGridEdgeAndReorder.setArg(13, num);
	}
	catch (cl::Error er) {
		log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
	}

	err = queue.enqueueNDRangeKernel(kernel_findGridEdgeAndReorder, cl::NullRange, cl::NDRange(num), cl::NDRange(LOCAL_PREF), NULL, &event);

	queue.finish();
	event.wait();
	event.getProfilingInfo<cl_ulong>(CL_PROFILING_COMMAND_START, &startTime);
	event.getProfilingInfo<cl_ulong>(CL_PROFILING_COMMAND_END, &endTime);
	times[2] = (endTime - startTime) / 1000000;

	try
	{
		if (counter)
			err = kernel_evalSH.setArg(0, cl_vel_vbos_out[0]);
		else
			err = kernel_evalSH.setArg(0, cl_vel_vbos[0]);

		err = kernel_evalSH.setArg(1, cl_shEvalX);
		err = kernel_evalSH.setArg(2, cl_shEvalY);
		err = kernel_evalSH.setArg(3, cl_shEvalZ);
		err = kernel_evalSH.setArg(4, cl_coef0X);
		err = kernel_evalSH.setArg(5, cl_coef0Y);
		err = kernel_evalSH.setArg(6, cl_coef0Z);
	}
	catch (cl::Error er){
		log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
	}



	int localWorkSize = LOCAL_PREF;
	int globalWorkSize = simParams.numBodies;
	err = queue.enqueueNDRangeKernel(kernel_evalSH, cl::NullRange, cl::NDRange(globalWorkSize), cl::NDRange(localWorkSize), NULL, &event);

	event.wait();
	event.getProfilingInfo<cl_ulong>(CL_PROFILING_COMMAND_START, &startTime);
	event.getProfilingInfo<cl_ulong>(CL_PROFILING_COMMAND_END, &endTime);
	times[4] = (endTime - startTime) / 1000000;

	//		std::vector<Vec4> C(2 * simParams.numCells);
	//		queue.enqueueReadBuffer(cl_shEval, CL_TRUE, 0, (size_t)2 * simParams.numCells * sizeof(Vec4), C.data());
	//		queue.finish(); 

	try
	{
		if (counter){
			err = kernel_simulate.setArg(0, cl_pos_vbos_out[0]);	//pos in
			err = kernel_simulate.setArg(1, cl_pos_vbos[0]);		//pos out
			err = kernel_simulate.setArg(2, cl_vel_vbos_out[0]);	//vel in
			err = kernel_simulate.setArg(3, cl_vel_vbos[0]);		//vel out
			err = kernel_simulate.setArg(6, cl_goal_out);
		}
		else{
			err = kernel_simulate.setArg(1, cl_pos_vbos_out[0]);	//pos out
			err = kernel_simulate.setArg(0, cl_pos_vbos[0]);		//pos in
			err = kernel_simulate.setArg(3, cl_vel_vbos_out[0]);	//vel out
			err = kernel_simulate.setArg(2, cl_vel_vbos[0]);		//vel in
			err = kernel_simulate.setArg(6, cl_goal_in);
		}

		err = kernel_simulate.setArg(4, cl_gridStartIndex);
		err = kernel_simulate.setArg(5, cl_gridEndIndex);
		err = kernel_simulate.setArg(7, cl_simParams);
		err = kernel_simulate.setArg(8, cl_range);
		err = kernel_simulate.setArg(9, dt);
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
	globalWorkSize = simParams.numBodies;
	err = queue.enqueueNDRangeKernel(kernel_simulate, cl::NullRange, cl::NDRange(globalWorkSize), cl::NDRange(localWorkSize), NULL, &eventSim);

	eventSim.wait();
	eventSim.getProfilingInfo<cl_ulong>(CL_PROFILING_COMMAND_START, &startTime);
	eventSim.getProfilingInfo<cl_ulong>(CL_PROFILING_COMMAND_END, &endTime);
	times[3] = (endTime - startTime) / 1000000;
	unsigned int numObst = 126;

	try
	{
		if (counter){
			err = kernel_useSH.setArg(0, cl_vel_vbos[0]);		//vel in
			err = kernel_useSH.setArg(1, cl_vel_vbos_out[0]);	//vel out
			err = kernel_useSH.setArg(8, cl_pos_vbos[0]);		//pos in	
			err = kernel_useSH.setArg(9, cl_pos_vbos_out[0]);	//pos out
		}
		else {
			err = kernel_useSH.setArg(1, cl_vel_vbos[0]);		//vel out
			err = kernel_useSH.setArg(0, cl_vel_vbos_out[0]);	//vel in
			err = kernel_useSH.setArg(9, cl_pos_vbos[0]);		//pos out
			err = kernel_useSH.setArg(8, cl_pos_vbos_out[0]);	//pos in
		}

		err = kernel_useSH.setArg(2, cl_gridStartIndex);
		err = kernel_useSH.setArg(3, cl_gridEndIndex);
		err = kernel_useSH.setArg(4, cl_shEvalX);
		err = kernel_useSH.setArg(5, cl_shEvalY);
		err = kernel_useSH.setArg(6, cl_shEvalZ);
		err = kernel_useSH.setArg(7, cl_simParams);
		err = kernel_useSH.setArg(10, cl::__local(sizeof(cl_float8)*(LOCAL_PREF)));
		err = kernel_useSH.setArg(11, cl::__local(sizeof(cl_float8)*(LOCAL_PREF)));
		err = kernel_useSH.setArg(12, cl::__local(sizeof(cl_float8)*(LOCAL_PREF)));
		err = kernel_useSH.setArg(13, cl_coef0X);
		err = kernel_useSH.setArg(14, cl_coef0Y);
		err = kernel_useSH.setArg(15, cl_coef0Z);
		err = kernel_useSH.setArg(16, cl::__local(sizeof(cl_float)*(LOCAL_PREF)));
		err = kernel_useSH.setArg(17, cl::__local(sizeof(cl_float)*(LOCAL_PREF)));
		err = kernel_useSH.setArg(18, cl::__local(sizeof(cl_float)*(LOCAL_PREF)));
		err = kernel_useSH.setArg(19, cl_shEvalOX);
		err = kernel_useSH.setArg(20, cl_shEvalOY);
		err = kernel_useSH.setArg(21, cl_shEvalOZ);
		err = kernel_useSH.setArg(22, cl_coef0OX);
		err = kernel_useSH.setArg(23, cl_coef0OY);
		err = kernel_useSH.setArg(24, cl_coef0OZ);
		err = kernel_useSH.setArg(25, cl_posObst);
		err = kernel_useSH.setArg(26, cl::__local(sizeof(cl_float4)*(LOCAL_PREF)));
		err = kernel_useSH.setArg(27, numObst);
		err = kernel_useSH.setArg(28, dt);
	}
	catch (cl::Error er){
		log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
	}

	localWorkSize = LOCAL_PREF;
	globalWorkSize = simParams.numBodies;
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
	err = queue.enqueueReleaseGLObjects(&cl_color_vbos, NULL, &event);
	err = queue.enqueueReleaseGLObjects(&cl_color_vbos_out, NULL, &event);
}

GLuint BoidModelSHCombined::getPosVBO(){
	if (counter)
		return pos_vbo[0];
	else
		return pos_vbo_out[0];
}

GLuint BoidModelSHCombined::getVelVBO(){
	if (counter)
		return vel_vbo[0];
	else
		return vel_vbo_out[0];
}

GLuint BoidModelSHCombined::getPosVAO(){
	if (counter)
		return pos_vao[0];
	else
		return pos_vao_out[0];
}

int BoidModelSHCombined::getNumBoid(){
	return num;
}

//Private Methods

cl::Program BoidModelSHCombined::loadProgram(const std::string &filename){
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

void BoidModelSHCombined::loadKernel(){
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
		kernel_evalSH = cl::Kernel(programBoid, "evalSH", &err);
		kernel_obstacle = cl::Kernel(programBoid, "obstacleSH", &err);

#if USE_SH_FOR_PATH
		kernel_useSH = cl::Kernel(programBoid, "useSH", &err);
#else
		kernel_useSH = cl::Kernel(programBoid, "dontUseSH", &err);
#endif
	}
	catch (cl::Error er) {
		log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
	}

}

void BoidModelSHCombined::createBuffer(std::vector<Vec4> pos, std::vector<Vec4> vel, std::vector<Vec4> goal, std::vector<Vec4> color){
	log("Create buffer for usage");

	size_t array_size_fp4 = num * sizeof(Vec4);
	size_t array_size_simple = num * sizeof(unsigned int);
	size_t array_size_edges = simParams.numCells * sizeof(unsigned int);
	size_t array_size_fp4_cells = simParams.numCells * sizeof(Vec4);
	size_t array_size_fp8 = 2 * num * sizeof(Vec4);
	size_t array_size_fp = num * sizeof(float);

	createVboBindShader(pos, vel, color);
	// create OpenCL buffer from GL VBO
	cl_pos_vbos.push_back(cl::BufferGL(context, CL_MEM_READ_WRITE, pos_vbo[0], &err));
	cl_pos_vbos_out.push_back(cl::BufferGL(context, CL_MEM_READ_WRITE, pos_vbo_out[0], &err));

	cl_vel_vbos.push_back(cl::BufferGL(context, CL_MEM_READ_WRITE, vel_vbo[0], &err));
	cl_vel_vbos_out.push_back(cl::BufferGL(context, CL_MEM_READ_WRITE, vel_vbo_out[0], &err));

	cl_color_vbos.push_back(cl::BufferGL(context, CL_MEM_READ_WRITE, color_vbo[0], &err));
	cl_color_vbos_out.push_back(cl::BufferGL(context, CL_MEM_READ_WRITE, color_vbo_out[0], &err));
	//create the OpenCL only arrays
	try
	{
		cl_coef0X = cl::Buffer(context, CL_MEM_READ_WRITE, array_size_fp, NULL, &err);
		cl_coef0Y = cl::Buffer(context, CL_MEM_READ_WRITE, array_size_fp, NULL, &err);
		cl_coef0Z = cl::Buffer(context, CL_MEM_READ_WRITE, array_size_fp, NULL, &err);
		cl_shEvalX = cl::Buffer(context, CL_MEM_READ_WRITE, array_size_fp8, NULL, &err);
		cl_shEvalY = cl::Buffer(context, CL_MEM_READ_WRITE, array_size_fp8, NULL, &err);
		cl_shEvalZ = cl::Buffer(context, CL_MEM_READ_WRITE, array_size_fp8, NULL, &err);
		cl_goal_in = cl::Buffer(context, CL_MEM_READ_WRITE, array_size_fp4, NULL, &err);
		cl_goal_out = cl::Buffer(context, CL_MEM_READ_WRITE, array_size_fp4, NULL, &err);
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

void BoidModelSHCombined::createAndLoadObstacleSH(std::vector<Vec4> cor, std::vector<unsigned int> start, std::vector<unsigned int> end, std::vector<Vec4> posObst){
	size_t array_size_fp8 = 2 * posObst.size() * sizeof(Vec4);
	size_t array_size_fp = posObst.size() * sizeof(float);
	size_t array_size_index = start.size() * sizeof(unsigned int);
	size_t array_size_cor = cor.size() * sizeof(Vec4);
	size_t array_size_pos = posObst.size() * sizeof(Vec4);

	try
	{
		cl_coef0OX = cl::Buffer(context, CL_MEM_READ_WRITE, array_size_fp, NULL, &err);
		cl_coef0OY = cl::Buffer(context, CL_MEM_READ_WRITE, array_size_fp, NULL, &err);
		cl_coef0OZ = cl::Buffer(context, CL_MEM_READ_WRITE, array_size_fp, NULL, &err);
		cl_shEvalOX = cl::Buffer(context, CL_MEM_READ_WRITE, array_size_fp8, NULL, &err);
		cl_shEvalOY = cl::Buffer(context, CL_MEM_READ_WRITE, array_size_fp8, NULL, &err);
		cl_shEvalOZ = cl::Buffer(context, CL_MEM_READ_WRITE, array_size_fp8, NULL, &err);
		cl_startCor = cl::Buffer(context, CL_MEM_READ_ONLY, array_size_index, NULL, &err);
		cl_endCor = cl::Buffer(context, CL_MEM_READ_ONLY, array_size_index, NULL, &err);
		cl_posObst = cl::Buffer(context, CL_MEM_READ_ONLY, array_size_pos, NULL, &err);
		cl_cor = cl::Buffer(context, CL_MEM_READ_ONLY, array_size_cor, NULL, &err);
	}
	catch (cl::Error er) {
		log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
	}

	err = queue.enqueueWriteBuffer(cl_startCor, CL_TRUE, 0, array_size_index, &start[0], NULL, &event);
	err = queue.enqueueWriteBuffer(cl_endCor, CL_TRUE, 0, array_size_index, &end[0], NULL, &event);
	err = queue.enqueueWriteBuffer(cl_posObst, CL_TRUE, 0, array_size_pos, &posObst[0], NULL, &event);
	err = queue.enqueueWriteBuffer(cl_cor, CL_TRUE, 0, array_size_cor, &cor[0], NULL, &event);
	queue.finish();

	try
	{
		err = kernel_obstacle.setArg(0, cl_cor);
		err = kernel_obstacle.setArg(1, cl_startCor);
		err = kernel_obstacle.setArg(2, cl_endCor);
		err = kernel_obstacle.setArg(3, cl_shEvalOX);
		err = kernel_obstacle.setArg(4, cl_shEvalOY);
		err = kernel_obstacle.setArg(5, cl_shEvalOZ);
		err = kernel_obstacle.setArg(6, cl_coef0OX);
		err = kernel_obstacle.setArg(7, cl_coef0OY);
		err = kernel_obstacle.setArg(8, cl_coef0OZ);
	}
	catch (cl::Error er){
		log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
	}


	int globalWorkSize = posObst.size();
	err = queue.enqueueNDRangeKernel(kernel_obstacle, cl::NullRange, cl::NDRange(globalWorkSize), cl::NullRange, NULL, &event);
	queue.finish();


	std::vector<Vec4> X(126);
	queue.enqueueReadBuffer(cl_posObst, CL_TRUE, 0, (size_t)126 * sizeof(Vec4), X.data());
	queue.finish();

	std::vector<Vec4> Y(2 * 126);
	queue.enqueueReadBuffer(cl_shEvalOX, CL_TRUE, 0, (size_t)2 * 126 * sizeof(Vec4), Y.data());
	queue.finish();


}

void BoidModelSHCombined::createVboBindShader(std::vector<Vec4> pos, std::vector<Vec4> vel, std::vector<Vec4> color){
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

	color_vbo[0] = clHelper->createVBO(&color[0], array_size, GL_ARRAY_BUFFER, GL_DYNAMIC_DRAW);

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

	color_vbo_out[0] = clHelper->createVBO(&color[0], array_size, GL_ARRAY_BUFFER, GL_DYNAMIC_DRAW);

	glVertexAttribPointer(colorLoc, 4, GL_FLOAT, GL_FALSE, 0, 0); // Set up our vertex attributes pointer  
	glEnableVertexAttribArray(colorLoc);

	glEnableVertexAttribArray(0); // Disable our Vertex Array Object  
	glBindVertexArray(0);

	log("GL VBO Buffer created");
}

void BoidModelSHCombined::loadData(std::vector<Vec4> goal){
	num = (int)goal.size();
	size_t array_size_fp4 = num * sizeof(Vec4);

	err = queue.enqueueWriteBuffer(cl_goal_in, CL_TRUE, 0, array_size_fp4, &goal[0], NULL, &event);
	err = queue.enqueueWriteBuffer(cl_simParams, CL_TRUE, 0, sizeof(simParams_t), &simParams, NULL, &event);
	queue.finish();
}

void BoidModelSHCombined::bitonicSort(
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

cl_uint BoidModelSHCombined::factorRadix2(cl_uint& log2L, cl_uint L){
	if (!L){
		log2L = 0;
		return 0;
	}
	else{
		for (log2L = 0; (L & 1) == 0; L >>= 1, log2L++);
		return L;
	}
}

long BoidModelSHCombined::getSimulationTime(){
	cl_ulong startTime, endTime;
	eventSim.wait();
	eventSim.getProfilingInfo<cl_ulong>(CL_PROFILING_COMMAND_START, &startTime);
	eventSim.getProfilingInfo<cl_ulong>(CL_PROFILING_COMMAND_END, &endTime);
	return (endTime - startTime) / 1000000;
}

void BoidModelSHCombined::bindShader(){
	shader->bind();
}

void BoidModelSHCombined::unbindShader(){
	shader->unbind();
}

std::vector<const char*> BoidModelSHCombined::getSimTimeDescriptions(){
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

void BoidModelSHCombined::getFollowedBoid(unsigned int* boidIndex, Vec4* pos, Vec4* vel){
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

	(*vel).set(v.x, v.y, v.z, 0.0);

	Vec4 p;
	vbo = getPosVBO();
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glGetBufferSubData(GL_ARRAY_BUFFER, sizeof(Vec4)* *boidIndex, sizeof(Vec4), &p);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	(*pos).set(p.x, p.y, p.z, 0.0);
}


