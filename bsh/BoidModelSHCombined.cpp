#include "common.h"
#include "BoidModel.h"

BoidModelSHCombined::BoidModelSHCombined(CLHelper* clHlpr, std::vector<Vec4> pos, std::vector<Vec4> vel,
	std::vector<Vec4> goal, std::vector<Vec4> color, simParams_t* simP, std::vector<Vec4> cor,
	std::vector<unsigned int> start, std::vector<unsigned int> end, std::vector<Vec4> posObst)
	: BoidModelSHCombined(clHlpr, pos, vel, goal, color, simP, cor, start, end, posObst,
		"boidModelSHCombined_kernel_v1.cl", "Boid Model SH Obstacle + Way")
{
}

BoidModelSHCombined::BoidModelSHCombined(CLHelper* clHlpr, std::vector<Vec4> pos, std::vector<Vec4> vel,
	std::vector<Vec4> goal, std::vector<Vec4> color, simParams_t* simP, std::vector<Vec4> cor,
	std::vector<unsigned int> start, std::vector<unsigned int> end, std::vector<Vec4> posObst,
	const std::string& kernelFile, const char* modelName)
	: BoidModelGridBase(clHlpr, simP, modelName, true)
{
	addTimingRow("Eval SH time", 4);
	addTimingRow("Use SH time", 5);

	numObst = (unsigned int)posObst.size();

	createCommonBuffers(pos, vel, color, true);
	createGoalBuffers(goal);

	//per boid SH coefficients
	size_t array_size_fp8 = 2 * num * sizeof(Vec4);
	size_t array_size_fp = num * sizeof(float);
	try{
		cl_shEvalX = cl::Buffer(context, CL_MEM_READ_WRITE, array_size_fp8, NULL, &err);
		cl_shEvalY = cl::Buffer(context, CL_MEM_READ_WRITE, array_size_fp8, NULL, &err);
		cl_shEvalZ = cl::Buffer(context, CL_MEM_READ_WRITE, array_size_fp8, NULL, &err);
		cl_coef0X = cl::Buffer(context, CL_MEM_READ_WRITE, array_size_fp, NULL, &err);
		cl_coef0Y = cl::Buffer(context, CL_MEM_READ_WRITE, array_size_fp, NULL, &err);
		cl_coef0Z = cl::Buffer(context, CL_MEM_READ_WRITE, array_size_fp, NULL, &err);
	}
	catch (cl::Error er) {
		log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
	}

	uploadSimParams();

	programBoid    = loadBoidProgram(kernel_path + kernelFile);
	programBitonic = loadProgram(kernel_path + "bitonic_sort.cl");
	loadCommonKernels();

	try{
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

	createAndLoadObstacleSH(cor, start, end, posObst);

	log("setup complete - simulation is runable");
}

/* project the static obstacle geometry into SH coefficients, done once at setup */
void BoidModelSHCombined::createAndLoadObstacleSH(const std::vector<Vec4>& cor, const std::vector<unsigned int>& start,
	const std::vector<unsigned int>& end, const std::vector<Vec4>& posObst){

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

	err = queue.enqueueNDRangeKernel(kernel_obstacle, cl::NullRange, cl::NDRange(numObst), cl::NullRange, NULL, &event);
	queue.finish();
}

void BoidModelSHCombined::simulate(float dt){
	counter = !counter;

	acquireGLBuffers();

	runGridHash(counter ? cl_pos_vbos[0] : cl_pos_vbos_out[0]);
	clearGridEdges();
	sortGridHash();

	//reorder pos/vel/goal/color into the buffer set that is not rendered this frame
	try
	{
		if (counter){
			err = kernel_findGridEdgeAndReorder.setArg(2, cl_pos_vbos_out[0]);	//pos out ordered
			err = kernel_findGridEdgeAndReorder.setArg(3, cl_vel_vbos_out[0]);	//vel out ordered
			err = kernel_findGridEdgeAndReorder.setArg(6, cl_pos_vbos[0]);		//pos in unordered
			err = kernel_findGridEdgeAndReorder.setArg(7, cl_vel_vbos[0]);		//vel in unordered
			err = kernel_findGridEdgeAndReorder.setArg(8, cl_goal_in);
			err = kernel_findGridEdgeAndReorder.setArg(9, cl_goal_out);
			err = kernel_findGridEdgeAndReorder.setArg(10, cl_color_vbos_out[0]);
			err = kernel_findGridEdgeAndReorder.setArg(11, cl_color_vbos[0]);
		}
		else {
			err = kernel_findGridEdgeAndReorder.setArg(2, cl_pos_vbos[0]);		//pos out
			err = kernel_findGridEdgeAndReorder.setArg(3, cl_vel_vbos[0]);		//vel out
			err = kernel_findGridEdgeAndReorder.setArg(6, cl_pos_vbos_out[0]);	//pos in
			err = kernel_findGridEdgeAndReorder.setArg(7, cl_vel_vbos_out[0]);	//vel in
			err = kernel_findGridEdgeAndReorder.setArg(8, cl_goal_out);
			err = kernel_findGridEdgeAndReorder.setArg(9, cl_goal_in);
			err = kernel_findGridEdgeAndReorder.setArg(10, cl_color_vbos[0]);
			err = kernel_findGridEdgeAndReorder.setArg(11, cl_color_vbos_out[0]);
		}
	}
	catch (cl::Error er) {
		log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
	}
	runReorder();

	//SH coefficients of every single boid
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

	err = queue.enqueueNDRangeKernel(kernel_evalSH, cl::NullRange, cl::NDRange(simParams.numBodies), cl::NDRange(LOCAL_PREF), NULL, &event);
	times[4] = eventTimeUs(event);

	//local flocking and goal seeking on the reordered buffers
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
			err = kernel_simulate.setArg(0, cl_pos_vbos[0]);		//pos in
			err = kernel_simulate.setArg(1, cl_pos_vbos_out[0]);	//pos out
			err = kernel_simulate.setArg(2, cl_vel_vbos[0]);		//vel in
			err = kernel_simulate.setArg(3, cl_vel_vbos_out[0]);	//vel out
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

	err = queue.enqueueNDRangeKernel(kernel_simulate, cl::NullRange, cl::NDRange(simParams.numBodies), cl::NDRange(LOCAL_PREF), NULL, &eventSim);
	times[3] = eventTimeUs(eventSim);

	//long range avoidance of the other boids and the obstacle SH
	try
	{
		if (counter){
			err = kernel_useSH.setArg(0, cl_vel_vbos[0]);		//vel in
			err = kernel_useSH.setArg(1, cl_vel_vbos_out[0]);	//vel out
			err = kernel_useSH.setArg(8, cl_pos_vbos[0]);		//pos in
			err = kernel_useSH.setArg(9, cl_pos_vbos_out[0]);	//pos out
		}
		else {
			err = kernel_useSH.setArg(0, cl_vel_vbos_out[0]);	//vel in
			err = kernel_useSH.setArg(1, cl_vel_vbos[0]);		//vel out
			err = kernel_useSH.setArg(8, cl_pos_vbos_out[0]);	//pos in
			err = kernel_useSH.setArg(9, cl_pos_vbos[0]);		//pos out
		}

		err = kernel_useSH.setArg(2, cl_gridStartIndex);
		err = kernel_useSH.setArg(3, cl_gridEndIndex);
		err = kernel_useSH.setArg(4, cl_shEvalX);
		err = kernel_useSH.setArg(5, cl_shEvalY);
		err = kernel_useSH.setArg(6, cl_shEvalZ);
		err = kernel_useSH.setArg(7, cl_simParams);
		err = kernel_useSH.setArg(10, cl::Local(sizeof(cl_float8)*(LOCAL_PREF)));
		err = kernel_useSH.setArg(11, cl::Local(sizeof(cl_float8)*(LOCAL_PREF)));
		err = kernel_useSH.setArg(12, cl::Local(sizeof(cl_float8)*(LOCAL_PREF)));
		err = kernel_useSH.setArg(13, cl_coef0X);
		err = kernel_useSH.setArg(14, cl_coef0Y);
		err = kernel_useSH.setArg(15, cl_coef0Z);
		err = kernel_useSH.setArg(16, cl::Local(sizeof(cl_float)*(LOCAL_PREF)));
		err = kernel_useSH.setArg(17, cl::Local(sizeof(cl_float)*(LOCAL_PREF)));
		err = kernel_useSH.setArg(18, cl::Local(sizeof(cl_float)*(LOCAL_PREF)));
		err = kernel_useSH.setArg(19, cl_shEvalOX);
		err = kernel_useSH.setArg(20, cl_shEvalOY);
		err = kernel_useSH.setArg(21, cl_shEvalOZ);
		err = kernel_useSH.setArg(22, cl_coef0OX);
		err = kernel_useSH.setArg(23, cl_coef0OY);
		err = kernel_useSH.setArg(24, cl_coef0OZ);
		err = kernel_useSH.setArg(25, cl_posObst);
		err = kernel_useSH.setArg(26, cl::Local(sizeof(cl_float4)*(LOCAL_PREF)));
		err = kernel_useSH.setArg(27, numObst);
		err = kernel_useSH.setArg(28, dt);
	}
	catch (cl::Error er){
		log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
	}

	err = queue.enqueueNDRangeKernel(kernel_useSH, cl::NullRange, cl::NDRange(simParams.numBodies), cl::NDRange(LOCAL_PREF), NULL, &event);
	times[5] = eventTimeUs(event);

	releaseGLBuffers();
}
