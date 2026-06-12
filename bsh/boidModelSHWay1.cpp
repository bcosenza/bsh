#include "stdafx.h"
#include "BoidModel.h"

BoidModelSHWay1::BoidModelSHWay1(CLHelper* clHlpr, std::vector<Vec4> pos, std::vector<Vec4> vel,
	std::vector<Vec4> goal, std::vector<Vec4> color, simParams_t* simP)
	: BoidModelGridBase(clHlpr, simP, "Boid Model SH Way 1", true)
{
	addTimingRow("Eval SH time", 4);
	addTimingRow("Use SH time", 5);

	createCommonBuffers(pos, vel, color, true);
	createGoalBuffers(goal);

	//per cell SH coefficients
	size_t array_size_fp8 = 2 * simParams.numCells * sizeof(Vec4);
	size_t array_size_fp = simParams.numCells * sizeof(float);
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

	programBoid    = loadBoidProgram(kernel_path + "boidModelSHWay1_kernel_v1.cl");
	programBitonic = loadProgram(kernel_path + "bitonic_sort.cl");
	loadCommonKernels();

	try{
		kernel_evalSH = cl::Kernel(programBoid, "evalSH", &err);
#if USE_SH_FOR_PATH
	#if USE_LOOKAHEAD
		kernel_useSH = cl::Kernel(programBoid, "useSHLookahead", &err);
	#else
		kernel_useSH = cl::Kernel(programBoid, "useSH", &err);
	#endif
#else
		kernel_useSH = cl::Kernel(programBoid, "dontUseSH", &err);
#endif
	}
	catch (cl::Error er) {
		log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
	}

	log("setup complete - simulation is runable");
}

void BoidModelSHWay1::simulate(float dt){
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

	//aggregate the SH coefficients per cell
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
		err = kernel_evalSH.setArg(7, cl::Local(sizeof(cl_float)*(LOCAL_PREF)));
		err = kernel_evalSH.setArg(8, cl::Local(sizeof(cl_float)*(LOCAL_PREF)));
		err = kernel_evalSH.setArg(9, cl::Local(sizeof(cl_float)*(LOCAL_PREF)));
		err = kernel_evalSH.setArg(10, cl::Local(sizeof(cl_float8)*(LOCAL_PREF)));
		err = kernel_evalSH.setArg(11, cl::Local(sizeof(cl_float8)*(LOCAL_PREF)));
		err = kernel_evalSH.setArg(12, cl::Local(sizeof(cl_float8)*(LOCAL_PREF)));
		err = kernel_evalSH.setArg(13, cl_gridStartIndex);
		err = kernel_evalSH.setArg(14, cl_gridEndIndex);
	}
	catch (cl::Error er){
		log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
	}

	err = queue.enqueueNDRangeKernel(kernel_evalSH, cl::NullRange, cl::NDRange(simParams.numCells * LOCAL_PREF), cl::NDRange(LOCAL_PREF), NULL, &event);
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

	//long range avoidance with the per cell SH coefficients
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
		err = kernel_useSH.setArg(19, dt);
	}
	catch (cl::Error er){
		log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
	}

	err = queue.enqueueNDRangeKernel(kernel_useSH, cl::NullRange, cl::NDRange(simParams.numBodies), cl::NDRange(LOCAL_PREF), NULL, &event);
	times[5] = eventTimeUs(event);

	releaseGLBuffers();
}
