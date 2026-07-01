#include "common.h"
#include "BoidModelCL.h"

BoidModelSH::BoidModelSH(CLHelper* clHlpr, std::vector<Vec4> pos, std::vector<Vec4> vel, simParams_t* simP)
	: BoidModelSH(clHlpr, pos, vel, simP, "boidModelSH_kernel_v1.cl", "Boid Model SH")
{
}

BoidModelSH::BoidModelSH(CLHelper* clHlpr, std::vector<Vec4> pos, std::vector<Vec4> vel, simParams_t* simP,
	const std::string& kernelFile, const char* modelName)
	: BoidModelGridBase(clHlpr, simP, modelName, true)
{
	addTimingRow("Sum vel. time", 4);
	addTimingRow("SH total time", 5);

	createCommonBuffers(pos, vel, std::vector<Vec4>(), false);

	try{
		cl_sumVel = cl::Buffer(context, CL_MEM_READ_WRITE, simParams.numCells * sizeof(Vec4), NULL, &err);
	}
	catch (cl::Error er) {
		log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
	}

	uploadSimParams();

	programBoid    = loadBoidProgram(kernel_path + kernelFile);
	programBitonic = loadProgram(kernel_path + "bitonic_sort.cl");
	loadCommonKernels();

	try{
		kernel_sumVelSH = cl::Kernel(programBoid, "sumVelSH", &err);
		kernel_useSH = cl::Kernel(programBoid, "useSH", &err);
	}
	catch (cl::Error er) {
		log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
	}

	log("setup complete - simulation is runable");
}

void BoidModelSH::simulate(float dt){
	counter = !counter;

	acquireGLBuffers();

	runGridHash(counter ? cl_pos_vbos[0] : cl_pos_vbos_out[0]);
	clearGridEdges();
	sortGridHash();

	//reorder pos/vel into the VBO set that is not rendered this frame
	try
	{
		if (counter){
			err = kernel_findGridEdgeAndReorder.setArg(2, cl_pos_vbos_out[0]);	//pos out ordered
			err = kernel_findGridEdgeAndReorder.setArg(3, cl_vel_vbos_out[0]);	//vel out ordered
			err = kernel_findGridEdgeAndReorder.setArg(6, cl_pos_vbos[0]);		//pos in unordered
			err = kernel_findGridEdgeAndReorder.setArg(7, cl_vel_vbos[0]);		//vel in unordered
		}
		else {
			err = kernel_findGridEdgeAndReorder.setArg(2, cl_pos_vbos[0]);		//pos out
			err = kernel_findGridEdgeAndReorder.setArg(3, cl_vel_vbos[0]);		//vel out
			err = kernel_findGridEdgeAndReorder.setArg(6, cl_pos_vbos_out[0]);	//pos in
			err = kernel_findGridEdgeAndReorder.setArg(7, cl_vel_vbos_out[0]);	//vel in
		}
	}
	catch (cl::Error er) {
		log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
	}
	runReorder();

	//sum the velocities of the boids in every cell
	try
	{
		if (counter)
			err = kernel_sumVelSH.setArg(0, cl_vel_vbos_out[0]);
		else
			err = kernel_sumVelSH.setArg(0, cl_vel_vbos[0]);

		err = kernel_sumVelSH.setArg(1, cl_gridStartIndex);
		err = kernel_sumVelSH.setArg(2, cl_gridEndIndex);
		err = kernel_sumVelSH.setArg(3, cl_sumVel);
		err = kernel_sumVelSH.setArg(4, cl::Local(sizeof(cl_float4)*(2 * LOCAL_PREF)));
	}
	catch (cl::Error er){
		log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
	}

	err = queue.enqueueNDRangeKernel(kernel_sumVelSH, cl::NullRange, cl::NDRange(LOCAL_PREF * simParams.numCells), cl::NDRange(LOCAL_PREF), NULL, &event);
	times[4] = eventTimeUs(event);

	//local flocking on the reordered buffers
	try
	{
		if (counter){
			err = kernel_simulate.setArg(0, cl_pos_vbos_out[0]);	//pos in
			err = kernel_simulate.setArg(1, cl_pos_vbos[0]);		//pos out
			err = kernel_simulate.setArg(2, cl_vel_vbos_out[0]);	//vel in
			err = kernel_simulate.setArg(3, cl_vel_vbos[0]);		//vel out
		}
		else{
			err = kernel_simulate.setArg(0, cl_pos_vbos[0]);		//pos in
			err = kernel_simulate.setArg(1, cl_pos_vbos_out[0]);	//pos out
			err = kernel_simulate.setArg(2, cl_vel_vbos[0]);		//vel in
			err = kernel_simulate.setArg(3, cl_vel_vbos_out[0]);	//vel out
		}

		err = kernel_simulate.setArg(4, cl_gridStartIndex);
		err = kernel_simulate.setArg(5, cl_gridEndIndex);
		err = kernel_simulate.setArg(6, cl::Local(sizeof(cl_float4)*(LOCAL_PREF)));
		err = kernel_simulate.setArg(7, cl::Local(sizeof(cl_float4)*(LOCAL_PREF)));
		err = kernel_simulate.setArg(8, cl_simParams);
		err = kernel_simulate.setArg(9, cl_range);
		err = kernel_simulate.setArg(10, dt);
	}
	catch (cl::Error er){
		log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
	}

	err = queue.enqueueNDRangeKernel(kernel_simulate, cl::NullRange, cl::NDRange(LOCAL_PREF * simParams.numCells), cl::NDRange(LOCAL_PREF), NULL, &eventSim);
	times[3] = eventTimeUs(eventSim);

	//long range collision avoidance with the per cell SH coefficients
	try
	{
		if (counter){
			err = kernel_useSH.setArg(0, cl_vel_vbos[0]);		//vel in
			err = kernel_useSH.setArg(1, cl_vel_vbos_out[0]);	//vel out
			err = kernel_useSH.setArg(6, cl_pos_vbos[0]);		//pos in
			err = kernel_useSH.setArg(7, cl_pos_vbos_out[0]);	//pos out
		}
		else {
			err = kernel_useSH.setArg(0, cl_vel_vbos_out[0]);	//vel in
			err = kernel_useSH.setArg(1, cl_vel_vbos[0]);		//vel out
			err = kernel_useSH.setArg(6, cl_pos_vbos_out[0]);	//pos in
			err = kernel_useSH.setArg(7, cl_pos_vbos[0]);		//pos out
		}

		err = kernel_useSH.setArg(2, cl_gridStartIndex);
		err = kernel_useSH.setArg(3, cl_gridEndIndex);
		err = kernel_useSH.setArg(4, cl_sumVel);
		err = kernel_useSH.setArg(5, cl_simParams);
		err = kernel_useSH.setArg(8, cl::Local(sizeof(cl_float4)*(LOCAL_PREF)));
		setUseSHFinalArgs(dt);
	}
	catch (cl::Error er){
		log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
	}

	err = queue.enqueueNDRangeKernel(kernel_useSH, cl::NullRange, cl::NDRange(LOCAL_PREF * simParams.numCells), cl::NDRange(LOCAL_PREF), NULL, &event);
	times[5] = eventTimeUs(event);

	releaseGLBuffers();
}

void BoidModelSH::setUseSHFinalArgs(float dt){
	err = kernel_useSH.setArg(9, dt);
}
