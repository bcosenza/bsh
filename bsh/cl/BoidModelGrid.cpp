#include "common.h"
#include "BoidModelCL.h"

BoidModelGrid::BoidModelGrid(CLHelper* clHlpr, std::vector<Vec4> pos, std::vector<Vec4> vel, simParams_t* simP)
	: BoidModelGrid(clHlpr, pos, vel, simP, "boidModelGrid_kernel_v3.cl", "Boid Model Grid")
{
}

BoidModelGrid::BoidModelGrid(CLHelper* clHlpr, std::vector<Vec4> pos, std::vector<Vec4> vel, simParams_t* simP,
	const std::string& kernelFile, const char* modelName)
	: BoidModelGridBase(clHlpr, simP, modelName, false)
{
	createCommonBuffers(pos, vel, std::vector<Vec4>(), false);

	size_t array_size_fp4 = num * sizeof(Vec4);
	try{
		cl_pos_out = cl::Buffer(context, CL_MEM_READ_WRITE, array_size_fp4, NULL, &err);
		cl_velocities_out = cl::Buffer(context, CL_MEM_READ_WRITE, array_size_fp4, NULL, &err);
	}
	catch (cl::Error er) {
		log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
	}

	err = queue.enqueueWriteBuffer(cl_velocities_out, CL_TRUE, 0, array_size_fp4, &vel[0], NULL, &event);
	uploadSimParams();

	programBoid    = loadBoidProgram(kernel_path + kernelFile);
	programBitonic = loadProgram(kernel_path + "bitonic_sort.cl");
	loadCommonKernels();

	log("setup complete - simulation is runable");
}

void BoidModelGrid::simulate(float dt){
	acquireGLBuffers();

	runGridHash(cl_pos_vbos[0]);
	clearGridEdges();
	sortGridHash();

	//reorder pos/vel into the plain buffers used by the simulation kernel
	try
	{
		err = kernel_findGridEdgeAndReorder.setArg(2, cl_pos_out);
		err = kernel_findGridEdgeAndReorder.setArg(3, cl_velocities_out);
		err = kernel_findGridEdgeAndReorder.setArg(6, cl_pos_vbos[0]);
		err = kernel_findGridEdgeAndReorder.setArg(7, cl_vel_vbos[0]);
	}
	catch (cl::Error er) {
		log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
	}
	runReorder();

	//do the simulation dance
	try
	{
		err = kernel_simulate.setArg(0, cl_pos_out);
		err = kernel_simulate.setArg(1, cl_pos_vbos[0]);
		err = kernel_simulate.setArg(2, cl_velocities_out);
		err = kernel_simulate.setArg(3, cl_vel_vbos[0]);
		err = kernel_simulate.setArg(4, cl_gridStartIndex);
		err = kernel_simulate.setArg(5, cl_gridEndIndex);
		err = kernel_simulate.setArg(6, cl::Local(sizeof(cl_float4)*(LOCAL_SIZE_VEC4)));
		err = kernel_simulate.setArg(7, cl::Local(sizeof(cl_float4)*(LOCAL_SIZE_VEC4)));
		err = kernel_simulate.setArg(8, cl_simParams);
		err = kernel_simulate.setArg(9, cl_range);
		err = kernel_simulate.setArg(10, dt);
	}
	catch (cl::Error er){
		log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
	}

	err = queue.enqueueNDRangeKernel(kernel_simulate, cl::NullRange, cl::NDRange(simParams.numBodies), cl::NDRange(LOCAL_PREF), NULL, &eventSim);
	times[3] = eventTimeUs(eventSim);

	releaseGLBuffers();
}
