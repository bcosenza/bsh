#include "common.h"
#include "BoidModelCL.h"
#include "vectorTypes.h"

BoidModelSimple::BoidModelSimple(CLHelper* clHlpr, std::vector<Vec4> pos, std::vector<Vec4> vel, simParams_t* simP) : BoidModelSimpleView()
{
	clHelper = clHlpr;

	simTimeDisc = std::vector<const char*>(5);
	simTimeDisc[0] = "Boid Model Simple";
	simTimeDisc[1] = "OpenCL Simulation Times:";
	simTimeDisc[2] = "";
	simTimeDisc[3] = "";
	simTimeDisc[4] = "";

	simParams = *simP;
	context = clHelper->getContext();
	queue = clHelper->getCmdQueue();
	devices = clHelper->getDevices();

	num = simParams.numBodies;

	createBuffer(pos, vel);
	loadData();

	loadProgram(kernel_path + "boidModelSimple_kernel_v2.cl");

	loadKernel();
	log("setup complete - simulation is runable");
}

BoidModelSimple::~BoidModelSimple(){
	destroyGLResources();
	queue.finish();
}


void BoidModelSimple::createBuffer(std::vector<Vec4> pos, std::vector<Vec4> vel){
	log("Create buffer for usage");

	size_t array_size = num * sizeof(Vec4);

	createVboBindShader(pos, vel);

	//make sure OpenGL is finished before we proceed
	glFinish();

	log("GL VBO Buffer created");

	useGLInterop = clHelper->hasGLInterop();

	try {
		if (useGLInterop) {
			cl_pos_vbos.push_back(cl::BufferGL(context, CL_MEM_READ_WRITE, pos_vbo[0], &err));
			cl_pos_vbos_out.push_back(cl::BufferGL(context, CL_MEM_READ_WRITE, pos_out_vbo[0], &err));
			cl_vel_vbos.push_back(cl::BufferGL(context, CL_MEM_READ_WRITE, vel_vbo[0], &err));
			cl_vel_vbos_out.push_back(cl::BufferGL(context, CL_MEM_READ_WRITE, vel_out_vbo[0], &err));
			log("CL/GL shared buffers created");
		} else {
			cl_pos_plain     = cl::Buffer(context, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, array_size, (void*)pos.data(), &err);
			cl_pos_out_plain = cl::Buffer(context, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, array_size, (void*)pos.data(), &err);
			cl_vel_plain     = cl::Buffer(context, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, array_size, (void*)vel.data(), &err);
			cl_vel_out_plain = cl::Buffer(context, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, array_size, (void*)vel.data(), &err);
			log("CL plain buffers created (no GL interop)");
		}
		cl_simParams = cl::Buffer(context, CL_MEM_READ_ONLY, sizeof(simParams_t), NULL, &err);
	}
	catch (cl::Error er) {
		log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
	}
}


void BoidModelSimple::loadProgram(const std::string &filename){
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

	try
	{
	  cl::Program::Sources source;
	  source.push_back(kernelSource);
	  program = cl::Program(context, source);
	}
	catch (cl::Error er) {
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
}


void BoidModelSimple::loadKernel(){
	try{
		kernel = cl::Kernel(program, "boidKernel", &err);
	}
	catch (cl::Error er) {
		log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
	}

}


void BoidModelSimple::loadData(){
	err = queue.enqueueWriteBuffer(cl_simParams, CL_TRUE, 0, sizeof(simParams_t), &simParams, NULL, &event);
	queue.finish();
}

void BoidModelSimple::simulate(float dt){
	size_t array_size = num * sizeof(Vec4);

	if (useGLInterop) {
		glFinish();
		err = queue.enqueueAcquireGLObjects(&cl_pos_vbos, NULL, &event);
		err = queue.enqueueAcquireGLObjects(&cl_pos_vbos_out, NULL, &event);
		err = queue.enqueueAcquireGLObjects(&cl_vel_vbos, NULL, &event);
		err = queue.enqueueAcquireGLObjects(&cl_vel_vbos_out, NULL, &event);
		queue.finish();
	}

	bool pingPong = ((helper++ % 2) == 0);

	try {
		if (pingPong) {
			if (useGLInterop) {
				err = kernel.setArg(0, cl_pos_vbos[0]);
				err = kernel.setArg(1, cl_pos_vbos_out[0]);
				err = kernel.setArg(2, cl_vel_vbos[0]);
				err = kernel.setArg(3, cl_vel_vbos_out[0]);
			} else {
				err = kernel.setArg(0, cl_pos_plain);
				err = kernel.setArg(1, cl_pos_out_plain);
				err = kernel.setArg(2, cl_vel_plain);
				err = kernel.setArg(3, cl_vel_out_plain);
			}
		} else {
			if (useGLInterop) {
				err = kernel.setArg(1, cl_pos_vbos[0]);
				err = kernel.setArg(0, cl_pos_vbos_out[0]);
				err = kernel.setArg(3, cl_vel_vbos[0]);
				err = kernel.setArg(2, cl_vel_vbos_out[0]);
			} else {
				err = kernel.setArg(1, cl_pos_plain);
				err = kernel.setArg(0, cl_pos_out_plain);
				err = kernel.setArg(3, cl_vel_plain);
				err = kernel.setArg(2, cl_vel_out_plain);
			}
		}
		err = kernel.setArg(5, cl_simParams);
	} catch (cl::Error er) {
		log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
	}

	kernel.setArg(4, dt);
	err = queue.enqueueNDRangeKernel(kernel, cl::NullRange, cl::NDRange(num), cl::NullRange, NULL, &eventSim);
	queue.finish();

	if (useGLInterop) {
		err = queue.enqueueReleaseGLObjects(&cl_pos_vbos, NULL, &event);
		err = queue.enqueueReleaseGLObjects(&cl_pos_vbos_out, NULL, &event);
		err = queue.enqueueReleaseGLObjects(&cl_vel_vbos, NULL, &event);
		err = queue.enqueueReleaseGLObjects(&cl_vel_vbos_out, NULL, &event);
		queue.finish();
	} else {
		// Copy CL output back to the GL VBO that will be rendered next frame
		std::vector<Vec4> tmpBuf(num);
		if (pingPong) {
			// kernel wrote to cl_pos_out_plain / cl_vel_out_plain
			queue.enqueueReadBuffer(cl_pos_out_plain, CL_TRUE, 0, array_size, tmpBuf.data());
			glBindBuffer(GL_ARRAY_BUFFER, pos_out_vbo[0]);
			glBufferSubData(GL_ARRAY_BUFFER, 0, array_size, tmpBuf.data());
			queue.enqueueReadBuffer(cl_vel_out_plain, CL_TRUE, 0, array_size, tmpBuf.data());
			glBindBuffer(GL_ARRAY_BUFFER, vel_out_vbo[0]);
			glBufferSubData(GL_ARRAY_BUFFER, 0, array_size, tmpBuf.data());
		} else {
			// kernel wrote to cl_pos_plain / cl_vel_plain
			queue.enqueueReadBuffer(cl_pos_plain, CL_TRUE, 0, array_size, tmpBuf.data());
			glBindBuffer(GL_ARRAY_BUFFER, pos_vbo[0]);
			glBufferSubData(GL_ARRAY_BUFFER, 0, array_size, tmpBuf.data());
			queue.enqueueReadBuffer(cl_vel_plain, CL_TRUE, 0, array_size, tmpBuf.data());
			glBindBuffer(GL_ARRAY_BUFFER, vel_vbo[0]);
			glBufferSubData(GL_ARRAY_BUFFER, 0, array_size, tmpBuf.data());
		}
	}
}

long BoidModelSimple::getSimulationTime(){
	cl_ulong startTime, endTime;
	eventSim.wait();
	eventSim.getProfilingInfo<cl_ulong>(CL_PROFILING_COMMAND_START, &startTime);
	eventSim.getProfilingInfo<cl_ulong>(CL_PROFILING_COMMAND_END, &endTime);
	return (endTime - startTime) / 1000000;
}

std::vector<const char*> BoidModelSimple::getSimTimeDescriptions(){
	std::stringstream strstream;
	strstream.str(std::string());
	strstream << "Simulation time: " << getSimulationTime() << "ms" << "\0";
	stringSimTime = strstream.str();
	simTimeDisc[4] = stringSimTime.c_str();
	return simTimeDisc;
}
