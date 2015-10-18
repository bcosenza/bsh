#include "stdafx.h"
#include "boidModel.h"
#include "vectorTypes.h"

BoidModelSimple::BoidModelSimple(CLHelper* clHlpr, std::vector<Vec4> pos, std::vector<Vec4> vel, simParams_t* simP) : BoidModel(clHlpr)
{
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
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glDeleteBuffers(1, pos_vbo);
	glDeleteBuffers(1, pos_out_vbo);

	glBindVertexArray(0);
	glDeleteVertexArrays(1, pos_vao);
	glDeleteVertexArrays(1, pos_out_vao);

	delete shader;

	queue.finish();
}

GLuint BoidModelSimple::getPosVAO(){
	if (helper % 2 == 0)
		return pos_out_vao[0];
	else
		return pos_vao[0];
}

GLuint BoidModelSimple::getPosVBO(){
	if (helper % 2 == 0)
		return pos_out_vbo[0];
	else
		return pos_vbo[0];
}

GLuint BoidModelSimple::getVelVBO(){
	if (helper % 2 == 0)
		return vel_out_vbo[0];
	else
		return vel_vbo[0];
}

int BoidModelSimple::getNumBoid(){
	return num;
}


void BoidModelSimple::createVboBindShader(std::vector<Vec4> pos, std::vector<Vec4> vel){

	std::vector<Vec4> newDataColor(num);

	for (int i = 0; i < num; i++){
		newDataColor[i] = BOID_COLOR;
	}

	GLuint id[1];
	size_t array_size = num * sizeof(Vec4);

	shader = new Shader("boidTri.v.glsl", "boidTri.f.glsl", "boidTri.g.glsl");
	GLint vertLoc = glGetAttribLocation(shader->id(), "coord3d");
	GLint colorLoc = glGetAttribLocation(shader->id(), "color");
	GLint velLoc = glGetAttribLocation(shader->id(), "vel3d");

	//------VBO 1--------- (in)
	glGenVertexArrays(1, &pos_vao[0]); // Create our Vertex Array Object  
	glBindVertexArray(pos_vao[0]); // Bind our Vertex Array Object so we can use it  

	pos_vbo[0] = clHelper->createVBO(&pos[0], array_size, GL_ARRAY_BUFFER, GL_DYNAMIC_DRAW);

	glVertexAttribPointer(vertLoc, 4, GL_FLOAT, GL_FALSE, 0, 0); // Set up our vertex attributes pointer
	glEnableVertexAttribArray(vertLoc);

	vel_vbo[0] = clHelper->createVBO(&vel[0], array_size, GL_ARRAY_BUFFER, GL_DYNAMIC_DRAW);

	glVertexAttribPointer(velLoc, 4, GL_FLOAT, GL_FALSE, 0, 0); // Set up our vertex attributes pointer
	glEnableVertexAttribArray(velLoc);

	glGenBuffers(1, &id[0]);
	glBindBuffer(GL_ARRAY_BUFFER, id[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Vec4)* num, &newDataColor[0], GL_STATIC_DRAW);

	glVertexAttribPointer(colorLoc, 4, GL_FLOAT, GL_FALSE, 0, 0); // Set up our vertex attributes pointer  
	glEnableVertexAttribArray(colorLoc);

	glEnableVertexAttribArray(0); // Disable our Vertex Array Object  
	glBindVertexArray(0);

	//------VBO 2--------- (out)
	glGenVertexArrays(1, &pos_out_vao[0]); // Create our Vertex Array Object  
	glBindVertexArray(pos_out_vao[0]); // Bind our Vertex Array Object so we can use it  

	pos_out_vbo[0] = clHelper->createVBO(&pos[0], array_size, GL_ARRAY_BUFFER, GL_DYNAMIC_DRAW);

	glVertexAttribPointer(vertLoc, 4, GL_FLOAT, GL_FALSE, 0, 0); // Set up our vertex attributes pointer
	glEnableVertexAttribArray(vertLoc);

	vel_out_vbo[0] = clHelper->createVBO(&vel[0], array_size, GL_ARRAY_BUFFER, GL_DYNAMIC_DRAW);

	glVertexAttribPointer(velLoc, 4, GL_FLOAT, GL_FALSE, 0, 0); // Set up our vertex attributes pointer
	glEnableVertexAttribArray(velLoc);

	glGenBuffers(1, &id[0]);
	glBindBuffer(GL_ARRAY_BUFFER, id[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Vec4)* num, &newDataColor[0], GL_STATIC_DRAW);

	glVertexAttribPointer(colorLoc, 4, GL_FLOAT, GL_FALSE, 0, 0); // Set up our vertex attributes pointer  
	glEnableVertexAttribArray(colorLoc);

	glEnableVertexAttribArray(0); // Disable our Vertex Array Object  
	glBindVertexArray(0);

}


void BoidModelSimple::createBuffer(std::vector<Vec4> pos, std::vector<Vec4> vel){
	log("Create buffer for usage");

	size_t array_size = num * sizeof(Vec4);


	createVboBindShader(pos, vel);

	//make sure OpenGL is finished before we proceed
	glFinish();

	log("GL VBO Buffer created");
	// create OpenCL buffer from GL VBO
	try
	{
		cl_pos_vbos.push_back(cl::BufferGL(context, CL_MEM_READ_WRITE, pos_vbo[0], &err));
		cl_pos_vbos_out.push_back(cl::BufferGL(context, CL_MEM_READ_WRITE, pos_out_vbo[0], &err));

		//create the OpenCL only arrays
		cl_vel_vbos.push_back(cl::BufferGL(context, CL_MEM_READ_WRITE, vel_vbo[0], &err));
		cl_vel_vbos_out.push_back(cl::BufferGL(context, CL_MEM_READ_WRITE, vel_out_vbo[0], &err));

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

	int pl;

	pl = kernelSource.size();

	try
	{
		cl::Program::Sources source(1,
			std::make_pair(kernelSource.c_str(), pl));
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
	printf("run kernel\n");
	//this will update our system by calculating new velocity and updating the positions of our particles
	//Make sure OpenGL is done using our VBOs
	glFinish();
	// map OpenGL buffer object for writing from OpenCL
	//this passes in the vector of VBO buffer objects (position and color)
	err = queue.enqueueAcquireGLObjects(&cl_pos_vbos, NULL, &event);
	err = queue.enqueueAcquireGLObjects(&cl_pos_vbos_out, NULL, &event);
	err = queue.enqueueAcquireGLObjects(&cl_vel_vbos, NULL, &event);
	err = queue.enqueueAcquireGLObjects(&cl_vel_vbos_out, NULL, &event);
	//printf("acquire: %s\n", oclErrorString(err));
	queue.finish();

	if ((helper++ % 2) == 0){
		try
		{
			err = kernel.setArg(0, cl_pos_vbos[0]); //position vbo
			err = kernel.setArg(1, cl_pos_vbos_out[0]); //position vbo
			err = kernel.setArg(2, cl_vel_vbos[0]);
			err = kernel.setArg(3, cl_vel_vbos_out[0]);
			err = kernel.setArg(5, cl_simParams);
		}
		catch (cl::Error er) {
			log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
		}
	}
	else {
		try
		{
			err = kernel.setArg(1, cl_pos_vbos[0]); //position vbo
			err = kernel.setArg(0, cl_pos_vbos_out[0]); //position vbo
			err = kernel.setArg(3, cl_vel_vbos[0]);
			err = kernel.setArg(2, cl_vel_vbos_out[0]);
			err = kernel.setArg(5, cl_simParams);
		}
		catch (cl::Error er) {
			log("ERROR: " + std::string(er.what()) + clHelper->oclErrorString(er.err()));
		}
	}


	kernel.setArg(4, dt); //pass in the timestep
	//execute the kernel
	err = queue.enqueueNDRangeKernel(kernel, cl::NullRange, cl::NDRange(num), cl::NullRange, NULL, &eventSim);
	queue.finish();


	//Release the VBOs so OpenGL can play with them
	err = queue.enqueueReleaseGLObjects(&cl_pos_vbos, NULL, &event);
	err = queue.enqueueReleaseGLObjects(&cl_pos_vbos_out, NULL, &event);
	err = queue.enqueueReleaseGLObjects(&cl_vel_vbos, NULL, &event);
	err = queue.enqueueReleaseGLObjects(&cl_vel_vbos_out, NULL, &event);
	//printf("release gl: %s\n", oclErrorString(err));
	queue.finish();
}

void BoidModelSimple::render(){
	shader->bind();
	glBindVertexArray(getPosVAO());
	glDrawArrays(GL_POINTS, 0, num);
	glBindVertexArray(0);
	shader->unbind();
}

long BoidModelSimple::getSimulationTime(){
	cl_ulong startTime, endTime;
	eventSim.wait();
	eventSim.getProfilingInfo<cl_ulong>(CL_PROFILING_COMMAND_START, &startTime);
	eventSim.getProfilingInfo<cl_ulong>(CL_PROFILING_COMMAND_END, &endTime);
	return (endTime - startTime) / 1000000;
}

void BoidModelSimple::bindShader(){
	shader->bind();
}

void BoidModelSimple::unbindShader(){
	shader->unbind();
}

Shader* BoidModelSimple::getShader(){
	return shader;
}


std::vector<const char*> BoidModelSimple::getSimTimeDescriptions(){
	std::stringstream strstream;
	strstream.str(std::string());
	strstream << "Simulation time: " << getSimulationTime() << "ms" << "\0";
	stringSimTime = strstream.str();
	simTimeDisc[4] = stringSimTime.c_str();
	return simTimeDisc;
}

void BoidModelSimple::getFollowedBoid(unsigned int* boidIndex, Vec4* pos, Vec4* vel){
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
