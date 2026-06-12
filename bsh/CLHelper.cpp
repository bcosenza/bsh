#include "common.h"
#include <GL/glx.h>
#include "CLHelper.h"
#include "gfx.h"

#define LOG (std::string log){logFIle->writeLog(log)}


CLHelper::CLHelper(LogFile* logF){
	deviceUsed = 0;
	logFile = logF;
	glInteropEnabled = false;
	log("Starting to create context");

	err = cl::Platform::get(&platformList);
	log(getPlatformInformation());
	log("cl::Platform::get(): " + oclErrorString(err));

	if (platformList.empty()) {
		log("ERROR: No OpenCL platforms found");
		return;
	}

	GLXContext glxCtx = glXGetCurrentContext();
	Display*   glxDpy = glXGetCurrentDisplay();
	bool contextOk = false;

	// Try each platform's GPU devices for GL interop (requires cl_khr_gl_sharing)
	if (glxCtx != nullptr && glxDpy != nullptr) {
		for (size_t pi = 0; pi < platformList.size() && !contextOk; pi++) {
			std::vector<cl::Device> platDevices;
			try {
				platformList[pi].getDevices(CL_DEVICE_TYPE_GPU, &platDevices);
			} catch (cl::Error) {}
			if (platDevices.empty()) continue;

			std::string exts;
			platDevices[0].getInfo(CL_DEVICE_EXTENSIONS, &exts);
			if (exts.find("cl_khr_gl_sharing") == std::string::npos) {
				log("Platform " + std::to_string(pi) + " GPU has no cl_khr_gl_sharing, skipping");
				continue;
			}

			cl_context_properties cprops[] = {
				CL_GL_CONTEXT_KHR,  (cl_context_properties)glxCtx,
				CL_GLX_DISPLAY_KHR, (cl_context_properties)glxDpy,
				CL_CONTEXT_PLATFORM,(cl_context_properties)(platformList[pi])(),
				0
			};
			try {
				context = cl::Context(platDevices, cprops);
				devices = platDevices;
				glInteropEnabled = true;
				contextOk = true;
				log("GL interop context created on platform " + std::to_string(pi));
			} catch (cl::Error er) {
				log("GL interop failed on platform " + std::to_string(pi) + ": " +
				    std::string(er.what()) + oclErrorString(er.err()));
			}
		}
	} else {
		log("GLX context not available, skipping GL interop");
	}

	// Fall back to platform 0, best available device, no GL interop
	if (!contextOk) {
		try {
			err = platformList[0].getDevices(CL_DEVICE_TYPE_GPU, &devices);
		} catch (cl::Error) {}
		if (devices.empty()) {
			try {
				err = platformList[0].getDevices(CL_DEVICE_TYPE_ALL, &devices);
			} catch (cl::Error er) {
				log("ERROR getting any OpenCL device: " + std::string(er.what()));
			}
		}
		if (devices.empty()) {
			log("ERROR: No OpenCL devices found");
			return;
		}
		try {
			context = cl::Context(devices);
			log("OpenCL context (no GL interop) created");
			contextOk = true;
		} catch (cl::Error er) {
			log("ERROR creating OpenCL context: " + std::string(er.what()) + oclErrorString(er.err()));
			return;
		}
	}

	log(getDeviceInformation());
	log("getDevices: " + oclErrorString(err));

	try{
		queue = cl::CommandQueue(context, devices[deviceUsed], CL_QUEUE_PROFILING_ENABLE, &err);
		log("cl command queue succesfully created");
	}
	catch (cl::Error er) {
		log("ERROR: " + std::string(er.what()) + oclErrorString(er.err()));
	}
}

cl::Context CLHelper::getContext(){
	return context;
}

cl::CommandQueue CLHelper::getCmdQueue(){
	return queue;
}

std::vector<cl::Device> CLHelper::getDevices(){
	return devices;
}


GLuint CLHelper::createVBO(const void* data, size_t dataSize, GLenum target, GLenum usage)
{
	GLuint id = 0; // 0 is reserved, glGenBuffersARB() will return non-zero id if success
	
	glGenBuffers(1, &id); // create a vbo
	glBindBuffer(target, id); // activate vbo id to use
	glBufferData(target, dataSize, data, usage); // upload data to video card

	// check data size in VBO is same as input array, if not return 0 and delete VBO
	int bufferSize = 0;
	glGetBufferParameteriv(target, GL_BUFFER_SIZE, &bufferSize);
	if (dataSize != bufferSize)
	{
		glDeleteBuffers(1, &id);
		id = 0;
		printf("[createVBO()] Data size is mismatch with input array\n");
	}
	//this was important for working inside blender!
	//glBindBuffer(target, 0);
	/*
	glEnableVertexAttribArray(0); // Disable our Vertex Array Object  
	glBindVertexArray(0);
	*/

	return id;
}

cl::Memory CLHelper::createFromGLBuffer(GLuint vbo, cl_int* errRet){
	if (glInteropEnabled)
		return cl::BufferGL(context, CL_MEM_READ_WRITE, vbo, errRet);

	// No interop: create a plain CL buffer initialized from the VBO contents
	// and remember which VBO it shadows so releaseGLObjects() can sync back.
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	GLint bufferSize = 0;
	glGetBufferParameteriv(GL_ARRAY_BUFFER, GL_BUFFER_SIZE, &bufferSize);

	std::vector<char> data(bufferSize);
	glGetBufferSubData(GL_ARRAY_BUFFER, 0, bufferSize, data.data());
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	cl::Buffer buffer(context, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, (size_t)bufferSize, data.data(), errRet);
	shadowedVBO[buffer()] = std::make_pair(vbo, (size_t)bufferSize);
	return buffer;
}

cl_int CLHelper::acquireGLObjects(cl::CommandQueue& queue, std::vector<cl::Memory>* objects, cl::Event* event){
	if (glInteropEnabled)
		return queue.enqueueAcquireGLObjects(objects, NULL, event);
	// No interop: GL never writes to these VBOs, so the CL buffers are already current.
	return CL_SUCCESS;
}

cl_int CLHelper::releaseGLObjects(cl::CommandQueue& queue, std::vector<cl::Memory>* objects, cl::Event* event){
	if (glInteropEnabled)
		return queue.enqueueReleaseGLObjects(objects, NULL, event);

	// No interop: copy the CL buffer contents back into the shadowed VBOs for rendering
	cl_int result = CL_SUCCESS;
	for (size_t i = 0; i < objects->size(); i++){
		std::map<cl_mem, std::pair<GLuint, size_t> >::iterator it = shadowedVBO.find((*objects)[i]());
		if (it == shadowedVBO.end()) continue;

		GLuint vbo = it->second.first;
		size_t size = it->second.second;
		std::vector<char> data(size);
		// use the C API: wrapping the raw cl_mem in a cl::Buffer would take
		// ownership and release it on destruction
		result = clEnqueueReadBuffer(queue(), (*objects)[i](), CL_TRUE, 0, size, data.data(), 0, NULL, NULL);
		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		glBufferSubData(GL_ARRAY_BUFFER, 0, size, data.data());
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}
	return result;
}

std::string CLHelper::getPlatformInformation(){
	std::string info;
	std::string buffer;

	if (platformList.size() > 0){
		info += "\n\t----------------------Platform(s) Information----------------------";
		info += "\n\tOpenCL platforms found on this PC: " + std::to_string(platformList.size());

		for (int i = 0; i < platformList.size(); i++){
			platformList[i].getInfo(CL_PLATFORM_NAME, &buffer);
			info += "\n\tPlatform Name: " + buffer;
			platformList[i].getInfo(CL_PLATFORM_VERSION, &buffer);
			info += "\n\tPlatform Version: " + buffer;
			platformList[i].getInfo(CL_PLATFORM_VENDOR, &buffer);
			info += "\n\tPlatform Vendor: " + buffer;
		}
		info += "\n\t--------------------------------------------------------------------";
	}
	else {
		info.append("No OpenCL platforms found");
	}

	return info;
}

//rewrite code to chose freely from platform!
std::string CLHelper::getDeviceInformation(){
	std::string info;
	std::string buffer;

	if (devices.size() > 0){
		info += "\n\t----------------------Device(s) Information----------------------";
		info += "\n\tOpenCL devices found on this PC: " + std::to_string(platformList.size());

		for (int i = 0; i < devices.size(); i++){
			devices[i].getInfo(CL_DEVICE_NAME, &buffer);
			info += "\n\tDevice Name: " + buffer;
			devices[i].getInfo(CL_DEVICE_VENDOR, &buffer);
			info += "\n\tDevice Vendor: " + buffer;
			devices[i].getInfo(CL_DEVICE_VERSION, &buffer);
			info += "\n\tDevice Version: " + buffer;
			devices[i].getInfo(CL_DEVICE_EXTENSIONS, &buffer);
			info += "\n\tDevice Extensions: " + buffer;
			devices[i].getInfo(CL_DEVICE_MAX_WORK_ITEM_SIZES, &buffer);
			info += "\n\tDevice Max Number Work Item Sizes: " + buffer;
		}
		info += "\n\t--------------------------------------------------------------------";
	}
	else {
		info.append("No OpenCL platforms found");
	}

	return info;
}

std::string CLHelper::oclErrorString(cl_int error) const{

	static const char* errorString[] = {
		"CL_SUCCESS",
		"CL_DEVICE_NOT_FOUND",
		"CL_DEVICE_NOT_AVAILABLE",
		"CL_COMPILER_NOT_AVAILABLE",
		"CL_MEM_OBJECT_ALLOCATION_FAILURE",
		"CL_OUT_OF_RESOURCES",
		"CL_OUT_OF_HOST_MEMORY",
		"CL_PROFILING_INFO_NOT_AVAILABLE",
		"CL_MEM_COPY_OVERLAP",
		"CL_IMAGE_FORMAT_MISMATCH",
		"CL_IMAGE_FORMAT_NOT_SUPPORTED",
		"CL_BUILD_PROGRAM_FAILURE",
		"CL_MAP_FAILURE",
		"",
		"",
		"",
		"",
		"",
		"",
		"",
		"",
		"",
		"",
		"",
		"",
		"",
		"",
		"",
		"",
		"",
		"CL_INVALID_VALUE",
		"CL_INVALID_DEVICE_TYPE",
		"CL_INVALID_PLATFORM",
		"CL_INVALID_DEVICE",
		"CL_INVALID_CONTEXT",
		"CL_INVALID_QUEUE_PROPERTIES",
		"CL_INVALID_COMMAND_QUEUE",
		"CL_INVALID_HOST_PTR",
		"CL_INVALID_MEM_OBJECT",
		"CL_INVALID_IMAGE_FORMAT_DESCRIPTOR",
		"CL_INVALID_IMAGE_SIZE",
		"CL_INVALID_SAMPLER",
		"CL_INVALID_BINARY",
		"CL_INVALID_BUILD_OPTIONS",
		"CL_INVALID_PROGRAM",
		"CL_INVALID_PROGRAM_EXECUTABLE",
		"CL_INVALID_KERNEL_NAME",
		"CL_INVALID_KERNEL_DEFINITION",
		"CL_INVALID_KERNEL",
		"CL_INVALID_ARG_INDEX",
		"CL_INVALID_ARG_VALUE",
		"CL_INVALID_ARG_SIZE",
		"CL_INVALID_KERNEL_ARGS",
		"CL_INVALID_WORK_DIMENSION",
		"CL_INVALID_WORK_GROUP_SIZE",
		"CL_INVALID_WORK_ITEM_SIZE",
		"CL_INVALID_GLOBAL_OFFSET",
		"CL_INVALID_EVENT_WAIT_LIST",
		"CL_INVALID_EVENT",
		"CL_INVALID_OPERATION",
		"CL_INVALID_GL_OBJECT",
		"CL_INVALID_BUFFER_SIZE",
		"CL_INVALID_MIP_LEVEL",
		"CL_INVALID_GLOBAL_WORK_SIZE",
	};

	const int errorCount = sizeof(errorString) / sizeof(errorString[0]);

	const int index = -error;

	return std::string((index >= 0 && index < errorCount) ? errorString[index] : "");
}