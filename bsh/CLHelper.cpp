#include "stdafx.h"
#include "CLHelper.h"
#include "gfx.h"

#define LOG (std::string log){logFIle->writeLog(log)}


CLHelper::CLHelper(LogFile* logF){
	deviceUsed = 0;
	logFile = logF;
	log("Starting to create context");

	/*get Available platforms and log information*/
	err = cl::Platform::get(&platformList);
	log(getPlatformInformation());
	log("cl::Platform::get(): " + oclErrorString(err));

	/*Try to chose GPU device from available platforms*/
	err = platformList[0].getDevices(CL_DEVICE_TYPE_GPU, &devices);
	log(getDeviceInformation());
	log("getDevices: " + oclErrorString(err));

	int t = devices.front().getInfo<CL_DEVICE_TYPE>();
	log("type: device: %d CL_DEVICE_TYPE_GPU: %d \n" + t + CL_DEVICE_TYPE_GPU);

	cl_context_properties cprops[] =
	{
		CL_GL_CONTEXT_KHR, (cl_context_properties)wglGetCurrentContext(),
		CL_WGL_HDC_KHR, (cl_context_properties)wglGetCurrentDC(),
		CL_CONTEXT_PLATFORM, (cl_context_properties)(platformList[0])(),
		0
	};

	try{
		context = cl::Context(CL_DEVICE_TYPE_GPU, cprops);
		log("cl context succesfully created");
	}
	catch (cl::Error er) {
		log("ERROR: " + std::string(er.what()) + oclErrorString(er.err()));
	}

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