// Copyright (c) 2015, Biagio Cosenza.
// Technische Universitaet Berlin. All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#ifndef _CLHELPER_H_
#define _CLHELPER_H_

#include "stdafx.h"
#include "logFile.h"
#include <map>
#include <utility>

/*
	OpenCL helper class for context, queue and query for a device.
*/
class CLHelper
{

public:
	CLHelper(LogFile* log);

	cl::Context getContext();
	cl::CommandQueue getCmdQueue();
	std::vector<cl::Device> getDevices();
	bool hasGLInterop() const { return glInteropEnabled; }

	/*
		Creates VBO on current OpenGL context, shared with OpenCL
		Note: Does NOT unbind the buffer object!
	*/
	GLuint createVBO(const void* data, size_t dataSize, GLenum target, GLenum usage);

	/*
		Wraps a GL VBO as an OpenCL buffer. With GL interop this is a
		cl::BufferGL; without interop it is a plain cl::Buffer initialized
		from the VBO contents and synced back by releaseGLObjects().
	*/
	cl::Memory createFromGLBuffer(GLuint vbo, cl_int* err);

	/*
		Interop-aware replacements for enqueueAcquireGLObjects and
		enqueueReleaseGLObjects. Without interop, acquire is a no-op and
		release copies each buffer's contents back into its VBO.
	*/
	cl_int acquireGLObjects(cl::CommandQueue& queue, std::vector<cl::Memory>* objects, cl::Event* event);
	cl_int releaseGLObjects(cl::CommandQueue& queue, std::vector<cl::Memory>* objects, cl::Event* event);

	std::string getPlatformInformation();
	std::string getDeviceInformation();
	std::string oclErrorString(cl_int error) const;

	inline void log(std::string entry){
		logFile->writeLog(entry);
	}

private:
	cl::Context context;
	cl::CommandQueue queue;
	cl::Program program;

	unsigned int deviceUsed;
	std::vector<cl::Device> devices;
	std::vector<cl::Platform> platformList;

	cl_int err;
	bool glInteropEnabled;

	// no-interop mode: maps each plain CL buffer to the VBO it shadows and its size
	std::map<cl_mem, std::pair<GLuint, size_t> > shadowedVBO;

	LogFile* logFile;
};

#endif