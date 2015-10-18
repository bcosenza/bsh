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

	/*
		Creates VBO on current OpenGL context, shared with OpenCL
		Note: Does NOT unbind the buffer object!
	*/
	GLuint createVBO(const void* data, size_t dataSize, GLenum target, GLenum usage);

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

	LogFile* logFile;
};

#endif