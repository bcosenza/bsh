// Copyright (c) 2015, Biagio Cosenza.
// Technische Universitaet Berlin. All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#ifndef _LOGFILE_H_
#define _LOGFILE_H_

#include "stdafx.h"

/* Simple logger. Logfile path is set in simParam.h */
class LogFile
{

public:
	LogFile(std::string fileName);
	~LogFile();

	// write to log file
	void writeLog(const std::string &entry);

private:
	std::string fileName;

	// get a timestamp when the file is created	
	std::string getTimeStamp(bool withoutColon);

	// create log directory if it doesn't exist	
	bool createLogDir();
};

#endif