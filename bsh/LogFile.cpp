#include "stdafx.h"
#include "logFile.h"
#include "simParam.h"

LogFile::LogFile(std::string logFileName){
	createLogDir();
	fileName = std::string(LOG_PATH_WIN);
	fileName += "\\" + logFileName + getTimeStamp(true) + ".txt";

	std::cout << "\n" << fileName.data() << std::endl;
}

LogFile::~LogFile(){

}

void LogFile::writeLog(const std::string &entry){
	std::ofstream logFile(fileName, std::ios_base::out | std::ios_base::app);
	logFile << getTimeStamp(false).data() << ": " << entry.data() << std::endl;
}


std::string LogFile::getTimeStamp(bool withoutColon){
	char output[64];
	time_t t = time(0);
	struct tm t_struct;

	localtime_s(&t_struct, &t);
	if (withoutColon){
		strftime(output, sizeof(output), "%Y-%m-%d %H%M", &t_struct);
	}
	else {
		strftime(output, sizeof(output), "%Y-%m-%d %X", &t_struct);
	}
	return std::string(output);
}

bool LogFile::createLogDir(){

	if (CreateDirectory(LOG_PATH_WIN, NULL)){
		printf("Log directory created");
	}
	else {
		printf("Could not create log directory/directory already exists");
	}

	return true;
}

