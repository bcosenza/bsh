#include "stdafx.h"
#include "logFile.h"
#include "SimParam.h"
#include <sys/stat.h>
#include <algorithm>

LogFile::LogFile(std::string logFileName){
	createLogDir();
	std::string name = logFileName + getTimeStamp(true) + ".txt";
	std::replace(name.begin(), name.end(), ' ', '_');
	fileName = std::string(LOG_PATH_WIN) + "/" + name;

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

	localtime_r(&t, &t_struct);
	if (withoutColon){
		strftime(output, sizeof(output), "%Y-%m-%d %H%M", &t_struct);
	}
	else {
		strftime(output, sizeof(output), "%Y-%m-%d %X", &t_struct);
	}
	return std::string(output);
}

bool LogFile::createLogDir(){

	if (mkdir(LOG_PATH_WIN, 0755) == 0){
		printf("Log directory created");
	}
	else {
		printf("Could not create log directory/directory already exists");
	}

	return true;
}

