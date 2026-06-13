#include "stdafx.h"
#include "gfx.h"
#include "simulation.h"

// Windows entry point
int WINAPI WinMain(HINSTANCE hInstance,
	HINSTANCE hPrevInstance,
	LPSTR    lpCmdLine,
	int       nCmdShow) {

	Simulation::getInstance().start();
	
	return 0;
}