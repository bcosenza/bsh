#include "stdafx.h"
#include "gfx.h"
#include "simulation.h"


int WINAPI WinMain(HINSTANCE hInstance,
	HINSTANCE hPrevInstance,
	LPSTR    lpCmdLine,
	int       nCmdShow) {

	Simulation::getInstance().start();
	
	return 0;
}