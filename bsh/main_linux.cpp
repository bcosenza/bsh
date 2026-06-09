// Linux entry point replacing the Windows WinMain in main.cpp
#include "stdafx.h"
#include "gfx.h"
#include "simulation.h"

int main(int argc, char* argv[]) {
    Simulation::getInstance().start();
    return 0;
}
