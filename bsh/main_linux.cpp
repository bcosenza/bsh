// Linux entry point replacing the Windows WinMain in main.cpp
#include "stdafx.h"
#include "gfx.h"
#include "Simulation.h"

int main(int argc, char* argv[]) {
    int model = (argc > 1) ? std::atoi(argv[1]) : BOID_SIMPLE;
    Simulation::getInstance().start(model);
    return 0;
}
