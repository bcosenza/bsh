#include "stdafx.h"
#include "gfx.h"
#include "Simulation.h"


// Linux entry point 
int main(int argc, char* argv[]) {
    int model = (argc > 1) ? std::atoi(argv[1]) : BOID_SIMPLE;
    Simulation::getInstance().start(model);
    return 0;
}
