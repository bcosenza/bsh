#include "common.h"
#include "gfx.h"
#include "Simulation.h"
#include <cstring>
#include <cstdio>
#include <cstdlib>

static void usage(const char* prog){
    std::fprintf(stderr,
        "usage: %s [scene] [-scene n] [-sh dirac|vmf] [-kappa k] [-timing t]\n"
        "  scene, -scene n  scene to start in (keys 1-7)\n"
        "  -sh dirac|vmf    SH projection kernel (switch at runtime with 'p')\n"
        "  -kappa k         von Mises-Fisher concentration (default %.2f; '[' / ']' at runtime)\n"
        "  -timing t        run t steps (after %d warm-up steps) with a fixed time step,\n"
        "                   print the mean/min/max time per step and per phase, then quit\n",
        prog, SH_VMF_KAPPA, TIMING_WARMUP_STEPS);
}

// Linux entry point
int main(int argc, char* argv[]) {
    RunOptions opt;
    for (int i = 1; i < argc; i++){
        const char* a = argv[i];
        const bool hasValue = (i + 1 < argc);
        if (!std::strcmp(a, "-timing") && hasValue)      opt.timingSteps = std::atoi(argv[++i]);
        else if (!std::strcmp(a, "-scene") && hasValue)  opt.model = std::atoi(argv[++i]);
        else if (!std::strcmp(a, "-kappa") && hasValue)  opt.vmfK = (float)std::atof(argv[++i]);
        else if (!std::strcmp(a, "-sh") && hasValue){
            const char* m = argv[++i];
            if (!std::strcmp(m, "dirac"))    opt.shMode = SH_PROJECTION_DIRAC;
            else if (!std::strcmp(m, "vmf")) opt.shMode = SH_PROJECTION_VMF;
            else { usage(argv[0]); return 1; }
        }
        else if (a[0] != '-')                            opt.model = std::atoi(a);   // positional scene
        else { usage(argv[0]); return 1; }
    }
    if (opt.vmfK <= 0.0f){ usage(argv[0]); return 1; }

    Simulation::getInstance().start(opt);
    return 0;
}
