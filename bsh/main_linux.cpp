#include "common.h"
#include "gfx.h"
#include "Simulation.h"
#include <cstring>
#include <cstdio>
#include <cstdlib>

static void usage(const char* prog){
    std::fprintf(stderr,
        "usage: %s [scene] [-scene n] [-sh dirac|vmf|exact] [-kappa k] [-timing t]\n"
        "          [-accuracy t] [-out file.csv] [-seed s] [-hspread kh]\n"
        "  scene, -scene n  scene to start in (keys 1-9)\n"
        "  -sh dirac|vmf|exact\n"
        "                   SH projection kernel (switch at runtime with 'p'); 'exact' drives\n"
        "                   the accuracy scenes 8 and 9 with the brute-force reference\n"
        "  -kappa k         von Mises-Fisher concentration (default %.2f; '[' / ']' at runtime)\n"
        "  -timing t        run t steps (after %d warm-up steps) with a fixed time step,\n"
        "                   print the mean/min/max time per step and per phase, then quit\n"
        "  -accuracy t      scenes 8 and 9: run t fixed-dt steps comparing the Dirac and vMF\n"
        "                   SH steering with the brute-force reference on the same states,\n"
        "                   print a per-lane table, append the rows to the CSV, then quit\n"
        "  -out file.csv    accuracy CSV (default accuracy.csv; header written if new)\n"
        "  -seed s          random seed of the initial placement (accuracy default %d)\n"
        "  -hspread kh      lane scenes: vMF noise of the initial headings, concentration kh\n"
        "                   (0 = none, the default; smaller = more spread)\n",
        prog, SH_VMF_KAPPA, TIMING_WARMUP_STEPS, ACCURACY_DEFAULT_SEED);
}

// Linux entry point
int main(int argc, char* argv[]) {
    RunOptions opt;
    for (int i = 1; i < argc; i++){
        const char* a = argv[i];
        const bool hasValue = (i + 1 < argc);
        if (!std::strcmp(a, "-timing") && hasValue)        opt.timingSteps = std::atoi(argv[++i]);
        else if (!std::strcmp(a, "-accuracy") && hasValue) opt.accuracySteps = std::atoi(argv[++i]);
        else if (!std::strcmp(a, "-out") && hasValue)      opt.accuracyOut = argv[++i];
        else if (!std::strcmp(a, "-seed") && hasValue)     opt.seed = std::atoi(argv[++i]);
        else if (!std::strcmp(a, "-hspread") && hasValue)  opt.headingSpread = (float)std::atof(argv[++i]);
        else if (!std::strcmp(a, "-scene") && hasValue)    opt.model = std::atoi(argv[++i]);
        else if (!std::strcmp(a, "-kappa") && hasValue)    opt.vmfK = (float)std::atof(argv[++i]);
        else if (!std::strcmp(a, "-sh") && hasValue){
            const char* m = argv[++i];
            if (!std::strcmp(m, "dirac"))      opt.shMode = SH_PROJECTION_DIRAC;
            else if (!std::strcmp(m, "vmf"))   opt.shMode = SH_PROJECTION_VMF;
            else if (!std::strcmp(m, "exact")) opt.shMode = SH_PROJECTION_EXACT;
            else { usage(argv[0]); return 1; }
        }
        else if (a[0] != '-')                              opt.model = std::atoi(a);   // positional scene
        else { usage(argv[0]); return 1; }
    }
    if (opt.vmfK <= 0.0f || opt.headingSpread < 0.0f){ usage(argv[0]); return 1; }
    if (opt.timingSteps > 0 && opt.accuracySteps > 0){
        std::fprintf(stderr, "-timing and -accuracy cannot be combined\n");
        return 1;
    }

    Simulation::getInstance().start(opt);
    return 0;
}
