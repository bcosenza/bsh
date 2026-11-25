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
        "          [-snapshot f1 f2 ...] [-snapshot-dir dir] [-window w h]\n"
        "          [-camera ex ey ez cx cy cz] [-n N] [-neighbours K] [-goal w]\n"
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
        "                   (0 = none, the default; smaller = more spread)\n"
        "  -snapshot f1 f2 ...\n"
        "                   write sceneN_snap_FFFF.png after frames f1, f2, ... (frame 0 =\n"
        "                   initial state, frame f = after f fixed-dt steps), without the\n"
        "                   overlay, with a fixed seed (default %d), then quit\n"
        "  -snapshot-dir d  folder for the snapshots (default: current folder)\n"
        "  -window w h      window (and snapshot) size in pixels (default 640 480)\n"
        "  -camera ex ey ez cx cy cz\n"
        "                   fixed camera: eye position and look-at point (world units),\n"
        "                   replacing the scene's camera preset\n"
        "  -n N             number of agents (scaling tests): the world and the start\n"
        "                   spheres grow by cbrt(N / N_scene), keeping layout and density\n"
        "  -neighbours K    with -n: size the start spheres for ~K group mates within the\n"
        "                   cohesion radius instead, and the world to fit them\n"
        "  -goal w          scenes 4 and 5: each group seeks its goal (the opposite\n"
        "                   group's start) with weight w, so the groups keep their course\n",
        prog, SH_VMF_KAPPA, TIMING_WARMUP_STEPS, ACCURACY_DEFAULT_SEED, ACCURACY_DEFAULT_SEED);
}

// Linux entry point
int main(int argc, char* argv[]) {
    RunOptions opt;
    int winW = 0, winH = 0;
    bool fixedCam = false;
    float cam[6] = { 0, 0, 0, 0, 0, 0 };
    for (int i = 1; i < argc; i++){
        const char* a = argv[i];
        const bool hasValue = (i + 1 < argc);
        if (!std::strcmp(a, "-timing") && hasValue)        opt.timingSteps = std::atoi(argv[++i]);
        else if (!std::strcmp(a, "-accuracy") && hasValue) opt.accuracySteps = std::atoi(argv[++i]);
        else if (!std::strcmp(a, "-out") && hasValue)      opt.accuracyOut = argv[++i];
        else if (!std::strcmp(a, "-seed") && hasValue)     opt.seed = std::atoi(argv[++i]);
        else if (!std::strcmp(a, "-hspread") && hasValue)  opt.headingSpread = (float)std::atof(argv[++i]);
        else if (!std::strcmp(a, "-scene") && hasValue)    opt.model = std::atoi(argv[++i]);
        else if (!std::strcmp(a, "-n") && hasValue)        opt.numAgents = (unsigned int)std::strtoul(argv[++i], nullptr, 10);
        else if (!std::strcmp(a, "-neighbours") && hasValue) opt.neighbours = (float)std::atof(argv[++i]);
        else if (!std::strcmp(a, "-goal") && hasValue)     opt.goalWeight = (float)std::atof(argv[++i]);
        else if (!std::strcmp(a, "-kappa") && hasValue)    opt.vmfK = (float)std::atof(argv[++i]);
        else if (!std::strcmp(a, "-snapshot-dir") && hasValue) opt.snapshotDir = argv[++i];
        else if (!std::strcmp(a, "-snapshot")){
            // frame numbers up to the next option
            while (i + 1 < argc && argv[i + 1][0] != '-')
                opt.snapshotFrames.push_back(std::atoi(argv[++i]));
            if (opt.snapshotFrames.empty()){ usage(argv[0]); return 1; }
        }
        else if (!std::strcmp(a, "-window") && i + 2 < argc){
            winW = std::atoi(argv[++i]);
            winH = std::atoi(argv[++i]);
            if (winW <= 0 || winH <= 0){ usage(argv[0]); return 1; }
        }
        else if (!std::strcmp(a, "-camera") && i + 6 < argc){
            for (int c = 0; c < 6; c++) cam[c] = (float)std::atof(argv[++i]);
            fixedCam = true;
        }
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
    if (opt.vmfK <= 0.0f || opt.headingSpread < 0.0f || opt.neighbours < 0.0f || opt.goalWeight < 0.0f){ usage(argv[0]); return 1; }
    const int modes = (opt.timingSteps > 0) + (opt.accuracySteps > 0) + !opt.snapshotFrames.empty();
    if (modes > 1){
        std::fprintf(stderr, "-timing, -accuracy and -snapshot cannot be combined\n");
        return 1;
    }

    // the window is created with the simulation, so size it first
    GFX::getInstance().setInitialWindowSize(winW, winH);
    if (fixedCam)
        GFX::getInstance().setFixedCamera(glm::vec3(cam[0], cam[1], cam[2]), glm::vec3(cam[3], cam[4], cam[5]));

    Simulation::getInstance().start(opt);
    return 0;
}
