#include <array>
#include <cmath>
#include <cstdio>
#include <string>

#include "module/Simulation/src/SimCore.hpp"

int main() {
    k1sim::SimCore::Config cfg;
    cfg.model_path = "models/t2/t2_scene_robocup_middle.xml";
    cfg.robots = 6;
    cfg.spawns = {{-2.0, 0.0, 0.0}, {-6.5, 0.0, 0.0}, {-3.0, 2.5, 0.0},
                   {2.0, 0.0, 3.141592653589793}, {6.5, 0.0, 3.141592653589793},
                   {3.0, -2.5, 3.141592653589793}};
    k1sim::SimCore sim(cfg);
    sim.load_model();
    for (int i = 0; i < 6; ++i) {
        char name[32];
        if (i == 0) {
            std::snprintf(name, sizeof(name), "trunk");
        }
        else {
            std::snprintf(name, sizeof(name), "sub%02d_trunk", i);
        }
        const int body = mj_name2id(sim.model(), mjOBJ_BODY, name);
        if (body < 0) {
            std::fprintf(stderr, "missing robot body %s\n", name);
            return 1;
        }
        const auto* p = sim.data()->xpos + 3 * body;
        if (std::abs(p[0] - cfg.spawns[i][0]) > 0.01 ||
            std::abs(p[1] - cfg.spawns[i][1]) > 0.01) {
            std::fprintf(stderr, "%s at %.3f, %.3f; expected %.3f, %.3f\n",
                         name, p[0], p[1], cfg.spawns[i][0], cfg.spawns[i][1]);
            return 1;
        }
    }
    std::printf("T2 six-robot spawn geometry OK\n");
    return 0;
}
