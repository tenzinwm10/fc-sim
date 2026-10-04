#include <cmath>
#include <cstddef>
#include <cstdio>
#include <memory>
#include <mujoco/mujoco.h>

#include "shared/sim/ModelMap.hpp"
#include "shared/util/Config.hpp"
#include "module/Camera/src/SharedImageWriter.hpp"

static_assert(sizeof(k1sim::module::camera::SharedImageHeader) == 88,
              "t2_camera_bridge.py expects an 88-byte image header");
static_assert(offsetof(k1sim::module::camera::SharedImageHeader, focal_length) == 60,
              "t2_camera_bridge.py expects focal length at offset 60");

int main() {
    const auto path = k1sim::config::resolve_path("models/t2/t2_scene_robocup_middle.xml").string();
    char error[1024] = {};
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
        mj_loadXML(path.c_str(), nullptr, error, sizeof(error)), mj_deleteModel);
    if (!model) {
        std::fprintf(stderr, "T2 scene load failed: %s\n", error);
        return 1;
    }
    const auto map = k1sim::ModelMap::build(model.get());
    const int ready = mj_name2id(model.get(), mjOBJ_KEY, "ready");
    const int camera = mj_name2id(model.get(), mjOBJ_CAMERA, "head");
    const int ball = mj_name2id(model.get(), mjOBJ_BODY, "ball");
    if (model->nu != 31 || model->nq != 45 || ready < 0 || camera < 0 || ball < 0 ||
        map.sens_quat < 0 || map.sens_gyro < 0 || map.sens_acc < 0 || map.sens_linvel < 0) {
        std::fprintf(stderr, "T2 scene contract failed: nq=%ld nu=%ld ready=%d camera=%d ball=%d\n",
                     static_cast<long>(model->nq), static_cast<long>(model->nu), ready, camera, ball);
        return 1;
    }
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
    mj_resetDataKeyframe(model.get(), data.get(), ready);
    mj_forward(model.get(), data.get());
    if (!std::isfinite(data->qpos[map.root_qpos_adr + 2]) || data->ncon == 0) {
        std::fprintf(stderr, "T2 ready pose has invalid base height or no floor contacts\n");
        return 1;
    }
    std::printf("T2 model OK: 31 actuators, ball, camera, IMU, ready contact\n");
    return 0;
}
