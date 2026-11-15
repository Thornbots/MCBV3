// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <vector>

#include "subsystems/jetson/AimPlanner.hpp"

namespace {
using Clock = std::chrono::steady_clock;
void report(const char* name, std::vector<double> values) {
    std::sort(values.begin(), values.end());
    double sum = 0;
    for (double v : values) sum += v;
    std::printf("%s mean_us=%.3f p99_us=%.3f max_us=%.3f n=%zu\n", name, sum / values.size(), values[values.size() * 99 / 100], values.back(), values.size());
}
}  // namespace
int main() {
    aim::AimPlanner planner;
    std::vector<double> plans, gates;
    unsigned valid = 0, inside = 0;
    for (int i = 0; i < 10000; ++i) {
        aim::TargetInput in;
        in.valid = true;
        in.track_id = 1;
        in.center = {3, 0, .3f};
        in.yaw = aim::PI_F + i * .001f;
        in.yaw_rate = 9;
        in.velocity = {.3f, 2, 0};
        in.acceleration = {.2f, -.4f, 0};
        in.radius = {.25f, .2f};
        in.z_offset = {.05f, -.05f};
        const auto start = Clock::now();
        const auto plan = planner.plan(i, in, {0, 0, .4035f}, {.2f, -1, 0});
        const auto solved = Clock::now();
        const auto impact = aim::predict_impact(in, plan, {0, 0, .4035f}, {.2f, -1, 0}, {plan.yaw, plan.pitch, 0, 0}, {});
        const auto done = Clock::now();
        valid += plan.valid;
        inside += impact && impact->inside;
        plans.push_back(std::chrono::duration<double, std::micro>(solved - start).count());
        gates.push_back(std::chrono::duration<double, std::micro>(done - solved).count());
    }
    report("replan", plans);
    report("gate", gates);
    std::printf("valid=%u inside=%u\n", valid, inside);
    return valid > 0 && inside == valid ? 0 : 1;
}
