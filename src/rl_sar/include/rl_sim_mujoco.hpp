/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RL_SIM_HPP
#define RL_SIM_HPP

// #define PLOT
// #define CSV_LOGGER

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "inference_runtime.hpp"
#include "loop.hpp"
#include "fsm_all.hpp"

#include <csignal>
#include <vector>
#include <string>
#include <cstdlib>
#include <unistd.h>
#include <sys/wait.h>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <memory>
#include <string>

#include <mujoco/mujoco.h>
#include "joystick.hh"
#include "mujoco_utils.hpp"

#ifdef PLOT
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
#endif

class Button
{
public:
    Button() {}

    void update(bool state)
    {
        on_press = state ? state != pressed : false;
        on_release = state ? false : state != pressed;
        pressed = state;
    }

    bool pressed = false;
    bool on_press = false;
    bool on_release = false;
};

class RL_Sim : public RL
{
public:
    RL_Sim(int argc, char **argv);
    ~RL_Sim();

    enum class FaultMode
    {
        None = 0,
        Locked,
        Weakened,
    };

    std::unique_ptr<mj::Simulate> sim;
    static RL_Sim* instance;

private:
    // rl functions
    std::vector<float> Forward() override;
    void GetState(RobotState<float> *state) override;
    void SetCommand(const RobotCommand<float> *command) override;
    void RunModel();
    void RobotControl();

    // loop
    std::shared_ptr<LoopFunc> loop_keyboard;
    std::shared_ptr<LoopFunc> loop_joystick;
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_rl;
    std::shared_ptr<LoopFunc> loop_plot;

    // plot
    const int plot_size = 100;
    std::vector<int> plot_t;
    std::vector<std::vector<float>> plot_real_joint_pos, plot_target_joint_pos;
    void Plot();

    // mujoco
    mjData *mj_data;
    mjModel *mj_model;
    std::string scene_name;

    // joystick
    std::unique_ptr<Joystick> sys_js;
    JoystickEvent sys_js_event;
    std::string sys_js_device;

    Button sys_js_button[20];
    int sys_js_axis[10] = {0};
    bool sys_js_active = false;
    float axis_deadzone = 0.05f;
    int sys_js_max_value = (1 << (16 - 1));
    bool TryOpenSysJoystick(const std::string& device);
    void SetupSysJoystick(int bits);
    void GetSysJoystick();

    // others
    std::map<std::string, float> joint_positions;
    std::map<std::string, float> joint_velocities;
    std::map<std::string, float> joint_efforts;

    FaultMode fault_mode = FaultMode::None;
    int fault_joint_idx = 0;
    float fault_tau_scale = 0.20f;
    float fault_lock_half_range = 0.05f;
    float fault_locked_q = 0.0f;

    std::string GetFaultJointName() const;
    bool TryGetConfiguredLockedJointTarget(float* target_q) const;
    void RefreshLockedJointTarget();
    void CycleFaultMode();
    void SelectFaultJoint(int delta);
    void AdjustFaultSeverity(float delta);
    void PrintFaultStatus() const;
};

#endif // RL_SIM_HPP
