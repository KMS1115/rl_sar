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
#include <array>

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
    };

    enum class FaultReleasePhase
    {
        None = 0,
        ToStand,
        ToPolicy,
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
    int fault_leg_idx = 0;
    std::array<float, 3> fault_locked_q = {0.0f, 0.0f, 0.0f};
    std::array<float, 3> fault_lock_start_q = {0.0f, 0.0f, 0.0f};
    int fault_lock_start_motiontime = 0;
    float fault_lock_ramp_duration = 2.0f;
    bool fault_lock_transition_active = false;
    int fault_input_last_motiontime = -1000000;
    float fault_input_debounce_s = 0.3f;
    int fault_release_leg_idx = -1;
    std::array<float, 3> fault_release_start_q = {0.0f, 0.0f, 0.0f};
    std::array<float, 3> fault_release_target_q = {0.0f, 0.0f, 0.0f};
    int fault_release_start_motiontime = 0;
    bool fault_release_transition_active = false;
    FaultReleasePhase fault_release_phase = FaultReleasePhase::None;
    int pending_fault_leg_idx = -1;
    int fault_switch_settle_start_motiontime = -1;
    float fault_switch_settle_duration = 0.25f;
    float fault_release_to_stand_duration = 0.5f;
    float fault_return_to_policy_duration = 0.35f;
    float fault_transition_kp = 40.0f;
    float fault_transition_kd = 1.5f;

    std::string GetFaultLegName() const;
    std::array<int, 3> GetFaultLegJointIndices() const;
    std::array<int, 3> GetLegJointIndices(int leg_idx) const;
    std::vector<int> GetFaultJointOffsets() const;
    bool TryGetConfiguredLockedJointTarget(int joint_idx, float* target_q) const;
    void BeginLockedFaultTransition(const std::array<int, 3>& joint_indices, const std::array<float, 3>& target_q);
    float GetLockedFaultDesiredQ(int leg_joint_offset) const;
    void BeginReleaseTransition(int leg_idx, const RobotCommand<float>* command);
    float GetReleasedFaultDesiredQ(int leg_joint_offset) const;
    float GetFaultReleasePhaseDuration() const;
    bool IsFaultReleaseTransitionComplete() const;
    void StartPolicyResumeTransition(const RobotCommand<float>* command);
    void UpdatePendingFaultSwitch(const RobotCommand<float>* command);
    void RefreshLockedLegTarget();
    void CycleFaultMode(const RobotCommand<float>* command);
    void SelectFaultLeg(int delta, const RobotCommand<float>* command);
    void PrintFaultStatus() const;
};

#endif // RL_SIM_HPP
