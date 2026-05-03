/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_sim_mujoco.hpp"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sstream>
#include <vector>

namespace
{
std::string ToLower(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return text;
}

bool ContainsAny(const std::string& text, const std::vector<std::string>& tokens)
{
    return std::any_of(tokens.begin(), tokens.end(), [&](const std::string& token) {
        return text.find(token) != std::string::npos;
    });
}

bool IsThighJointName(const std::string& joint_name)
{
    const std::string lowered = ToLower(joint_name);
    return lowered.find("thigh") != std::string::npos;
}

bool IsHipJointName(const std::string& joint_name)
{
    const std::string lowered = ToLower(joint_name);
    return lowered.find("hip") != std::string::npos;
}

bool IsCalfJointName(const std::string& joint_name)
{
    const std::string lowered = ToLower(joint_name);
    return lowered.find("calf") != std::string::npos || lowered.find("knee") != std::string::npos;
}

bool IsPointerLikeDevice(const std::string& device_name)
{
    const std::string lowered = ToLower(device_name);
    static const std::vector<std::string> kBlockedTokens = {
        "mouse",
        "touchpad",
        "trackpad",
        "keyboard",
        "consumer control",
        "sleep button",
        "power button"
    };
    return ContainsAny(lowered, kBlockedTokens);
}

bool IsLikelyGamepad(const std::string& device_name, unsigned char axis_count, unsigned char button_count)
{
    const std::string lowered = ToLower(device_name);
    static const std::vector<std::string> kPreferredTokens = {
        "controller",
        "gamepad",
        "joystick",
        "xbox",
        "dualshock",
        "dualsense",
        "8bitdo",
        "switch pro",
        "steam controller"
    };

    if (ContainsAny(lowered, kPreferredTokens))
    {
        return true;
    }

    return axis_count >= 4 && button_count >= 8;
}

bool IsKnownConfigName(const std::string& robot_name, const std::string& name)
{
    const std::string lowered = ToLower(name);
    if (lowered.empty())
    {
        return false;
    }

    const std::filesystem::path config_path =
        std::filesystem::path(POLICY_DIR) / ToLower(robot_name) / lowered / "config.yaml";
    return std::filesystem::exists(config_path);
}

const char* FaultModeName(RL_Sim::FaultMode mode)
{
    switch (mode)
    {
    case RL_Sim::FaultMode::Locked:
        return "locked";
    default:
        return "none";
    }
}

bool IsFaultOverrideSuppressed(const RL_Sim& rl)
{
    return rl.fsm.current_state_ &&
           rl.fsm.current_state_->GetStateName() == "RLFSMStatePassive";
}
}

RL_Sim* RL_Sim::instance = nullptr;

RL_Sim::RL_Sim(int argc, char **argv)
{
    // Set static instance pointer early for signal handler
    instance = this;

    if (argc < 2)
    {
        std::cout << LOGGER::ERROR << "Usage: " << argv[0] << " <go2|go2w> [config_name]" << std::endl;
        throw std::runtime_error("Invalid arguments");
    }

    this->robot_name = argv[1];
    this->scene_name = "scene";
    this->config_name = "default";

    if (argc >= 3)
    {
        const std::string arg2 = argv[2];
        if (arg2 == "scene")
        {
            this->config_name = (argc > 3) ? argv[3] : "default";
        }
        else if (IsKnownConfigName(this->robot_name, arg2))
        {
            this->config_name = arg2;
        }
        else
        {
            this->scene_name = arg2;
            this->config_name = (argc > 3) ? argv[3] : "default";
        }
    }

    this->ang_vel_axis = "body";

    // now launch mujoco
    std::cout << LOGGER::INFO << "[MuJoCo] Launching..." << std::endl;

    // display an error if running on macOS under Rosetta 2
#if defined(__APPLE__) && defined(__AVX__)
    if (rosetta_error_msg)
    {
        DisplayErrorDialogBox("Rosetta 2 is not supported", rosetta_error_msg);
        std::exit(1);
    }
#endif

    // print version, check compatibility
    std::cout << LOGGER::INFO << "[MuJoCo] Version: " << mj_versionString() << std::endl;
    if (mjVERSION_HEADER != mj_version())
    {
        mju_error("Headers and library have different versions");
    }

    // scan for libraries in the plugin directory to load additional plugins
    scanPluginLibraries();

    mjvCamera cam;
    mjv_defaultCamera(&cam);

    mjvOption opt;
    mjv_defaultOption(&opt);

    mjvPerturb pert;
    mjv_defaultPerturb(&pert);

    // simulate object encapsulates the UI
    sim = std::make_unique<mj::Simulate>(
        std::make_unique<mj::GlfwAdapter>(),
        &cam, &opt, &pert, /* is_passive = */ false);

    std::string filename = std::string(CMAKE_CURRENT_SOURCE_DIR) + "/../rl_sar_zoo/" + this->robot_name + "_description/mjcf/" + this->scene_name + ".xml";

    // start physics thread
    std::thread physicsthreadhandle(&PhysicsThread, sim.get(), filename.c_str());
    physicsthreadhandle.detach();

    while (1)
    {
        if (d)
        {
            std::cout << LOGGER::INFO << "[MuJoCo] Data prepared" << std::endl;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    this->mj_model = m;
    this->mj_data = d;
    this->SetupSysJoystick(16);

    // read params from yaml
    this->ReadYaml(this->robot_name, "base.yaml");
    this->InitUdpCommandReceiver();

    // auto load FSM by robot_name
    if (FSMManager::GetInstance().IsTypeSupported(this->robot_name))
    {
        auto fsm_ptr = FSMManager::GetInstance().CreateFSM(this->robot_name, this);
        if (fsm_ptr)
        {
            this->fsm = *fsm_ptr;
        }
    }
    else
    {
        std::cout << LOGGER::ERROR << "[FSM] No FSM registered for robot: " << this->robot_name << std::endl;
    }

    // init robot
    this->InitJointNum(this->params.Get<int>("num_of_dofs"));
    this->InitOutputs();
    this->InitControl();

    // loop
    this->loop_control = std::make_shared<LoopFunc>("loop_control", this->params.Get<float>("dt"), std::bind(&RL_Sim::RobotControl, this));
    this->loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.Get<float>("dt") * this->params.Get<int>("decimation"), std::bind(&RL_Sim::RunModel, this));
    this->loop_control->start();
    this->loop_rl->start();

    // keyboard
    this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&RL_Sim::KeyboardInterface, this));
    this->loop_keyboard->start();

    // joystick
    this->loop_joystick = std::make_shared<LoopFunc>("loop_joystick", 0.01, std::bind(&RL_Sim::GetSysJoystick, this));
    this->loop_joystick->start();

#ifdef PLOT
    this->plot_t = std::vector<int>(this->plot_size, 0);
    this->plot_real_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    this->plot_target_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    for (auto &vector : this->plot_real_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    for (auto &vector : this->plot_target_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    this->loop_plot = std::make_shared<LoopFunc>("loop_plot", 0.001, std::bind(&RL_Sim::Plot, this));
    this->loop_plot->start();
#endif
#ifdef CSV_LOGGER
    this->CSVInit(this->robot_name + "/" + this->config_name);
#endif

    std::cout << LOGGER::INFO << "RL_Sim start" << std::endl;
    this->PrintFaultStatus();

    // start simulation UI loop (blocking call)
    sim->RenderLoop();
}

RL_Sim::~RL_Sim()
{
    // Clear static instance pointer
    instance = nullptr;

    this->loop_keyboard->shutdown();
    this->loop_joystick->shutdown();
    this->loop_control->shutdown();
    this->loop_rl->shutdown();
    this->CloseUdpCommandReceiver();
#ifdef PLOT
    this->loop_plot->shutdown();
#endif
    std::cout << LOGGER::INFO << "RL_Sim exit" << std::endl;
}

void RL_Sim::GetState(RobotState<float> *state)
{
    if (mj_data)
    {
        state->imu.quaternion[0] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 0];
        state->imu.quaternion[1] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 1];
        state->imu.quaternion[2] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 2];
        state->imu.quaternion[3] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 3];

        state->imu.gyroscope[0] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 4];
        state->imu.gyroscope[1] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 5];
        state->imu.gyroscope[2] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 6];

        for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
        {
            state->motor_state.q[i] = mj_data->sensordata[this->params.Get<std::vector<int>>("joint_mapping")[i]];
            state->motor_state.dq[i] = mj_data->sensordata[this->params.Get<std::vector<int>>("joint_mapping")[i] + this->params.Get<int>("num_of_dofs")];
            state->motor_state.tau_est[i] = mj_data->sensordata[this->params.Get<std::vector<int>>("joint_mapping")[i] + 2 * this->params.Get<int>("num_of_dofs")];
        }
    }
}

void RL_Sim::SetCommand(const RobotCommand<float> *command)
{
    if (mj_data)
    {
        const bool suppress_fault_override = IsFaultOverrideSuppressed(*this);
        if (!suppress_fault_override)
        {
            this->UpdatePendingFaultSwitch(command);
        }

        const auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
        const auto joint_names = this->params.Get<std::vector<std::string>>("joint_names");
        const int num_of_dofs = this->params.Get<int>("num_of_dofs");

        for (int i = 0; i < num_of_dofs; ++i)
        {
            float joint_q = mj_data->sensordata[joint_mapping[i]];
            float joint_dq = mj_data->sensordata[joint_mapping[i] + num_of_dofs];
            float desired_q = command->motor_command.q[i];
            float desired_dq = command->motor_command.dq[i];
            float desired_tau = command->motor_command.tau[i];
            float desired_kp = command->motor_command.kp[i];
            float desired_kd = command->motor_command.kd[i];

            if (!suppress_fault_override && this->fault_mode == FaultMode::Locked)
            {
                const auto fault_joint_indices = this->GetFaultLegJointIndices();
                const auto fault_joint_offsets = this->GetFaultJointOffsets();
                for (int leg_joint_offset : fault_joint_offsets)
                {
                    if (i == fault_joint_indices[leg_joint_offset])
                    {
                        desired_q = this->GetLockedFaultDesiredQ(leg_joint_offset);
                        desired_dq = 0.0f;
                        desired_tau = 0.0f;
                        desired_kp = this->fault_transition_kp;
                        desired_kd = this->fault_transition_kd;
                        break;
                    }
                }
            }
            if (!suppress_fault_override && this->fault_release_transition_active)
            {
                const auto release_joint_indices = this->GetLegJointIndices(this->fault_release_leg_idx);
                const auto fault_joint_offsets = this->GetFaultJointOffsets();
                for (int leg_joint_offset : fault_joint_offsets)
                {
                    if (i == release_joint_indices[leg_joint_offset])
                    {
                        desired_q = this->GetReleasedFaultDesiredQ(leg_joint_offset);
                        desired_dq = 0.0f;
                        desired_tau = 0.0f;
                        desired_kp = this->fault_transition_kp;
                        desired_kd = this->fault_transition_kd;
                        break;
                    }
                }
            }

            float ctrl =
                desired_tau +
                desired_kp * (desired_q - joint_q) +
                desired_kd * (desired_dq - joint_dq);

            mj_data->ctrl[joint_mapping[i]] = ctrl;
        }
    }
}

bool RL_Sim::IsGo2WPolicyToggleActive() const
{
    const std::string robot = ToLower(this->robot_name);
    const std::string config = ToLower(this->config_name);
    return robot == "go2w" && (config == "footstand" || config == "default");
}

void RL_Sim::ToggleGo2WPolicy(const std::string& source)
{
    if (!this->IsGo2WPolicyToggleActive())
    {
        return;
    }

    const std::string current_config = ToLower(this->config_name);
    const std::string target_config = current_config == "footstand" ? "default" : "footstand";
    this->LoadGo2WPolicy(target_config, source);
}

void RL_Sim::LoadGo2WPolicy(const std::string& target_config, const std::string& source)
{
    const std::string previous_config = this->config_name;
    const YAML::Node previous_config_node = this->params.config_node;

    try
    {
        this->rl_init_done = false;
        this->config_name = target_config;
        this->params.config_node = YAML::Node(YAML::NodeType::Map);
        this->ReadYaml(this->robot_name, "base.yaml");
        this->InitRL(this->robot_name + "/" + this->config_name);

        this->now_state = this->robot_state;
        this->start_state = this->robot_state;
        this->episode_length_buf = 0;

        this->fault_mode = FaultMode::None;
        this->fault_lock_transition_active = false;
        this->fault_release_transition_active = false;
        this->fault_release_phase = FaultReleasePhase::None;
        this->pending_fault_leg_idx = -1;
        this->fault_switch_settle_start_motiontime = -1;

        std::cout << std::endl << LOGGER::INFO << "[Policy Switch] " << source
                  << " -> go2w/" << this->config_name << std::endl;
    }
    catch (const std::exception& e)
    {
        this->config_name = previous_config;
        this->params.config_node = previous_config_node;
        this->rl_init_done = false;
        std::cout << std::endl << LOGGER::ERROR << "[Policy Switch] failed to load go2w/"
                  << target_config << ": "
                  << e.what() << std::endl;
    }
}

void RL_Sim::RobotControl()
{
    // Lock the sim mutex once for the entire control cycle to prevent race conditions
    const std::lock_guard<std::recursive_mutex> lock(sim->mtx);

    this->GetState(&this->robot_state);

    if (this->control.current_keyboard == Input::Keyboard::C)
    {
        this->ToggleGo2WPolicy("keyboard c");
    }
    if (this->control.current_gamepad == Input::Gamepad::Y)
    {
        this->ToggleGo2WPolicy("gamepad Y");
    }

    this->StateController(&this->robot_state, &this->robot_command);

    const float fault_input_elapsed =
        static_cast<float>(this->motiontime - this->fault_input_last_motiontime) * this->params.Get<float>("dt");
    const bool fault_input_ready = fault_input_elapsed >= this->fault_input_debounce_s;

    if (this->control.current_keyboard == Input::Keyboard::R || this->control.current_gamepad == Input::Gamepad::RB_Y)
    {
        if (this->mj_model && this->mj_data)
        {
            mj_resetData(this->mj_model, this->mj_data);
            mj_forward(this->mj_model, this->mj_data);
        }
    }
    if (this->control.current_keyboard == Input::Keyboard::Enter || this->control.current_gamepad == Input::Gamepad::RB_X)
    {
        if (simulation_running)
        {
            sim->run = 0;
            std::cout << std::endl << LOGGER::INFO << "Simulation Stop" << std::endl;
        }
        else
        {
            sim->run = 1;
            std::cout << std::endl << LOGGER::INFO << "Simulation Start" << std::endl;
        }
        simulation_running = !simulation_running;
    }
    if (fault_input_ready &&
        (this->control.current_keyboard == Input::Keyboard::T || this->control.current_gamepad == Input::Gamepad::LB_A))
    {
        this->CycleFaultMode(&this->robot_command);
        this->fault_input_last_motiontime = this->motiontime;
    }
    if (fault_input_ready &&
        (this->control.current_keyboard == Input::Keyboard::Y || this->control.current_gamepad == Input::Gamepad::LB_DPadLeft))
    {
        this->SelectFaultLeg(-1, &this->robot_command);
        this->fault_input_last_motiontime = this->motiontime;
    }
    if (fault_input_ready &&
        (this->control.current_keyboard == Input::Keyboard::U || this->control.current_gamepad == Input::Gamepad::LB_DPadRight))
    {
        this->SelectFaultLeg(1, &this->robot_command);
        this->fault_input_last_motiontime = this->motiontime;
    }
    this->control.ClearInput();

    this->SetCommand(&this->robot_command);
}


std::string RL_Sim::GetFaultLegName() const
{
    static const std::array<std::string, 4> kLegNames = {"FR", "FL", "RR", "RL"};
    if (this->fault_leg_idx >= 0 && this->fault_leg_idx < static_cast<int>(kLegNames.size()))
    {
        return kLegNames[this->fault_leg_idx];
    }
    return "leg_" + std::to_string(this->fault_leg_idx);
}

std::array<int, 3> RL_Sim::GetFaultLegJointIndices() const
{
    return this->GetLegJointIndices(this->fault_leg_idx);
}

std::array<int, 3> RL_Sim::GetLegJointIndices(int leg_idx) const
{
    const int leg_start = leg_idx * 3;
    return {leg_start + 0, leg_start + 1, leg_start + 2};
}

std::vector<int> RL_Sim::GetFaultJointOffsets() const
{
    const auto configured_offsets = this->params.Get<std::vector<int>>("fault_lock_joint_offsets", {});
    std::vector<int> valid_offsets;
    for (int offset : configured_offsets)
    {
        if (offset >= 0 && offset < 3 && std::find(valid_offsets.begin(), valid_offsets.end(), offset) == valid_offsets.end())
        {
            valid_offsets.push_back(offset);
        }
    }

    if (valid_offsets.empty())
    {
        return {0, 1, 2};
    }
    return valid_offsets;
}

bool RL_Sim::TryGetConfiguredLockedJointTarget(int joint_idx, float* target_q) const
{
    if (target_q == nullptr || joint_idx < 0 || joint_idx >= this->params.Get<int>("num_of_dofs"))
    {
        return false;
    }

    const auto joint_names = this->params.Get<std::vector<std::string>>("joint_names");
    if (joint_idx >= static_cast<int>(joint_names.size()))
    {
        return false;
    }

    const std::string joint_name = joint_names[joint_idx];
    const auto default_dof_pos = this->params.Get<std::vector<float>>("default_dof_pos");
    if (IsHipJointName(joint_name))
    {
        if (this->params.Has("fault_lock_hip_q"))
        {
            *target_q = this->params.Get<float>("fault_lock_hip_q");
            return true;
        }
        *target_q = default_dof_pos[joint_idx];
        return true;
    }

    if (IsThighJointName(joint_name))
    {
        if (this->params.Has("fault_lock_thigh_q"))
        {
            *target_q = this->params.Get<float>("fault_lock_thigh_q");
            return true;
        }
        *target_q = default_dof_pos[joint_idx];
        return true;
    }

    if (IsCalfJointName(joint_name))
    {
        if (this->params.Has("fault_lock_calf_q"))
        {
            *target_q = this->params.Get<float>("fault_lock_calf_q");
            return true;
        }

        *target_q = default_dof_pos[joint_idx];
        return true;
    }

    return false;
}

void RL_Sim::BeginLockedFaultTransition(const std::array<int, 3>& joint_indices, const std::array<float, 3>& target_q)
{
    for (int i = 0; i < 3; ++i)
    {
        this->fault_lock_start_q[i] = this->robot_state.motor_state.q[joint_indices[i]];
        this->fault_locked_q[i] = target_q[i];
    }
    this->fault_lock_start_motiontime = this->motiontime;
    this->fault_lock_transition_active = true;
    if (this->params.Has("fault_lock_ramp_duration"))
    {
        this->fault_lock_ramp_duration = std::max(this->params.Get<float>("fault_lock_ramp_duration"), 0.0f);
    }
    if (this->params.Has("fault_transition_kp"))
    {
        this->fault_transition_kp = std::max(this->params.Get<float>("fault_transition_kp"), 0.0f);
    }
    if (this->params.Has("fault_transition_kd"))
    {
        this->fault_transition_kd = std::max(this->params.Get<float>("fault_transition_kd"), 0.0f);
    }
}

float RL_Sim::GetLockedFaultDesiredQ(int leg_joint_offset) const
{
    if (leg_joint_offset < 0 || leg_joint_offset >= 3)
    {
        return 0.0f;
    }

    if (!this->fault_lock_transition_active || this->fault_lock_ramp_duration <= 0.0f)
    {
        return this->fault_locked_q[leg_joint_offset];
    }

    const float elapsed = static_cast<float>(this->motiontime - this->fault_lock_start_motiontime) * this->params.Get<float>("dt");
    const float alpha = std::clamp(elapsed / this->fault_lock_ramp_duration, 0.0f, 1.0f);
    return this->fault_lock_start_q[leg_joint_offset]
        + alpha * (this->fault_locked_q[leg_joint_offset] - this->fault_lock_start_q[leg_joint_offset]);
}

void RL_Sim::BeginReleaseTransition(int leg_idx, const RobotCommand<float>* command)
{
    this->fault_release_leg_idx = leg_idx;
    const auto joint_indices = this->GetLegJointIndices(leg_idx);
    const auto stand_dof_pos = this->params.Get<std::vector<float>>("default_dof_pos");
    for (int i = 0; i < 3; ++i)
    {
        this->fault_release_start_q[i] = this->robot_state.motor_state.q[joint_indices[i]];
        this->fault_release_target_q[i] = stand_dof_pos[joint_indices[i]];
    }
    this->fault_release_start_motiontime = this->motiontime;
    this->fault_release_transition_active = true;
    this->fault_release_phase = FaultReleasePhase::ToStand;
    if (this->params.Has("fault_lock_ramp_duration"))
    {
        this->fault_lock_ramp_duration = std::max(this->params.Get<float>("fault_lock_ramp_duration"), 0.0f);
    }
    if (this->params.Has("fault_release_to_stand_duration"))
    {
        this->fault_release_to_stand_duration =
            std::max(this->params.Get<float>("fault_release_to_stand_duration"), 0.0f);
    }
    if (this->params.Has("fault_return_to_policy_duration"))
    {
        this->fault_return_to_policy_duration =
            std::max(this->params.Get<float>("fault_return_to_policy_duration"), 0.0f);
    }
    if (this->params.Has("fault_transition_kp"))
    {
        this->fault_transition_kp = std::max(this->params.Get<float>("fault_transition_kp"), 0.0f);
    }
    if (this->params.Has("fault_transition_kd"))
    {
        this->fault_transition_kd = std::max(this->params.Get<float>("fault_transition_kd"), 0.0f);
    }
}

float RL_Sim::GetReleasedFaultDesiredQ(int leg_joint_offset) const
{
    const float duration = this->GetFaultReleasePhaseDuration();
    if (!this->fault_release_transition_active || duration <= 0.0f)
    {
        return this->fault_release_target_q[leg_joint_offset];
    }
    const float elapsed = static_cast<float>(this->motiontime - this->fault_release_start_motiontime) * this->params.Get<float>("dt");
    const float alpha = std::clamp(elapsed / duration, 0.0f, 1.0f);
    return this->fault_release_start_q[leg_joint_offset]
        + alpha * (this->fault_release_target_q[leg_joint_offset] - this->fault_release_start_q[leg_joint_offset]);
}

float RL_Sim::GetFaultReleasePhaseDuration() const
{
    if (this->fault_release_phase == FaultReleasePhase::ToStand)
    {
        return this->fault_release_to_stand_duration;
    }
    if (this->fault_release_phase == FaultReleasePhase::ToPolicy)
    {
        return this->fault_return_to_policy_duration;
    }
    return 0.0f;
}

bool RL_Sim::IsFaultReleaseTransitionComplete() const
{
    const float duration = this->GetFaultReleasePhaseDuration();
    if (!this->fault_release_transition_active || duration <= 0.0f)
    {
        return true;
    }

    const float elapsed = static_cast<float>(this->motiontime - this->fault_release_start_motiontime) * this->params.Get<float>("dt");
    return elapsed >= duration;
}

void RL_Sim::StartPolicyResumeTransition(const RobotCommand<float>* command)
{
    if (this->fault_release_leg_idx < 0)
    {
        this->fault_release_transition_active = false;
        this->fault_release_phase = FaultReleasePhase::None;
        return;
    }

    const auto joint_indices = this->GetLegJointIndices(this->fault_release_leg_idx);
    const auto default_dof_pos = this->params.Get<std::vector<float>>("default_dof_pos");
    for (int i = 0; i < 3; ++i)
    {
        this->fault_release_start_q[i] = this->fault_release_target_q[i];
        if (command != nullptr && joint_indices[i] < static_cast<int>(command->motor_command.q.size()))
        {
            this->fault_release_target_q[i] = command->motor_command.q[joint_indices[i]];
        }
        else
        {
            this->fault_release_target_q[i] = default_dof_pos[joint_indices[i]];
        }
    }
    this->fault_release_start_motiontime = this->motiontime;
    this->fault_release_phase = FaultReleasePhase::ToPolicy;
}

void RL_Sim::UpdatePendingFaultSwitch(const RobotCommand<float>* command)
{
    (void)command;
    if (this->params.Has("fault_switch_settle_duration"))
    {
        this->fault_switch_settle_duration = std::max(this->params.Get<float>("fault_switch_settle_duration"), 0.0f);
    }

    if (this->pending_fault_leg_idx < 0)
    {
        return;
    }

    if (this->fault_switch_settle_start_motiontime < 0)
    {
        this->fault_switch_settle_start_motiontime = this->motiontime;
        return;
    }

    const float settle_elapsed =
        static_cast<float>(this->motiontime - this->fault_switch_settle_start_motiontime) * this->params.Get<float>("dt");
    if (settle_elapsed < this->fault_switch_settle_duration)
    {
        return;
    }

    this->fault_leg_idx = this->pending_fault_leg_idx;
    this->pending_fault_leg_idx = -1;
    this->fault_switch_settle_start_motiontime = -1;
    this->fault_mode = FaultMode::Locked;
    this->RefreshLockedLegTarget();
}

void RL_Sim::RefreshLockedLegTarget()
{
    if (!this->mj_data)
    {
        return;
    }

    const auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
    const int num_of_dofs = this->params.Get<int>("num_of_dofs");
    const auto joint_names = this->params.Get<std::vector<std::string>>("joint_names");
    const auto fault_joint_indices = this->GetFaultLegJointIndices();
    std::array<float, 3> configured_target_q = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 3; ++i)
    {
        const int joint_idx = fault_joint_indices[i];
        if (joint_idx < 0 || joint_idx >= num_of_dofs)
        {
            return;
        }
        if (!this->TryGetConfiguredLockedJointTarget(joint_idx, &configured_target_q[i]))
        {
            configured_target_q[i] = this->mj_data->sensordata[joint_mapping[joint_idx]];
        }
        if (this->mj_model && joint_idx < static_cast<int>(joint_names.size()))
        {
            const int joint_id = mj_name2id(this->mj_model, mjOBJ_JOINT, joint_names[joint_idx].c_str());
            if (joint_id >= 0 && this->mj_model->jnt_limited[joint_id])
            {
                const double* range = this->mj_model->jnt_range + 2 * joint_id;
                configured_target_q[i] = std::clamp(configured_target_q[i], static_cast<float>(range[0]), static_cast<float>(range[1]));
            }
        }
    }
    this->BeginLockedFaultTransition(fault_joint_indices, configured_target_q);
}

void RL_Sim::CycleFaultMode(const RobotCommand<float>* command)
{
    (void)command;
    if (this->fault_mode == FaultMode::Locked || this->pending_fault_leg_idx >= 0)
    {
        this->fault_mode = FaultMode::None;
        this->fault_lock_transition_active = false;
        this->fault_release_transition_active = false;
        this->fault_release_phase = FaultReleasePhase::None;
        this->fault_release_leg_idx = -1;
        this->pending_fault_leg_idx = -1;
        this->fault_switch_settle_start_motiontime = -1;
    }
    else
    {
        this->fault_mode = FaultMode::Locked;
        this->RefreshLockedLegTarget();
    }
    this->PrintFaultStatus();
}

void RL_Sim::SelectFaultLeg(int delta, const RobotCommand<float>* command)
{
    (void)command;
    const int selected_leg_idx = (this->pending_fault_leg_idx >= 0) ? this->pending_fault_leg_idx : this->fault_leg_idx;
    const int next_leg_idx = (selected_leg_idx + delta + 4) % 4;
    if (this->fault_mode == FaultMode::Locked)
    {
        if (next_leg_idx != this->fault_leg_idx)
        {
            this->pending_fault_leg_idx = next_leg_idx;
            this->fault_mode = FaultMode::None;
            this->fault_lock_transition_active = false;
            this->fault_release_transition_active = false;
            this->fault_release_phase = FaultReleasePhase::None;
            this->fault_release_leg_idx = -1;
            this->fault_switch_settle_start_motiontime = this->motiontime;
        }
    }
    else if (this->pending_fault_leg_idx >= 0)
    {
        this->pending_fault_leg_idx = next_leg_idx;
    }
    else
    {
        this->fault_leg_idx = next_leg_idx;
    }
    this->PrintFaultStatus();
}

void RL_Sim::PrintFaultStatus() const
{
    const float release_elapsed = this->fault_release_transition_active
        ? static_cast<float>(this->motiontime - this->fault_release_start_motiontime) * this->params.Get<float>("dt")
        : 0.0f;
    const bool release_active = this->fault_release_transition_active && release_elapsed < this->fault_lock_ramp_duration;
    std::ostringstream message;
    message << LOGGER::INFO << "[DreamFLEX Fault] mode=" << FaultModeName(this->fault_mode)
            << ", leg=" << this->fault_leg_idx << " (" << this->GetFaultLegName() << ")";
    if (this->fault_mode == FaultMode::Locked)
    {
        const auto fault_joint_indices = this->GetFaultLegJointIndices();
        const auto fault_joint_offsets = this->GetFaultJointOffsets();
        std::array<float, 3> configured_target_q = {0.0f, 0.0f, 0.0f};
        for (int i = 0; i < 3; ++i)
        {
            this->TryGetConfiguredLockedJointTarget(fault_joint_indices[i], &configured_target_q[i]);
        }
        message << ", joint_offsets=[";
        for (size_t i = 0; i < fault_joint_offsets.size(); ++i)
        {
            if (i > 0)
            {
                message << ", ";
            }
            message << fault_joint_offsets[i];
        }
        message << "]";
        message << ", q_target=[" << std::fixed << std::setprecision(3)
                << configured_target_q[0] << ", " << configured_target_q[1] << ", " << configured_target_q[2] << "]"
                << ", q_ref=[" << std::fixed << std::setprecision(3)
                << this->GetLockedFaultDesiredQ(0) << ", " << this->GetLockedFaultDesiredQ(1) << ", " << this->GetLockedFaultDesiredQ(2) << "]"
                << ", ramp_s=" << std::fixed << std::setprecision(2) << this->fault_lock_ramp_duration;
    }
    if (release_active)
    {
        message << ", release_leg=" << this->fault_release_leg_idx
                << ", release_s=" << std::fixed << std::setprecision(2) << release_elapsed;
    }
    if (this->pending_fault_leg_idx >= 0)
    {
        message << ", pending_leg=" << this->pending_fault_leg_idx;
    }
    message << ". Keys: T=cycle fault, Y/U=leg -, +";
    std::cout << message.str() << std::endl;
}

bool RL_Sim::TryOpenSysJoystick(const std::string& device)
{
    auto joystick = std::make_unique<Joystick>(device);
    if (!joystick->isFound())
    {
        return false;
    }

    const char* forced_device = std::getenv("RL_SIM_JOYSTICK_DEVICE");
    if (forced_device && device != forced_device)
    {
        return false;
    }

    const char* preferred_name = std::getenv("RL_SIM_JOYSTICK_NAME");
    const std::string device_name = joystick->name().empty() ? "unknown" : joystick->name();
    if (preferred_name)
    {
        const std::string preferred_name_lower = ToLower(preferred_name);
        if (ToLower(device_name).find(preferred_name_lower) == std::string::npos)
        {
            return false;
        }
    }

    if (IsPointerLikeDevice(device_name))
    {
        std::cout << LOGGER::INFO << "Skipping non-gamepad device: " << device
                  << " [" << device_name << "]" << std::endl;
        return false;
    }

    this->sys_js = std::move(joystick);
    this->sys_js_device = device;
    this->sys_js_active = false;
    std::fill(std::begin(this->sys_js_axis), std::end(this->sys_js_axis), 0);
    std::cout << LOGGER::INFO << "Joystick connected: " << this->sys_js_device
              << " [" << device_name << ", axes=" << int(this->sys_js->axisCount())
              << ", buttons=" << int(this->sys_js->buttonCount()) << "]" << std::endl;
    return true;
}

void RL_Sim::SetupSysJoystick(int bits)
{
    this->sys_js_max_value = (1 << (bits - 1));

    const char* forced_device = std::getenv("RL_SIM_JOYSTICK_DEVICE");
    if (forced_device)
    {
        if (this->TryOpenSysJoystick(forced_device))
        {
            return;
        }

        std::cout << LOGGER::WARNING << "Requested joystick device not usable: "
                  << forced_device << std::endl;
    }

    std::vector<std::string> fallback_devices;
    for (int index = 0; index <= 9; ++index)
    {
        const std::string device = "/dev/input/js" + std::to_string(index);
        auto joystick = std::make_unique<Joystick>(device);
        if (!joystick->isFound())
        {
            continue;
        }

        const std::string device_name = joystick->name().empty() ? "unknown" : joystick->name();
        if (IsPointerLikeDevice(device_name))
        {
            std::cout << LOGGER::INFO << "Skipping non-gamepad device: " << device
                      << " [" << device_name << "]" << std::endl;
            continue;
        }

        const char* preferred_name = std::getenv("RL_SIM_JOYSTICK_NAME");
        if (preferred_name)
        {
            const std::string preferred_name_lower = ToLower(preferred_name);
            if (ToLower(device_name).find(preferred_name_lower) == std::string::npos)
            {
                continue;
            }
        }

        if (IsLikelyGamepad(device_name, joystick->axisCount(), joystick->buttonCount()))
        {
            this->sys_js = std::move(joystick);
            this->sys_js_device = device;
            this->sys_js_active = false;
            std::fill(std::begin(this->sys_js_axis), std::end(this->sys_js_axis), 0);
            std::cout << LOGGER::INFO << "Joystick connected: " << this->sys_js_device
                      << " [" << device_name << ", axes=" << int(this->sys_js->axisCount())
                      << ", buttons=" << int(this->sys_js->buttonCount()) << "]" << std::endl;
            return;
        }

        fallback_devices.push_back(device);
        std::cout << LOGGER::INFO << "Ignoring low-confidence joystick candidate: " << device
                  << " [" << device_name << ", axes=" << int(joystick->axisCount())
                  << ", buttons=" << int(joystick->buttonCount()) << "]" << std::endl;
    }

    for (const auto& device : fallback_devices)
    {
        if (this->TryOpenSysJoystick(device))
        {
            std::cout << LOGGER::WARNING << "Falling back to joystick with weak gamepad signature: "
                      << device << std::endl;
            return;
        }
    }

    this->sys_js.reset();
    this->sys_js_device.clear();
    std::cout << LOGGER::WARNING
              << "No joystick found in /dev/input/js0-9. "
              << "Use RL_SIM_JOYSTICK_DEVICE=/dev/input/jsN or RL_SIM_JOYSTICK_NAME=<substring> to force one."
              << std::endl;
}

void RL_Sim::GetSysJoystick()
{
    // Clear all button event states
    for (int i = 0; i < 20; ++i)
    {
        this->sys_js_button[i].on_press = false;
        this->sys_js_button[i].on_release = false;
    }

    // Check if joystick is valid before using
    if (!this->sys_js)
    {
        this->PollUdpCommand();
        return;
    }

    while (this->sys_js->sample(&this->sys_js_event))
    {
        if (this->sys_js_event.isButton())
        {
            this->sys_js_button[this->sys_js_event.number].update(this->sys_js_event.value);
        }
        else if (this->sys_js_event.isAxis())
        {
            double normalized = double(this->sys_js_event.value) / this->sys_js_max_value;
            if (std::abs(normalized) < this->axis_deadzone)
            {
                this->sys_js_axis[this->sys_js_event.number] = 0;
            }
            else
            {
                this->sys_js_axis[this->sys_js_event.number] = this->sys_js_event.value;
            }
        }
    }

    const bool left_trigger_pressed =
        this->sys_js_button[6].pressed ||
        this->sys_js_axis[2] > (this->sys_js_max_value / 2);
    const bool right_trigger_pressed =
        this->sys_js_button[7].pressed ||
        this->sys_js_axis[5] > (this->sys_js_max_value / 2);

    if (this->sys_js_button[0].on_press) this->control.SetGamepad(Input::Gamepad::A);
    if (this->sys_js_button[1].on_press) this->control.SetGamepad(Input::Gamepad::B);
    if (this->sys_js_button[2].on_press) this->control.SetGamepad(Input::Gamepad::X);
    if (this->sys_js_button[3].on_press) this->control.SetGamepad(Input::Gamepad::Y);
    if (this->sys_js_button[4].on_press) this->control.SetGamepad(Input::Gamepad::LB);
    if (this->sys_js_button[5].on_press) this->control.SetGamepad(Input::Gamepad::RB);
    if (this->sys_js_button[9].on_press) this->control.SetGamepad(Input::Gamepad::LStick);
    if (this->sys_js_button[10].on_press) this->control.SetGamepad(Input::Gamepad::RStick);
    if (this->sys_js_axis[7] < 0) this->control.SetGamepad(Input::Gamepad::DPadUp);
    if (this->sys_js_axis[7] > 0) this->control.SetGamepad(Input::Gamepad::DPadDown);
    if (this->sys_js_axis[6] > 0) this->control.SetGamepad(Input::Gamepad::DPadLeft);
    if (this->sys_js_axis[6] < 0) this->control.SetGamepad(Input::Gamepad::DPadRight);
    if (this->sys_js_button[4].pressed && this->sys_js_button[0].on_press) this->control.SetGamepad(Input::Gamepad::LB_A);
    if (this->sys_js_button[4].pressed && this->sys_js_button[1].on_press) this->control.SetGamepad(Input::Gamepad::LB_B);
    if (this->sys_js_button[4].pressed && this->sys_js_button[2].on_press) this->control.SetGamepad(Input::Gamepad::LB_X);
    if (this->sys_js_button[4].pressed && this->sys_js_button[3].on_press) this->control.SetGamepad(Input::Gamepad::LB_Y);
    if (this->sys_js_button[4].pressed && this->sys_js_button[9].on_press) this->control.SetGamepad(Input::Gamepad::LB_LStick);
    if (this->sys_js_button[4].pressed && this->sys_js_button[10].on_press) this->control.SetGamepad(Input::Gamepad::LB_RStick);
    if (this->sys_js_button[4].pressed && this->sys_js_axis[6] > 0) this->control.SetGamepad(Input::Gamepad::LB_DPadLeft);
    if (this->sys_js_button[4].pressed && this->sys_js_axis[6] < 0) this->control.SetGamepad(Input::Gamepad::LB_DPadRight);
    if (this->sys_js_button[5].pressed && this->sys_js_button[0].on_press) this->control.SetGamepad(Input::Gamepad::RB_A);
    if (this->sys_js_button[5].pressed && this->sys_js_button[1].on_press) this->control.SetGamepad(Input::Gamepad::RB_B);
    if (this->sys_js_button[5].pressed && this->sys_js_button[2].on_press) this->control.SetGamepad(Input::Gamepad::RB_X);
    if (this->sys_js_button[5].pressed && this->sys_js_button[3].on_press) this->control.SetGamepad(Input::Gamepad::RB_Y);
    if (this->sys_js_button[5].pressed && this->sys_js_button[9].on_press) this->control.SetGamepad(Input::Gamepad::RB_LStick);
    if (this->sys_js_button[5].pressed && this->sys_js_button[10].on_press) this->control.SetGamepad(Input::Gamepad::RB_RStick);
    if (this->sys_js_button[5].pressed && this->sys_js_axis[7] < 0) this->control.SetGamepad(Input::Gamepad::RB_DPadUp);
    if (this->sys_js_button[5].pressed && this->sys_js_axis[7] > 0) this->control.SetGamepad(Input::Gamepad::RB_DPadDown);
    if (this->sys_js_button[5].pressed && this->sys_js_axis[6] > 0) this->control.SetGamepad(Input::Gamepad::RB_DPadRight);
    if (this->sys_js_button[5].pressed && this->sys_js_axis[6] < 0) this->control.SetGamepad(Input::Gamepad::RB_DPadLeft);
    if (this->sys_js_button[4].pressed && this->sys_js_button[5].on_press) this->control.SetGamepad(Input::Gamepad::LB_RB);
    if (left_trigger_pressed && right_trigger_pressed) this->control.SetGamepad(Input::Gamepad::L2_R2);

    float ly = -float(this->sys_js_axis[1]) / float(this->sys_js_max_value);
    float lx = -float(this->sys_js_axis[0]) / float(this->sys_js_max_value);
    float rx = -float(this->sys_js_axis[3]) / float(this->sys_js_max_value);

    bool has_input = (ly != 0.0f || lx != 0.0f || rx != 0.0f);

    if (has_input)
    {
        this->control.x = ly * this->GetCommandLimit("max_cmd_x");
        this->control.y = lx * this->GetCommandLimit("max_cmd_y");
        this->control.yaw = rx * this->GetCommandLimit("max_cmd_yaw");
        this->ClampControlCommands();
        this->sys_js_active = true;
    }
    else if (this->sys_js_active)
    {
        this->control.x = 0.0f;
        this->control.y = 0.0f;
        this->control.yaw = 0.0f;
        this->sys_js_active = false;
    }

    this->PollUdpCommand();
}

void RL_Sim::InitUdpCommandReceiver()
{
    this->udp_command_enabled = this->params.Get<bool>("udp_command_enabled", true);
    if (!this->udp_command_enabled)
    {
        return;
    }
    if (this->udp_command_fd >= 0 || this->udp_command_init_attempted)
    {
        return;
    }
    this->udp_command_init_attempted = true;

    const std::string host = this->params.Get<std::string>("udp_command_host", "0.0.0.0");
    const int port = this->params.Get<int>("udp_command_port", 5555);
    this->udp_command_timeout = std::max(this->params.Get<float>("udp_command_timeout", 0.5f), 0.0f);

    this->udp_command_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (this->udp_command_fd < 0)
    {
        std::cout << LOGGER::WARNING << "[UDP Command] socket() failed: " << std::strerror(errno) << std::endl;
        this->udp_command_enabled = false;
        return;
    }

    int reuse = 1;
    setsockopt(this->udp_command_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<uint16_t>(port));
    if (host.empty() || host == "0.0.0.0")
    {
        address.sin_addr.s_addr = INADDR_ANY;
    }
    else if (inet_pton(AF_INET, host.c_str(), &address.sin_addr) != 1)
    {
        std::cout << LOGGER::WARNING << "[UDP Command] invalid bind host: " << host << std::endl;
        this->CloseUdpCommandReceiver();
        this->udp_command_enabled = false;
        return;
    }

    if (bind(this->udp_command_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0)
    {
        std::cout << LOGGER::WARNING << "[UDP Command] bind(" << host << ":" << port
                  << ") failed: " << std::strerror(errno) << std::endl;
        this->CloseUdpCommandReceiver();
        this->udp_command_enabled = false;
        return;
    }

    int flags = fcntl(this->udp_command_fd, F_GETFL, 0);
    if (flags >= 0)
    {
        fcntl(this->udp_command_fd, F_SETFL, flags | O_NONBLOCK);
    }

    this->udp_command_last_packet_time = std::chrono::steady_clock::now();
    std::cout << LOGGER::INFO << "[UDP Command] listening on " << host << ":" << port
              << " (packet: vx vy wz)" << std::endl;
}

void RL_Sim::CloseUdpCommandReceiver()
{
    if (this->udp_command_fd >= 0)
    {
        close(this->udp_command_fd);
        this->udp_command_fd = -1;
    }
    this->udp_command_active = false;
}

void RL_Sim::PollUdpCommand()
{
    if (this->udp_command_fd < 0 && this->params.Get<bool>("udp_command_enabled", true) && !this->udp_command_init_attempted)
    {
        this->InitUdpCommandReceiver();
    }
    if (!this->udp_command_enabled || this->udp_command_fd < 0)
    {
        return;
    }

    bool received_packet = false;
    while (true)
    {
        char buffer[256] = {};
        sockaddr_in source{};
        socklen_t source_len = sizeof(source);
        const ssize_t bytes = recvfrom(
            this->udp_command_fd,
            buffer,
            sizeof(buffer) - 1,
            0,
            reinterpret_cast<sockaddr*>(&source),
            &source_len);

        if (bytes < 0)
        {
            if (errno != EAGAIN && errno != EWOULDBLOCK)
            {
                std::cout << LOGGER::WARNING << "[UDP Command] recvfrom failed: " << std::strerror(errno) << std::endl;
            }
            break;
        }
        if (bytes == 0)
        {
            continue;
        }

        buffer[bytes] = '\0';
        std::istringstream stream(buffer);
        float vx = 0.0f;
        float vy = 0.0f;
        float wz = 0.0f;
        if (!(stream >> vx >> vy >> wz))
        {
            continue;
        }

        this->udp_command = {vx, vy, wz};
        this->udp_command_last_packet_time = std::chrono::steady_clock::now();
        this->udp_command_active = true;
        received_packet = true;

        if (!this->udp_command_source_reported)
        {
            char source_ip[INET_ADDRSTRLEN] = {};
            inet_ntop(AF_INET, &source.sin_addr, source_ip, sizeof(source_ip));
            std::cout << LOGGER::INFO << "[UDP Command] receiving from "
                      << source_ip << ":" << ntohs(source.sin_port) << std::endl;
            this->udp_command_source_reported = true;
        }
    }

    bool timed_out = false;
    if (this->udp_command_active && this->udp_command_timeout > 0.0f)
    {
        const float stale_s = std::chrono::duration<float>(
            std::chrono::steady_clock::now() - this->udp_command_last_packet_time).count();
        if (stale_s > this->udp_command_timeout)
        {
            this->udp_command = {0.0f, 0.0f, 0.0f};
            this->udp_command_active = false;
            timed_out = true;
        }
    }

    if (this->udp_command_active || received_packet)
    {
        this->control.x = this->udp_command[0];
        this->control.y = this->udp_command[1];
        this->control.yaw = this->udp_command[2];
        this->ClampControlCommands();
    }
    else if (timed_out && !this->sys_js_active)
    {
        this->control.x = 0.0f;
        this->control.y = 0.0f;
        this->control.yaw = 0.0f;
        this->ClampControlCommands();
    }
}

void RL_Sim::RunModel()
{
    if (this->rl_init_done && simulation_running)
    {
        this->episode_length_buf += 1;
        this->obs.ang_vel = this->robot_state.imu.gyroscope;
        this->obs.commands = {this->control.x, this->control.y, this->control.yaw};
        //not currently available for non-ros mujoco version
        // if (this->control.navigation_mode)
        // {
        //     this->obs.commands = {(float)this->cmd_vel.linear.x, (float)this->cmd_vel.linear.y, (float)this->cmd_vel.angular.z};
        // }
        this->obs.base_quat = this->robot_state.imu.quaternion;
        this->obs.dof_pos = this->robot_state.motor_state.q;
        this->obs.dof_vel = this->robot_state.motor_state.dq;

        this->obs.actions = this->Forward();
        this->ComputeOutput(this->obs.actions, this->output_dof_pos, this->output_dof_vel, this->output_dof_tau);

        if (!this->output_dof_pos.empty())
        {
            output_dof_pos_queue.push(this->output_dof_pos);
        }
        if (!this->output_dof_vel.empty())
        {
            output_dof_vel_queue.push(this->output_dof_vel);
        }
        if (!this->output_dof_tau.empty())
        {
            output_dof_tau_queue.push(this->output_dof_tau);
        }

        // this->TorqueProtect(this->output_dof_tau);
        // this->AttitudeProtect(this->robot_state.imu.quaternion, 75.0f, 75.0f);

#ifdef CSV_LOGGER
        std::vector<float> tau_est(this->params.Get<int>("num_of_dofs"), 0.0f);
        for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
        {
            tau_est[i] = this->joint_efforts[this->params.Get<std::vector<std::string>>("joint_controller_names")[i]];
        }
        this->CSVLogger(this->output_dof_tau, tau_est, this->obs.dof_pos, this->output_dof_pos, this->obs.dof_vel);
#endif
    }
}

std::vector<float> RL_Sim::Forward()
{
    std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);

    // If model is being reinitialized, return previous actions to avoid blocking
    if (!lock.owns_lock())
    {
        std::cout << LOGGER::WARNING << "Model is being reinitialized, using previous actions" << std::endl;
        return this->obs.actions;
    }

    std::vector<float> direct_obs = this->ComputeObservation();

    std::vector<float> actions;
    if (this->params.Get<std::vector<int>>("observations_history").size() != 0)
    {
        if (this->history_obs.empty())
        {
            this->history_obs_buf.reset({0}, direct_obs);
        }
        this->history_obs_buf.insert(direct_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(this->params.Get<std::vector<int>>("observations_history"));
        const bool history_two_inputs =
            this->params.Get<bool>("history_two_inputs", false)
            || this->params.Get<bool>("dreamwaq_two_inputs", false)
            || this->model->get_input_count() >= 2;
        if (history_two_inputs)
        {
            actions = this->model->forward({direct_obs, this->history_obs});
        }
        else
        {
            actions = this->model->forward({this->history_obs});
        }
    }
    else
    {
        actions = this->model->forward({direct_obs});
    }

    if (!this->params.Get<std::vector<float>>("clip_actions_upper").empty() && !this->params.Get<std::vector<float>>("clip_actions_lower").empty())
    {
        return clamp(actions, this->params.Get<std::vector<float>>("clip_actions_lower"), this->params.Get<std::vector<float>>("clip_actions_upper"));
    }
    else
    {
        return actions;
    }
}

#ifdef PLOT
void RL_Sim::Plot()
{
    this->plot_t.erase(this->plot_t.begin());
    this->plot_t.push_back(this->motiontime);
    plt::cla();
    plt::clf();
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        this->plot_real_joint_pos[i].erase(this->plot_real_joint_pos[i].begin());
        this->plot_target_joint_pos[i].erase(this->plot_target_joint_pos[i].begin());
        this->plot_real_joint_pos[i].push_back(mj_data->sensordata[i]);
        // this->plot_target_joint_pos[i].push_back();  // TODO
        plt::subplot(this->params.Get<int>("num_of_dofs"), 1, i + 1);
        plt::named_plot("_real_joint_pos", this->plot_t, this->plot_real_joint_pos[i], "r");
        plt::named_plot("_target_joint_pos", this->plot_t, this->plot_target_joint_pos[i], "b");
        plt::xlim(this->plot_t.front(), this->plot_t.back());
    }
    plt::pause(0.01);
}
#endif

// Signal handler for Ctrl+C
void signalHandler(int signum)
{
    std::cout << LOGGER::INFO << "Received signal " << signum << ", exiting..." << std::endl;
    if (RL_Sim::instance && RL_Sim::instance->sim)
    {
        RL_Sim::instance->sim->exitrequest.store(1);
    }
}

int main(int argc, char **argv)
{
    signal(SIGINT, signalHandler);
    RL_Sim rl_sar(argc, argv);
    return 0;
}
