/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_sim_mujoco.hpp"

#include <algorithm>
#include <cctype>
#include <iomanip>
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

const char* FaultModeName(RL_Sim::FaultMode mode)
{
    switch (mode)
    {
    case RL_Sim::FaultMode::Locked:
        return "locked";
    case RL_Sim::FaultMode::Weakened:
        return "weakened";
    default:
        return "none";
    }
}
}

RL_Sim* RL_Sim::instance = nullptr;

RL_Sim::RL_Sim(int argc, char **argv)
{
    // Set static instance pointer early for signal handler
    instance = this;

    if (argc < 3)
    {
        std::cout << LOGGER::ERROR << "Usage: " << argv[0] << " robot_name scene_name [config_name]" << std::endl;
        throw std::runtime_error("Invalid arguments");
    }
    else
    {
        this->robot_name = argv[1];
        this->scene_name = argv[2];
        this->config_name = (argc > 3) ? argv[3] : "default";
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
        const auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
        const auto joint_names = this->params.Get<std::vector<std::string>>("joint_names");
        const int num_of_dofs = this->params.Get<int>("num_of_dofs");

        int locked_qpos_adr = -1;
        int locked_dof_adr = -1;
        if (this->fault_mode == FaultMode::Locked && this->fault_joint_idx >= 0 && this->fault_joint_idx < static_cast<int>(joint_names.size()))
        {
            const int joint_id = mj_name2id(this->mj_model, mjOBJ_JOINT, joint_names[this->fault_joint_idx].c_str());
            if (joint_id >= 0)
            {
                locked_qpos_adr = this->mj_model->jnt_qposadr[joint_id];
                locked_dof_adr = this->mj_model->jnt_dofadr[joint_id];
                this->mj_data->qpos[locked_qpos_adr] = this->fault_locked_q;
                this->mj_data->qvel[locked_dof_adr] = 0.0;
            }
        }

        for (int i = 0; i < num_of_dofs; ++i)
        {
            float joint_q = mj_data->sensordata[joint_mapping[i]];
            float joint_dq = mj_data->sensordata[joint_mapping[i] + num_of_dofs];
            float desired_q = command->motor_command.q[i];
            float desired_dq = command->motor_command.dq[i];
            float desired_tau = command->motor_command.tau[i];
            float desired_kp = command->motor_command.kp[i];
            float desired_kd = command->motor_command.kd[i];

            if (this->fault_mode == FaultMode::Locked && i == this->fault_joint_idx)
            {
                if (locked_qpos_adr >= 0 && locked_dof_adr >= 0)
                {
                    joint_q = static_cast<float>(this->mj_data->qpos[locked_qpos_adr]);
                    joint_dq = static_cast<float>(this->mj_data->qvel[locked_dof_adr]);
                }
                desired_q = this->fault_locked_q;
                desired_dq = 0.0f;
                desired_tau = 0.0f;
                desired_kp = std::max(desired_kp, 80.0f);
                desired_kd = std::max(desired_kd, 2.0f);
            }

            float ctrl =
                desired_tau +
                desired_kp * (desired_q - joint_q) +
                desired_kd * (desired_dq - joint_dq);

            if (this->fault_mode == FaultMode::Weakened && i == this->fault_joint_idx)
            {
                ctrl *= this->fault_tau_scale;
            }

            mj_data->ctrl[joint_mapping[i]] = ctrl;
        }
    }
}

void RL_Sim::RobotControl()
{
    // Lock the sim mutex once for the entire control cycle to prevent race conditions
    const std::lock_guard<std::recursive_mutex> lock(sim->mtx);

    this->GetState(&this->robot_state);

    this->StateController(&this->robot_state, &this->robot_command);

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
    if (this->control.current_keyboard == Input::Keyboard::T || this->control.current_gamepad == Input::Gamepad::LB_A)
    {
        this->CycleFaultMode();
    }
    if (this->control.current_keyboard == Input::Keyboard::Y || this->control.current_gamepad == Input::Gamepad::LB_DPadLeft)
    {
        this->SelectFaultJoint(-1);
    }
    if (this->control.current_keyboard == Input::Keyboard::U || this->control.current_gamepad == Input::Gamepad::LB_DPadRight)
    {
        this->SelectFaultJoint(1);
    }
    if (this->control.current_keyboard == Input::Keyboard::I || this->control.current_gamepad == Input::Gamepad::LB_DPadDown)
    {
        this->AdjustFaultSeverity(-0.01f);
    }
    if (this->control.current_keyboard == Input::Keyboard::O || this->control.current_gamepad == Input::Gamepad::LB_DPadUp)
    {
        this->AdjustFaultSeverity(0.01f);
    }

    this->control.ClearInput();

    this->SetCommand(&this->robot_command);
}


std::string RL_Sim::GetFaultJointName() const
{
    const auto joint_names = this->params.Get<std::vector<std::string>>("joint_names");
    if (this->fault_joint_idx >= 0 && this->fault_joint_idx < static_cast<int>(joint_names.size()))
    {
        return joint_names[this->fault_joint_idx];
    }
    return "joint_" + std::to_string(this->fault_joint_idx);
}

bool RL_Sim::TryGetConfiguredLockedJointTarget(float* target_q) const
{
    if (target_q == nullptr)
    {
        return false;
    }

    const std::string joint_name = this->GetFaultJointName();
    if (IsThighJointName(joint_name))
    {
        if (this->params.Has("fault_lock_thigh_q"))
        {
            *target_q = this->params.Get<float>("fault_lock_thigh_q");
            return true;
        }

        const auto seated_dof_pos = this->params.Get<std::vector<float>>("seated_dof_pos");
        if (this->fault_joint_idx >= 0 && this->fault_joint_idx < static_cast<int>(seated_dof_pos.size()))
        {
            *target_q = seated_dof_pos[this->fault_joint_idx];
            return true;
        }
    }

    if (IsCalfJointName(joint_name))
    {
        if (this->params.Has("fault_lock_calf_q"))
        {
            *target_q = this->params.Get<float>("fault_lock_calf_q");
            return true;
        }

        const auto seated_dof_pos = this->params.Get<std::vector<float>>("seated_dof_pos");
        if (this->fault_joint_idx >= 0 && this->fault_joint_idx < static_cast<int>(seated_dof_pos.size()))
        {
            *target_q = seated_dof_pos[this->fault_joint_idx];
            return true;
        }
    }

    return false;
}

void RL_Sim::RefreshLockedJointTarget()
{
    if (!this->mj_data)
    {
        return;
    }

    const auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
    const auto default_dof_pos = this->params.Get<std::vector<float>>("default_dof_pos");
    const int num_of_dofs = this->params.Get<int>("num_of_dofs");
    if (this->fault_joint_idx < 0 || this->fault_joint_idx >= num_of_dofs)
    {
        return;
    }

    float configured_target_q = 0.0f;
    if (this->TryGetConfiguredLockedJointTarget(&configured_target_q))
    {
        const auto joint_names = this->params.Get<std::vector<std::string>>("joint_names");
        if (this->mj_model && this->fault_joint_idx < static_cast<int>(joint_names.size()))
        {
            const int joint_id = mj_name2id(this->mj_model, mjOBJ_JOINT, joint_names[this->fault_joint_idx].c_str());
            if (joint_id >= 0 && this->mj_model->jnt_limited[joint_id])
            {
                const double* range = this->mj_model->jnt_range + 2 * joint_id;
                configured_target_q = std::clamp(configured_target_q, static_cast<float>(range[0]), static_cast<float>(range[1]));
            }
        }

        this->fault_locked_q = configured_target_q;
        return;
    }

    const float current_q = this->mj_data->sensordata[joint_mapping[this->fault_joint_idx]];
    const float lower = default_dof_pos[this->fault_joint_idx] - this->fault_lock_half_range;
    const float upper = default_dof_pos[this->fault_joint_idx] + this->fault_lock_half_range;
    this->fault_locked_q = std::clamp(current_q, lower, upper);
}

void RL_Sim::CycleFaultMode()
{
    switch (this->fault_mode)
    {
    case FaultMode::None:
        this->fault_mode = FaultMode::Locked;
        this->RefreshLockedJointTarget();
        break;
    case FaultMode::Locked:
        this->fault_mode = FaultMode::Weakened;
        break;
    default:
        this->fault_mode = FaultMode::None;
        break;
    }
    this->PrintFaultStatus();
}

void RL_Sim::SelectFaultJoint(int delta)
{
    const int num_of_dofs = this->params.Get<int>("num_of_dofs");
    if (num_of_dofs <= 0)
    {
        return;
    }
    this->fault_joint_idx = (this->fault_joint_idx + delta + num_of_dofs) % num_of_dofs;
    if (this->fault_mode == FaultMode::Locked)
    {
        this->RefreshLockedJointTarget();
    }
    this->PrintFaultStatus();
}

void RL_Sim::AdjustFaultSeverity(float delta)
{
    if (this->fault_mode == FaultMode::Locked)
    {
        this->fault_lock_half_range = std::clamp(this->fault_lock_half_range + delta, 0.01f, 0.25f);
        this->RefreshLockedJointTarget();
    }
    else if (this->fault_mode == FaultMode::Weakened)
    {
        this->fault_tau_scale = std::clamp(this->fault_tau_scale + delta, 0.0f, 1.0f);
    }
    this->PrintFaultStatus();
}

void RL_Sim::PrintFaultStatus() const
{
    std::ostringstream message;
    message << LOGGER::INFO << "[DreamFLEX Fault] mode=" << FaultModeName(this->fault_mode)
            << ", joint=" << this->fault_joint_idx << " (" << this->GetFaultJointName() << ")";
    if (this->fault_mode == FaultMode::Locked)
    {
        float configured_target_q = 0.0f;
        if (this->TryGetConfiguredLockedJointTarget(&configured_target_q))
        {
            message << ", q_target=" << std::fixed << std::setprecision(3) << configured_target_q;
        }
        else
        {
            message << ", q_half_range=" << std::fixed << std::setprecision(3) << this->fault_lock_half_range;
        }
    }
    else if (this->fault_mode == FaultMode::Weakened)
    {
        message << ", k_tau=" << std::fixed << std::setprecision(3) << this->fault_tau_scale;
    }
    message << ". Keys: T=cycle fault, Y/U=joint -, +, I/O=severity -, +";
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
    if (this->sys_js_button[4].pressed && this->sys_js_axis[7] < 0) this->control.SetGamepad(Input::Gamepad::LB_DPadUp);
    if (this->sys_js_button[4].pressed && this->sys_js_axis[7] > 0) this->control.SetGamepad(Input::Gamepad::LB_DPadDown);
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
        this->control.x = ly;
        this->control.y = lx;
        this->control.yaw = rx;
        this->sys_js_active = true;
    }
    else if (this->sys_js_active)
    {
        this->control.x = 0.0f;
        this->control.y = 0.0f;
        this->control.yaw = 0.0f;
        this->sys_js_active = false;
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

    std::vector<float> clamped_obs = this->ComputeObservation();

    std::vector<float> actions;
    if (this->params.Get<std::vector<int>>("observations_history").size() != 0)
    {
        if (this->history_obs.empty())
        {
            this->history_obs_buf.reset({0}, clamped_obs);
        }
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(this->params.Get<std::vector<int>>("observations_history"));
        const bool dreamwaq_two_inputs =
            this->params.Get<bool>("dreamwaq_two_inputs", false) || this->model->get_input_count() >= 2;
        if (dreamwaq_two_inputs)
        {
            actions = this->model->forward({clamped_obs, this->history_obs});
        }
        else
        {
            actions = this->model->forward({this->history_obs});
        }
    }
    else
    {
        actions = this->model->forward({clamped_obs});
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
