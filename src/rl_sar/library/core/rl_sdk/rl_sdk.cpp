/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_sdk.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace
{
std::string ToLowerCopy(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return text;
}

std::vector<float> RearFootstandFrameFromBody(const std::vector<float>& vec_body)
{
    if (vec_body.size() < 3)
    {
        return vec_body;
    }

    return {-vec_body[2], vec_body[1], vec_body[0]};
}

bool ContainsIndex(const std::vector<int>& values, int index)
{
    return std::find(values.begin(), values.end(), index) != values.end();
}

std::vector<int> GetLegDofIndices(const YamlParams& params)
{
    const int num_dofs = params.Get<int>("num_of_dofs", 0);
    const auto wheel_indices = params.Get<std::vector<int>>("wheel_indices", {});

    std::vector<int> leg_indices;
    leg_indices.reserve(std::max(num_dofs, 0));
    for (int i = 0; i < num_dofs; ++i)
    {
        if (!ContainsIndex(wheel_indices, i))
        {
            leg_indices.push_back(i);
        }
    }
    return leg_indices;
}

std::vector<float> SelectDofs(const std::vector<float>& values, const std::vector<int>& indices)
{
    std::vector<float> selected;
    selected.reserve(indices.size());
    for (int index : indices)
    {
        if (index >= 0 && index < static_cast<int>(values.size()))
        {
            selected.push_back(values[index]);
        }
    }
    return selected;
}

std::string MakeTimestampForFilename()
{
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm local_time{};
    localtime_r(&now_time, &local_time);

    std::ostringstream ss;
    ss << std::put_time(&local_time, "%Y%m%d_%H%M%S");
    return ss.str();
}

float GetOrZero(const std::vector<float>& values, int index)
{
    if (index < 0 || index >= static_cast<int>(values.size()))
    {
        return 0.0f;
    }
    return values[index];
}

void WriteHeaderVector(std::ofstream& file, const std::string& prefix, int size)
{
    for (int i = 0; i < size; ++i)
    {
        file << prefix << "_" << i << ",";
    }
}

void WriteValues(std::ofstream& file, const std::vector<float>& values, int size)
{
    for (int i = 0; i < size; ++i)
    {
        file << GetOrZero(values, i) << ",";
    }
}
}

void RL::StateController(const RobotState<float>* state, RobotCommand<float>* command)
{
    auto updateState = [&](std::shared_ptr<FSMState> statePtr)
    {
        if (auto rl_fsm_state = std::dynamic_pointer_cast<RLFSMState>(statePtr))
        {
            rl_fsm_state->fsm_state = state;
            rl_fsm_state->fsm_command = command;
        }
    };
    for (auto& pair : fsm.states_)
    {
        updateState(pair.second);
    }

    fsm.Run();

    this->motiontime++;

    if (this->control.current_keyboard == Input::Keyboard::W)
    {
        this->control.x += 0.1f;
    }
    if (this->control.current_keyboard == Input::Keyboard::S)
    {
        this->control.x -= 0.1f;
    }
    if (this->control.current_keyboard == Input::Keyboard::A)
    {
        this->control.y += 0.1f;
    }
    if (this->control.current_keyboard == Input::Keyboard::D)
    {
        this->control.y -= 0.1f;
    }
    if (this->control.current_keyboard == Input::Keyboard::Q)
    {
        this->control.yaw += 0.1f;
    }
    if (this->control.current_keyboard == Input::Keyboard::E)
    {
        this->control.yaw -= 0.1f;
    }
    if (this->control.current_keyboard == Input::Keyboard::Space)
    {
        this->control.x = 0.0f;
        this->control.y = 0.0f;
        this->control.yaw = 0.0f;
    }
    if (this->control.current_keyboard == Input::Keyboard::N || this->control.current_gamepad == Input::Gamepad::X)
    {
        this->control.navigation_mode = !this->control.navigation_mode;
        std::cout << std::endl << LOGGER::INFO << "Navigation mode: " << (this->control.navigation_mode ? "ON" : "OFF") << std::endl;
    }

    this->ClampControlCommands();
}

float RL::GetCommandLimit(const std::string& key, float default_value) const
{
    return std::max(this->params.Get<float>(key, default_value), 0.0f);
}

std::string RL::GetObservationFrame() const
{
    return ToLowerCopy(this->params.Get<std::string>("observation_frame", "body"));
}

std::string RL::GetCommandFrame() const
{
    return ToLowerCopy(this->params.Get<std::string>("command_frame", "body"));
}

std::vector<float> RL::AdaptRootVectorToObservationFrame(const std::vector<float>& vec_body) const
{
    if (GetObservationFrame() == "footstand_rear")
    {
        return RearFootstandFrameFromBody(vec_body);
    }
    return vec_body;
}

std::vector<float> RL::AdaptCommandToPolicyFrame(const std::vector<float>& command) const
{
    const std::string command_frame = GetCommandFrame();
    const std::string observation_frame = GetObservationFrame();

    // Planar commands are already entered in the policy frame semantics.
    // For footstand_rear this means:
    // x -> rear-footstand forward, y -> rear-footstand lateral, yaw -> rear-footstand yaw.
    if (command_frame != observation_frame)
    {
        return command;
    }
    return command;
}

void RL::ClampControlCommands()
{
    const float max_cmd_x = this->GetCommandLimit("max_cmd_x");
    const float max_cmd_y = this->GetCommandLimit("max_cmd_y");
    const float max_cmd_yaw = this->GetCommandLimit("max_cmd_yaw");

    this->control.x = std::clamp(this->control.x, -max_cmd_x, max_cmd_x);
    this->control.y = std::clamp(this->control.y, -max_cmd_y, max_cmd_y);
    this->control.yaw = std::clamp(this->control.yaw, -max_cmd_yaw, max_cmd_yaw);
}

std::vector<float> RL::ComputeObservation()
{
    std::vector<std::vector<float>> obs_list;

    for (const std::string &observation : this->params.Get<std::vector<std::string>>("observations"))
    {
        // ============= Base Observations =============
        if (observation == "lin_vel")
        {
            obs_list.push_back(AdaptRootVectorToObservationFrame(this->obs.lin_vel) * this->params.Get<float>("lin_vel_scale"));
        }
        else if (observation == "ang_vel")
        {
            // In ROS1 Gazebo, the coordinate system for angular velocity is in the world coordinate system.
            // In ROS2 Gazebo, mujoco and real robot, the coordinate system for angular velocity is in the body coordinate system.
            if (this->ang_vel_axis == "body")
            {
                obs_list.push_back(AdaptRootVectorToObservationFrame(this->obs.ang_vel) * this->params.Get<float>("ang_vel_scale"));
            }
            else if (this->ang_vel_axis == "world")
            {
                obs_list.push_back(
                    AdaptRootVectorToObservationFrame(QuatRotateInverse(this->obs.base_quat, this->obs.ang_vel))
                    * this->params.Get<float>("ang_vel_scale")
                );
            }
        }
        else if (observation == "gravity_vec")
        {
            obs_list.push_back(AdaptRootVectorToObservationFrame(QuatRotateInverse(this->obs.base_quat, this->obs.gravity_vec)));
        }
        else if (observation == "commands")
        {
            obs_list.push_back(AdaptCommandToPolicyFrame(this->obs.commands) * this->params.Get<std::vector<float>>("commands_scale"));
        }
        else if (observation == "dof_pos")
        {
            std::vector<float> dof_pos_rel = this->obs.dof_pos - this->params.Get<std::vector<float>>("default_dof_pos");
            for (int i : this->params.Get<std::vector<int>>("wheel_indices"))
            {
                dof_pos_rel[i] = 0.0f;
            }
            obs_list.push_back(dof_pos_rel * this->params.Get<float>("dof_pos_scale"));
        }
        else if (observation == "dof_vel")
        {
            obs_list.push_back(this->obs.dof_vel * this->params.Get<float>("dof_vel_scale"));
        }
        else if (observation == "leg_dof_pos")
        {
            std::vector<float> dof_pos_rel = this->obs.dof_pos - this->params.Get<std::vector<float>>("default_dof_pos");
            obs_list.push_back(SelectDofs(dof_pos_rel, GetLegDofIndices(this->params)) * this->params.Get<float>("dof_pos_scale"));
        }
        else if (observation == "leg_dof_vel")
        {
            obs_list.push_back(SelectDofs(this->obs.dof_vel, GetLegDofIndices(this->params)) * this->params.Get<float>("dof_vel_scale"));
        }
        else if (observation == "wheel_dof_vel")
        {
            obs_list.push_back(SelectDofs(this->obs.dof_vel, this->params.Get<std::vector<int>>("wheel_indices")) * this->params.Get<float>("dof_vel_scale"));
        }
        else if (observation == "actions")
        {
            obs_list.push_back(this->obs.actions);
        }
        else if (observation == "joint_fault_vector")
        {
            obs_list.push_back(this->GetJointFaultVector());
        }
        // ============= Other Observations =============
        else if (observation == "whole_body_tracking/motion_command")
        {
            std::vector<float> motion_cmd;
            if (this->motion_loader)
            {
                auto joint_pos_sdk = this->motion_loader->GetJointPos();
                auto joint_vel_sdk = this->motion_loader->GetJointVel();
                auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
                std::vector<float> joint_pos_training(joint_mapping.size());
                std::vector<float> joint_vel_training(joint_mapping.size());
                for (size_t i = 0; i < joint_mapping.size(); ++i)
                {
                    joint_pos_training[i] = joint_pos_sdk[joint_mapping[i]];
                    joint_vel_training[i] = joint_vel_sdk[joint_mapping[i]];
                }
                motion_cmd.insert(motion_cmd.end(), joint_pos_training.begin(), joint_pos_training.end());
                motion_cmd.insert(motion_cmd.end(), joint_vel_training.begin(), joint_vel_training.end());
            }
            else
            {
                motion_cmd.resize(this->params.Get<int>("num_of_dofs") * 2, 0.0f);
            }
            obs_list.push_back(motion_cmd);
        }
        else if (observation == "whole_body_tracking/motion_anchor_ori_b")
        {
            std::vector<float> anchor_ori(6, 0.0f);
            if (this->motion_loader)
            {
                auto waist_sdk_indices = this->params.Get<std::vector<int>>("waist_joint_indices");
                std::vector<float> waist_angles = {
                    this->obs.dof_pos[InverseJointMapping(waist_sdk_indices[0])],
                    this->obs.dof_pos[InverseJointMapping(waist_sdk_indices[1])],
                    this->obs.dof_pos[InverseJointMapping(waist_sdk_indices[2])]
                };
                std::vector<float> robot_torso_quat_w = MotionLoader::ComputeTorsoQuat(this->obs.base_quat, waist_angles);
                std::vector<float> ref_torso_quat_w = this->motion_loader->GetAnchorQuat();
                std::vector<float> init_quat = this->motion_loader->GetInitQuat();
                std::vector<float> motion_anchor_quat_w = QuaternionMultiply(init_quat, ref_torso_quat_w);
                std::vector<float> robot_quat_inv = QuaternionConjugate(robot_torso_quat_w);
                std::vector<float> relative_quat = QuaternionMultiply(robot_quat_inv, motion_anchor_quat_w);
                std::vector<float> rot_matrix = QuaternionToRotationMatrix(relative_quat);
                anchor_ori = MatrixFirstTwoColumns(rot_matrix);
            }
            obs_list.push_back(anchor_ori);
        }
        else if (observation == "RoboMimic_Deploy/phase")
        {
            float motion_time = this->episode_length_buf * this->params.Get<float>("dt") * this->params.Get<int>("decimation");
            float count = motion_time;
            float phase = count / this->motion_length;
            std::vector<float> phase_vec = {phase};
            obs_list.push_back(phase_vec);
        }
    }

    this->obs_dims.clear();
    for (const auto& obs : obs_list)
    {
       this->obs_dims.push_back(obs.size());
    }

    std::vector<float> obs;
    for (const auto& obs_vec : obs_list)
    {
        obs.insert(obs.end(), obs_vec.begin(), obs_vec.end());
    }
    std::vector<float> clamped_obs = clamp(obs, -this->params.Get<float>("clip_obs"), this->params.Get<float>("clip_obs"));
    return clamped_obs;
}

std::vector<float> RL::GetJointFaultVector() const
{
    int fault_vector_dim = this->params.Get<int>("joint_fault_vector_dim", this->params.Get<int>("num_of_dofs", 0));
    if (fault_vector_dim <= 0)
    {
        fault_vector_dim = static_cast<int>(this->obs.dof_pos.size());
    }
    return std::vector<float>(fault_vector_dim, 0.0f);
}

void RL::InitObservations()
{
    this->obs.lin_vel = {0.0f, 0.0f, 0.0f};
    this->obs.ang_vel = {0.0f, 0.0f, 0.0f};
    this->obs.gravity_vec = {0.0f, 0.0f, -1.0f};
    this->obs.commands = {0.0f, 0.0f, 0.0f};
    this->obs.base_quat = {0.0f, 0.0f, 0.0f, 1.0f};
    this->obs.dof_pos = this->params.Get<std::vector<float>>("default_dof_pos");
    this->obs.dof_vel.clear();
    this->obs.dof_vel.resize(this->params.Get<int>("num_of_dofs"), 0.0f);
    this->obs.actions.clear();
    this->obs.actions.resize(this->params.Get<int>("num_of_dofs"), 0.0f);
    this->ComputeObservation();
}

void RL::InitOutputs()
{
    int num_of_dofs = this->params.Get<int>("num_of_dofs");
    this->output_dof_tau.clear();
    this->output_dof_tau.resize(num_of_dofs, 0.0f);
    this->output_dof_pos = this->params.Get<std::vector<float>>("default_dof_pos");
    this->output_dof_vel.clear();
    this->output_dof_vel.resize(num_of_dofs, 0.0f);
}

void RL::InitControl()
{
    this->control.x = 0.0f;
    this->control.y = 0.0f;
    this->control.yaw = 0.0f;
    this->ClampControlCommands();
}

void RL::InitJointNum(size_t num_joints)
{
    this->robot_state.motor_state.resize(num_joints);
    this->start_state.motor_state.resize(num_joints);
    this->now_state.motor_state.resize(num_joints);
    this->robot_command.motor_command.resize(num_joints);
}

void RL::InitRL(std::string robot_config_path)
{
    std::lock_guard<std::mutex> lock(this->model_mutex);

    this->ReadYaml(robot_config_path, "config.yaml");

    // init joint num first
    this->InitJointNum(this->params.Get<int>("num_of_dofs"));

    // init rl
    this->InitObservations();
    this->InitOutputs();
    this->InitControl();

    // init obs history
    const auto& observations_history = this->params.Get<std::vector<int>>("observations_history");  // avoid dangling reference
    if (!observations_history.empty())
    {
        int history_length = *std::max_element(observations_history.begin(), observations_history.end()) + 1;
        this->history_obs_buf = ObservationBuffer(1, this->obs_dims, history_length, this->params.Get<std::string>("observations_history_priority"));
    }

    // init model
    std::string model_path = std::string(POLICY_DIR) + "/" + robot_config_path + "/" + this->params.Get<std::string>("model_name");
    this->model = InferenceRuntime::ModelFactory::load_model(model_path);
    if (!this->model)
    {
        throw std::runtime_error("Failed to load model from: " + model_path);
    }
    this->model->set_output_index(static_cast<size_t>(this->params.Get<int>("model_output_index", 0)));
    this->CSVInit(robot_config_path);
}

void RL::ComputeOutput(const std::vector<float> &actions, std::vector<float> &output_dof_pos, std::vector<float> &output_dof_vel, std::vector<float> &output_dof_tau)
{
    std::vector<float> actions_scaled = actions * this->params.Get<std::vector<float>>("action_scale");
    std::vector<float> pos_actions_scaled = actions_scaled;
    std::vector<float> vel_actions_scaled(actions.size(), 0.0f);
    for (int i : this->params.Get<std::vector<int>>("wheel_indices"))
    {
        pos_actions_scaled[i] = 0.0f;
        vel_actions_scaled[i] = actions_scaled[i];
    }
    std::vector<float> all_actions_scaled = pos_actions_scaled + vel_actions_scaled;
    output_dof_pos = pos_actions_scaled + this->params.Get<std::vector<float>>("default_dof_pos");
    output_dof_vel = vel_actions_scaled;

    output_dof_tau = this->params.Get<std::vector<float>>("rl_kp") * (all_actions_scaled + this->params.Get<std::vector<float>>("default_dof_pos") - this->obs.dof_pos) - this->params.Get<std::vector<float>>("rl_kd") * this->obs.dof_vel;
    output_dof_tau = clamp(output_dof_tau, -this->params.Get<std::vector<float>>("torque_limits"), this->params.Get<std::vector<float>>("torque_limits"));
}

int RL::InverseJointMapping(int idx) const
{
    auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
    for (size_t i = 0; i < joint_mapping.size(); ++i) {
        if (joint_mapping[i] == idx) return (int)i;
    }
    return -1;
}

void RL::TorqueProtect(const std::vector<float>& origin_output_dof_tau)
{
    std::vector<int> out_of_range_indices;
    std::vector<float> out_of_range_values;
    for (size_t i = 0; i < origin_output_dof_tau.size(); ++i)
    {
        float torque_value = origin_output_dof_tau[i];
        float limit_lower = -this->params.Get<std::vector<float>>("torque_limits")[i];
        float limit_upper = this->params.Get<std::vector<float>>("torque_limits")[i];

        if (torque_value < limit_lower || torque_value > limit_upper)
        {
            out_of_range_indices.push_back(i);
            out_of_range_values.push_back(torque_value);
        }
    }
    if (!out_of_range_indices.empty())
    {
        for (size_t i = 0; i < out_of_range_indices.size(); ++i)
        {
            int index = out_of_range_indices[i];
            float value = out_of_range_values[i];
            float limit_lower = -this->params.Get<std::vector<float>>("torque_limits")[index];
            float limit_upper = this->params.Get<std::vector<float>>("torque_limits")[index];

            std::cout << LOGGER::WARNING << "Torque(" << index + 1 << ")=" << value << " out of range(" << limit_lower << ", " << limit_upper << ")" << std::endl;
        }
        // Just a reminder, no protection
        // this->control.SetKeyboard(Input::Keyboard::P);
        std::cout << LOGGER::INFO << "Switching to STATE_POS_GETDOWN"<< std::endl;
    }
}

void RL::AttitudeProtect(const std::vector<float> &quaternion, float pitch_threshold, float roll_threshold)
{
    // Use QuaternionToEuler from vector_math.hpp
    std::vector<float> euler = QuaternionToEuler(quaternion);
    float roll = euler[0] * 57.2958f;   // Convert to degrees
    float pitch = euler[1] * 57.2958f;

    if (std::fabs(roll) > roll_threshold)
    {
        this->control.SetKeyboard(Input::Keyboard::P);
        std::cout << LOGGER::WARNING << "Roll exceeds " << roll_threshold << " degrees. Current: " << roll << " degrees." << std::endl;
    }
    if (std::fabs(pitch) > pitch_threshold)
    {
        this->control.SetKeyboard(Input::Keyboard::P);
        std::cout << LOGGER::WARNING << "Pitch exceeds " << pitch_threshold << " degrees. Current: " << pitch << " degrees." << std::endl;
    }
}

#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

static int kbhit()
{
    static bool initialized = false;
    static termios original_term;

    // Initialize terminal to non-canonical mode on first call
    if (!initialized)
    {
        tcgetattr(STDIN_FILENO, &original_term);

        termios new_term = original_term;
        new_term.c_lflag &= ~(ICANON | ECHO);  // Disable canonical mode and echo
        new_term.c_cc[VMIN] = 0;   // Non-blocking read
        new_term.c_cc[VTIME] = 0;  // No timeout

        tcsetattr(STDIN_FILENO, TCSANOW, &new_term);

        // Register cleanup function to restore terminal on exit
        static bool cleanup_registered = false;
        if (!cleanup_registered)
        {
            std::atexit([]() {
                tcsetattr(STDIN_FILENO, TCSANOW, &original_term);
            });
            cleanup_registered = true;
        }

        initialized = true;
    }

    // Non-blocking read of a single character
    char c;
    int result = read(STDIN_FILENO, &c, 1);

    return (result == 1) ? (unsigned char)c : -1;
}

void RL::KeyboardInterface()
{
    int c = kbhit();
    if (c > 0)
    {
        switch (c)
        {
        case '0': this->control.SetKeyboard(Input::Keyboard::Num0); break;
        case '1': this->control.SetKeyboard(Input::Keyboard::Num1); break;
        case '2': this->control.SetKeyboard(Input::Keyboard::Num2); break;
        case '3': this->control.SetKeyboard(Input::Keyboard::Num3); break;
        case '4': this->control.SetKeyboard(Input::Keyboard::Num4); break;
        case '5': this->control.SetKeyboard(Input::Keyboard::Num5); break;
        case '6': this->control.SetKeyboard(Input::Keyboard::Num6); break;
        case '7': this->control.SetKeyboard(Input::Keyboard::Num7); break;
        case '8': this->control.SetKeyboard(Input::Keyboard::Num8); break;
        case '9': this->control.SetKeyboard(Input::Keyboard::Num9); break;
        case 'a': case 'A': this->control.SetKeyboard(Input::Keyboard::A); break;
        case 'b': case 'B': this->control.SetKeyboard(Input::Keyboard::B); break;
        case 'c': case 'C': this->control.SetKeyboard(Input::Keyboard::C); break;
        case 'd': case 'D': this->control.SetKeyboard(Input::Keyboard::D); break;
        case 'e': case 'E': this->control.SetKeyboard(Input::Keyboard::E); break;
        case 'f': case 'F': this->control.SetKeyboard(Input::Keyboard::F); break;
        case 'g': case 'G': this->control.SetKeyboard(Input::Keyboard::G); break;
        case 'h': case 'H': this->control.SetKeyboard(Input::Keyboard::H); break;
        case 'j': case 'J': this->control.SetKeyboard(Input::Keyboard::J); break;
        case 'k': case 'K': this->control.SetKeyboard(Input::Keyboard::K); break;
        case 'l': case 'L': this->control.SetKeyboard(Input::Keyboard::L); break;
        case 'm': case 'M': this->control.SetKeyboard(Input::Keyboard::M); break;
        case 'n': case 'N': this->control.SetKeyboard(Input::Keyboard::N); break;
        case 'p': case 'P': this->control.SetKeyboard(Input::Keyboard::P); break;
        case 'q': case 'Q': this->control.SetKeyboard(Input::Keyboard::Q); break;
        case 'r': case 'R': this->control.SetKeyboard(Input::Keyboard::R); break;
        case 's': case 'S': this->control.SetKeyboard(Input::Keyboard::S); break;
        case 't': case 'T': this->control.SetKeyboard(Input::Keyboard::T); break;
        case 'u': case 'U': this->control.SetKeyboard(Input::Keyboard::U); break;
        case 'v': case 'V': this->control.SetKeyboard(Input::Keyboard::V); break;
        case 'w': case 'W': this->control.SetKeyboard(Input::Keyboard::W); break;
        case 'x': case 'X': this->control.SetKeyboard(Input::Keyboard::X); break;
        case 'y': case 'Y': this->control.SetKeyboard(Input::Keyboard::Y); break;
        case 'z': case 'Z': this->control.SetKeyboard(Input::Keyboard::Z); break;
        case ' ': this->control.SetKeyboard(Input::Keyboard::Space); break;
        case '\n': case '\r': this->control.SetKeyboard(Input::Keyboard::Enter); break;
        case 27:  // Escape sequence (for arrow keys on Unix/Linux/macOS)
        {
            char seq[2];
            // Try to read escape sequence non-blockingly
            if (read(STDIN_FILENO, &seq[0], 1) == 1)
            {
                if (seq[0] == '[')
                {
                    if (read(STDIN_FILENO, &seq[1], 1) == 1)
                    {
                        switch (seq[1])
                        {
                        case 'A': this->control.SetKeyboard(Input::Keyboard::Up); break;
                        case 'B': this->control.SetKeyboard(Input::Keyboard::Down); break;
                        case 'C': this->control.SetKeyboard(Input::Keyboard::Right); break;
                        case 'D': this->control.SetKeyboard(Input::Keyboard::Left); break;
                        default: break;
                        }
                    }
                }
                else
                {
                    // Plain escape key
                    this->control.SetKeyboard(Input::Keyboard::Escape);
                }
            }
            else
            {
                // Plain escape key
                this->control.SetKeyboard(Input::Keyboard::Escape);
            }
        } break;
        default:  break;
        }
    }
}

template <typename T>
std::vector<T> ReadVectorFromYaml(const YAML::Node &node)
{
    std::vector<T> values;
    for (const auto &val : node)
    {
        values.push_back(val.as<T>());
    }
    return values;
}

void RL::ReadYaml(const std::string& file_path, const std::string& file_name)
{
    std::string config_path = std::string(POLICY_DIR) + "/" + file_path + "/" + file_name;
    YAML::Node config;
    try
    {
        config = YAML::LoadFile(config_path)[file_path];
    }
    catch (YAML::BadFile &e)
    {
        std::cout << LOGGER::ERROR << "The file '" << config_path << "' does not exist" << std::endl;
        return;
    }

    for (auto it = config.begin(); it != config.end(); ++it)
    {
        std::string key = it->first.as<std::string>();
        this->params.config_node[key] = it->second;
    }
}

void RL::CSVInit(std::string robot_path)
{
    this->CSVClose();
    this->csv_logger_enabled = this->params.Get<bool>("diagnostic_log", false);
    if (!this->csv_logger_enabled)
    {
        return;
    }

    this->csv_logger_flush = this->params.Get<bool>("diagnostic_log_flush", false);
    const int num_dofs = this->params.Get<int>("num_of_dofs");
    const int fault_dim = this->params.Get<int>("joint_fault_vector_dim", num_dofs);

    try
    {
        namespace fs = std::filesystem;
        const fs::path log_dir = fs::path(std::string(POLICY_DIR)).parent_path() / "logs" / robot_path;
        fs::create_directories(log_dir);
        this->csv_filename = (log_dir / ("diagnostic_" + MakeTimestampForFilename() + ".csv")).string();
        this->csv_file.open(this->csv_filename.c_str(), std::ios::out);
    }
    catch (const std::exception& e)
    {
        this->csv_logger_enabled = false;
        std::cout << LOGGER::WARNING << "Failed to create diagnostic log: " << e.what() << std::endl;
        return;
    }

    if (!this->csv_file.is_open())
    {
        this->csv_logger_enabled = false;
        std::cout << LOGGER::WARNING << "Failed to open diagnostic log: " << this->csv_filename << std::endl;
        return;
    }

    this->csv_prev_actions = this->obs.actions;
    this->csv_prev_output_dof_pos = this->output_dof_pos;
    if (this->csv_prev_actions.empty())
    {
        this->csv_prev_actions.resize(num_dofs, 0.0f);
    }
    if (this->csv_prev_output_dof_pos.empty())
    {
        this->csv_prev_output_dof_pos = this->params.Get<std::vector<float>>("default_dof_pos");
    }

    this->csv_file << "motiontime,episode_step,cmd_x,cmd_y,cmd_yaw,"
                   << "gyro_x,gyro_y,gyro_z,gravity_x,gravity_y,gravity_z,"
                   << "quat_w,quat_x,quat_y,quat_z,"
                   << "action_l2,action_delta_l2,target_delta_l2,";
    WriteHeaderVector(this->csv_file, "action", num_dofs);
    WriteHeaderVector(this->csv_file, "action_delta", num_dofs);
    WriteHeaderVector(this->csv_file, "q", num_dofs);
    WriteHeaderVector(this->csv_file, "dq", num_dofs);
    WriteHeaderVector(this->csv_file, "q_target", num_dofs);
    WriteHeaderVector(this->csv_file, "q_error", num_dofs);
    WriteHeaderVector(this->csv_file, "q_target_delta", num_dofs);
    WriteHeaderVector(this->csv_file, "dq_target", num_dofs);
    WriteHeaderVector(this->csv_file, "tau_cal", num_dofs);
    WriteHeaderVector(this->csv_file, "tau_est", num_dofs);
    WriteHeaderVector(this->csv_file, "kp", num_dofs);
    WriteHeaderVector(this->csv_file, "kd", num_dofs);
    WriteHeaderVector(this->csv_file, "fault", fault_dim);
    this->csv_file << std::endl;

    std::cout << LOGGER::INFO << "Diagnostic CSV log: " << this->csv_filename << std::endl;
}

void RL::CSVClose()
{
    if (this->csv_file.is_open())
    {
        this->csv_file.flush();
        this->csv_file.close();
        std::cout << LOGGER::INFO << "Diagnostic CSV saved: " << this->csv_filename << std::endl;
    }
    this->csv_logger_enabled = false;
}

void RL::CSVLogger(const std::vector<float>& tau_est)
{
    if (!this->csv_logger_enabled || !this->csv_file.is_open())
    {
        return;
    }

    const int interval = std::max(1, this->params.Get<int>("diagnostic_log_interval", 1));
    if (interval > 1 && this->episode_length_buf % static_cast<unsigned long long>(interval) != 0)
    {
        return;
    }

    const int num_dofs = this->params.Get<int>("num_of_dofs");
    const auto rl_kp = this->params.Get<std::vector<float>>("rl_kp", std::vector<float>(num_dofs, 0.0f));
    const auto rl_kd = this->params.Get<std::vector<float>>("rl_kd", std::vector<float>(num_dofs, 0.0f));
    const auto fault_vector = this->GetJointFaultVector();

    std::vector<float> action_delta(num_dofs, 0.0f);
    std::vector<float> q_error(num_dofs, 0.0f);
    std::vector<float> q_target_delta(num_dofs, 0.0f);
    float action_l2 = 0.0f;
    float action_delta_l2 = 0.0f;
    float target_delta_l2 = 0.0f;

    for (int i = 0; i < num_dofs; ++i)
    {
        const float action = GetOrZero(this->obs.actions, i);
        const float prev_action = GetOrZero(this->csv_prev_actions, i);
        const float q = GetOrZero(this->obs.dof_pos, i);
        const float q_target = GetOrZero(this->output_dof_pos, i);
        const float prev_q_target = GetOrZero(this->csv_prev_output_dof_pos, i);

        action_delta[i] = action - prev_action;
        q_error[i] = q_target - q;
        q_target_delta[i] = q_target - prev_q_target;
        action_l2 += action * action;
        action_delta_l2 += action_delta[i] * action_delta[i];
        target_delta_l2 += q_target_delta[i] * q_target_delta[i];
    }

    this->csv_file << std::fixed << std::setprecision(6)
                   << this->motiontime << ","
                   << this->episode_length_buf << ","
                   << this->control.x << ","
                   << this->control.y << ","
                   << this->control.yaw << ","
                   << GetOrZero(this->obs.ang_vel, 0) << ","
                   << GetOrZero(this->obs.ang_vel, 1) << ","
                   << GetOrZero(this->obs.ang_vel, 2) << ","
                   << GetOrZero(this->obs.gravity_vec, 0) << ","
                   << GetOrZero(this->obs.gravity_vec, 1) << ","
                   << GetOrZero(this->obs.gravity_vec, 2) << ","
                   << GetOrZero(this->obs.base_quat, 0) << ","
                   << GetOrZero(this->obs.base_quat, 1) << ","
                   << GetOrZero(this->obs.base_quat, 2) << ","
                   << GetOrZero(this->obs.base_quat, 3) << ","
                   << std::sqrt(action_l2) << ","
                   << std::sqrt(action_delta_l2) << ","
                   << std::sqrt(target_delta_l2) << ",";

    WriteValues(this->csv_file, this->obs.actions, num_dofs);
    WriteValues(this->csv_file, action_delta, num_dofs);
    WriteValues(this->csv_file, this->obs.dof_pos, num_dofs);
    WriteValues(this->csv_file, this->obs.dof_vel, num_dofs);
    WriteValues(this->csv_file, this->output_dof_pos, num_dofs);
    WriteValues(this->csv_file, q_error, num_dofs);
    WriteValues(this->csv_file, q_target_delta, num_dofs);
    WriteValues(this->csv_file, this->output_dof_vel, num_dofs);
    WriteValues(this->csv_file, this->output_dof_tau, num_dofs);
    WriteValues(this->csv_file, tau_est, num_dofs);
    WriteValues(this->csv_file, rl_kp, num_dofs);
    WriteValues(this->csv_file, rl_kd, num_dofs);
    WriteValues(this->csv_file, fault_vector, static_cast<int>(fault_vector.size()));
    this->csv_file << std::endl;

    if (this->csv_logger_flush)
    {
        this->csv_file.flush();
    }

    this->csv_prev_actions = this->obs.actions;
    this->csv_prev_output_dof_pos = this->output_dof_pos;
}

bool RLFSMState::Interpolate(
    float& percent,
    const std::vector<float>& start_pos,
    const std::vector<float>& target_pos,
    float duration_seconds,
    const std::string& description,
    bool use_fixed_gains)
{
    if (percent >= 1.0f)
    {
        return false;
    }

    if (percent == 0.0f)
    {
        float max_diff = 0.0f;
        for (size_t i = 0; i < start_pos.size() && i < target_pos.size(); ++i)
        {
            max_diff = std::max(max_diff, std::abs(start_pos[i] - target_pos[i]));
        }

        if (max_diff < 0.1f)
        {
            percent = 1.0f;
        }
    }

    int required_frames = std::max(1, static_cast<int>(std::ceil(duration_seconds / rl.params.Get<float>("dt"))));
    float step = 1.0f / required_frames;

    percent += step;
    percent = std::min(percent, 1.0f);

    auto kp = use_fixed_gains ? rl.params.Get<std::vector<float>>("fixed_kp") : rl.params.Get<std::vector<float>>("rl_kp");
    auto kd = use_fixed_gains ? rl.params.Get<std::vector<float>>("fixed_kd") : rl.params.Get<std::vector<float>>("rl_kd");

    for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
    {
        fsm_command->motor_command.q[i] = (1 - percent) * start_pos[i] + percent * target_pos[i];
        fsm_command->motor_command.dq[i] = 0;
        fsm_command->motor_command.kp[i] = kp[i];
        fsm_command->motor_command.kd[i] = kd[i];
        fsm_command->motor_command.tau[i] = 0;
    }

    if (!description.empty())
    {
        LOGGER::PrintProgress(percent, description);
    }

    if (percent >= 1.0f)
    {
        return false;
    }

    return true;
}

void RLFSMState::RLControl()
{
    std::vector<float> _output_dof_pos, _output_dof_vel;
    if (rl.output_dof_pos_queue.try_pop(_output_dof_pos) && rl.output_dof_vel_queue.try_pop(_output_dof_vel))
    {
        for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
        {
            if (!_output_dof_pos.empty())
            {
                fsm_command->motor_command.q[i] = _output_dof_pos[i];
            }
            if (!_output_dof_vel.empty())
            {
                fsm_command->motor_command.dq[i] = _output_dof_vel[i];
            }
            fsm_command->motor_command.kp[i] = rl.params.Get<std::vector<float>>("rl_kp")[i];
            fsm_command->motor_command.kd[i] = rl.params.Get<std::vector<float>>("rl_kd")[i];
            fsm_command->motor_command.tau[i] = 0;
        }
    }
}
