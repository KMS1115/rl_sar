/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef GO2_FSM_HPP
#define GO2_FSM_HPP

#include "fsm.hpp"
#include "rl_sdk.hpp"

namespace go2_fsm
{

inline bool IsEstopRequested(const RL& rl)
{
    return rl.control.current_keyboard == Input::Keyboard::P ||
           rl.control.current_gamepad == Input::Gamepad::LB_X ||
           rl.control.current_gamepad == Input::Gamepad::L2_R2;
}

class RLFSMStatePassive : public RLFSMState
{
public:
    RLFSMStatePassive(RL *rl) : RLFSMState(*rl, "RLFSMStatePassive") {}

    void Enter() override
    {
        std::cout << LOGGER::NOTE
                  << "Entered limp standby. Keyboard: 0=Stand, P=E-stop. "
                  << "Gamepad: A=Stand/Sit, B=RL when standing, L2+R2=E-stop."
                  << std::endl;
    }

    void Run() override
    {
        for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
        {
            fsm_command->motor_command.q[i] = fsm_state->motor_state.q[i];
            fsm_command->motor_command.dq[i] = 0;
            fsm_command->motor_command.kp[i] = 0;
            fsm_command->motor_command.kd[i] = 0;
            fsm_command->motor_command.tau[i] = 0;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (IsEstopRequested(rl))
        {
            return "RLFSMStatePassive";
        }
        if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        return state_name_;
    }
};

class RLFSMStateGetUp : public RLFSMState
{
public:
    RLFSMStateGetUp(RL *rl) : RLFSMState(*rl, "RLFSMStateGetUp") {}

    float percent_pre_getup = 0.0f;
    float percent_getup = 0.0f;
    bool stand_from_passive = true;

    void Enter() override
    {
        percent_pre_getup = 0.0f;
        percent_getup = 0.0f;
        if (rl.fsm.previous_state_->GetStateName() == "RLFSMStatePassive")
        {
            stand_from_passive = true;
        }
        else
        {
            stand_from_passive = false;
        }
        rl.now_state = *fsm_state;
        rl.start_state = rl.now_state;
    }

    void Run() override
    {
        const auto seated_dof_pos = rl.params.Get<std::vector<float>>(
            "seated_dof_pos",
            rl.params.Get<std::vector<float>>("startup_dof_pos",
            rl.params.Get<std::vector<float>>("default_dof_pos"))
        );

        if(stand_from_passive)
        {
            if (Interpolate(percent_pre_getup, rl.now_state.motor_state.q, seated_dof_pos, 0.5f, "Seating before stand", true)) return;
            if (Interpolate(percent_getup, seated_dof_pos, rl.params.Get<std::vector<float>>("default_dof_pos"), 1.2f, "Getting up", true)) return;
        }
        else
        {
            if (Interpolate(percent_getup, rl.now_state.motor_state.q, rl.params.Get<std::vector<float>>("default_dof_pos"), 0.8f, "Getting up", true)) return;
        }

        const auto stand_dof_pos = rl.params.Get<std::vector<float>>("default_dof_pos");
        const auto fixed_kp = rl.params.Get<std::vector<float>>("fixed_kp");
        const auto fixed_kd = rl.params.Get<std::vector<float>>("fixed_kd");
        for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
        {
            fsm_command->motor_command.q[i] = stand_dof_pos[i];
            fsm_command->motor_command.dq[i] = 0;
            fsm_command->motor_command.kp[i] = fixed_kp[i];
            fsm_command->motor_command.kd[i] = fixed_kd[i];
            fsm_command->motor_command.tau[i] = 0;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (IsEstopRequested(rl))
        {
            return "RLFSMStatePassive";
        }
        if (percent_getup >= 1.0f)
        {
            if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::B)
            {
                return "RLFSMStateRLLocomotion";
            }
            else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::A)
            {
                return "RLFSMStateGetDown";
            }
        }
        return state_name_;
    }
};

class RLFSMStateGetDown : public RLFSMState
{
public:
    RLFSMStateGetDown(RL *rl) : RLFSMState(*rl, "RLFSMStateGetDown") {}

    float percent_getdown = 0.0f;

    void Enter() override
    {
        percent_getdown = 0.0f;
        rl.now_state = *fsm_state;
    }

    void Run() override
    {
        const auto seated_dof_pos = rl.params.Get<std::vector<float>>(
            "seated_dof_pos",
            rl.params.Get<std::vector<float>>("startup_dof_pos",
            rl.params.Get<std::vector<float>>("default_dof_pos"))
        );
        if (Interpolate(percent_getdown, rl.now_state.motor_state.q, seated_dof_pos, 0.9f, "Getting down", true)) return;

        const auto fixed_kp = rl.params.Get<std::vector<float>>("fixed_kp");
        const auto fixed_kd = rl.params.Get<std::vector<float>>("fixed_kd");
        for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
        {
            fsm_command->motor_command.q[i] = seated_dof_pos[i];
            fsm_command->motor_command.dq[i] = 0;
            fsm_command->motor_command.kp[i] = fixed_kp[i];
            fsm_command->motor_command.kd[i] = fixed_kd[i];
            fsm_command->motor_command.tau[i] = 0;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (IsEstopRequested(rl))
        {
            return "RLFSMStatePassive";
        }
        if (percent_getdown >= 1.0f &&
            (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A))
        {
            return "RLFSMStateGetUp";
        }
        return state_name_;
    }
};

class RLFSMStateRLLocomotion : public RLFSMState
{
public:
    RLFSMStateRLLocomotion(RL *rl) : RLFSMState(*rl, "RLFSMStateRLLocomotion") {}

    float percent_transition = 0.0f;

    void Enter() override
    {
        percent_transition = 0.0f;
        rl.episode_length_buf = 0;

        // read params from yaml
        if (rl.config_name.empty())
        {
            rl.config_name = "default";
        }
        std::string robot_config_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_config_path);
            rl.now_state = *fsm_state;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }
    }

    void Run() override
    {
        // position transition from last default_dof_pos to current default_dof_pos
        // if (Interpolate(percent_transition, rl.now_state.motor_state.q, rl.params.Get<std::vector<float>>("default_dof_pos"), 0.5f, "Policy transition", true)) return;

        if (!rl.rl_init_done) rl.rl_init_done = true;

        std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "RL Controller [" << rl.config_name << "] x:" << rl.control.x << " y:" << rl.control.y << " yaw:" << rl.control.yaw << std::flush;
        RLControl();
    }

    void Exit() override
    {
        rl.rl_init_done = false;
    }

    std::string CheckChange() override
    {
        if (IsEstopRequested(rl))
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateRLLocomotion";
        }
        return state_name_;
    }
};

} // namespace go2_fsm

class Go2FSMFactory : public FSMFactory
{
public:
    Go2FSMFactory(const std::string& initial) : initial_state_(initial) {}
    std::shared_ptr<FSMState> CreateState(void *context, const std::string &state_name) override
    {
        RL *rl = static_cast<RL *>(context);
        if (state_name == "RLFSMStatePassive")
            return std::make_shared<go2_fsm::RLFSMStatePassive>(rl);
        else if (state_name == "RLFSMStateGetUp")
            return std::make_shared<go2_fsm::RLFSMStateGetUp>(rl);
        else if (state_name == "RLFSMStateGetDown")
            return std::make_shared<go2_fsm::RLFSMStateGetDown>(rl);
        else if (state_name == "RLFSMStateRLLocomotion")
            return std::make_shared<go2_fsm::RLFSMStateRLLocomotion>(rl);
        return nullptr;
    }
    std::string GetType() const override { return "go2"; }
    std::vector<std::string> GetSupportedStates() const override
    {
        return {
            "RLFSMStatePassive",
            "RLFSMStateGetUp",
            "RLFSMStateGetDown",
            "RLFSMStateRLLocomotion"
        };
    }
    std::string GetInitialState() const override { return initial_state_; }
private:
    std::string initial_state_;
};

REGISTER_FSM_FACTORY(Go2FSMFactory, "RLFSMStatePassive")

#endif // GO2_FSM_HPP
