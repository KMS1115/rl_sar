#ifndef RL_SAR_JOYSTICK_HH
#define RL_SAR_JOYSTICK_HH

#include <linux/joystick.h>

#include <string>

class JoystickEvent
{
public:
    JoystickEvent() = default;
    explicit JoystickEvent(const js_event& event);

    bool isButton() const;
    bool isAxis() const;

    unsigned int time = 0;
    short value = 0;
    unsigned char type = 0;
    unsigned char number = 0;
};

class Joystick
{
public:
    explicit Joystick(const std::string& device_path);
    ~Joystick();

    bool isFound() const;
    bool sample(JoystickEvent* event);

private:
    int fd_ = -1;
};

#endif
