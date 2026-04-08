#include "joystick.hh"

#include <cerrno>
#include <fcntl.h>
#include <unistd.h>

JoystickEvent::JoystickEvent(const js_event& event)
    : time(event.time),
      value(event.value),
      type(static_cast<unsigned char>(event.type & ~JS_EVENT_INIT)),
      number(event.number)
{
}

bool JoystickEvent::isButton() const
{
    return (type & JS_EVENT_BUTTON) != 0;
}

bool JoystickEvent::isAxis() const
{
    return (type & JS_EVENT_AXIS) != 0;
}

Joystick::Joystick(const std::string& device_path)
{
    fd_ = open(device_path.c_str(), O_RDONLY | O_NONBLOCK);
}

Joystick::~Joystick()
{
    if (fd_ >= 0)
    {
        close(fd_);
    }
}

bool Joystick::isFound() const
{
    return fd_ >= 0;
}

bool Joystick::sample(JoystickEvent* event)
{
    if (fd_ < 0 || !event)
    {
        return false;
    }

    js_event raw_event{};
    const ssize_t bytes = read(fd_, &raw_event, sizeof(raw_event));
    if (bytes == static_cast<ssize_t>(sizeof(raw_event)))
    {
        *event = JoystickEvent(raw_event);
        return true;
    }

    if (bytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
    {
        return false;
    }

    return false;
}
