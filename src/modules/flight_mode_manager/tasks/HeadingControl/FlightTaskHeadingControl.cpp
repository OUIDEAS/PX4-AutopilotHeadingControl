// Flight Task Heading Control

#include "FlightTaskHeadingControl.hpp"
#include <mathlib/mathlib.h>
#include <cmath>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/Subscription.hpp>
#include <uORB/uORB.h>


bool FlightTaskHeadingControl::activate(vehicle_local_position_setpoint_s last_setpoint)
{
  _position_setpoint(0) = NAN;
  _position_setpoint(1) = NAN;
  _position_setpoint(2) = -1.5f;
  return true;
}

bool FlightTaskHeadingControl::update()
{
  uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
  manual_control_setpoint_s pos;

  if(_manual_control_setpoint_sub.update(&pos)){
    _yaw_setpoint = pos.yaw;
    _velocity_setpoint(0) = pos.throttle * cos(_yaw_setpoint);
    _velocity_setpoint(1) = pos.throttle * sin(_yaw_setpoint);
    _position_setpoint(0) = NAN;
    _position_setpoint(1) = NAN;
    _position_setpoint(2) = -1.0f * pos.pitch; // NED coordinates
  }

  _velocity_setpoint(2) = 0.0f;
  if(std::isnan(_yaw_setpoint)){
    _velocity_setpoint(0) = 0;
    _velocity_setpoint(1) = 0;
    _position_setpoint(0) = 0;
    _position_setpoint(1) = 0;
    _position_setpoint(2) = -1.5f; // NED coordinates
  }

  return true;
}