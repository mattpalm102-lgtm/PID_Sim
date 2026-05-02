/******************************************************************************
 * @file        pid.cpp
 * @brief       PID controller implementation.
 *
 * @details     Implements the PID control algorithm including proportional,
 *              integral, and derivative terms. Designed for use in simulation
 *              for embedded systems with deterministic time steps.
 *
 * @author      Matt Palmer
 * @date        2026-05-01
 * @version     0.1.0
 *
 * @copyright
 * Copyright (c) 2026 Matt Palmer
 *
 * This file is part of the PID Controller Project.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 ******************************************************************************/

#include "PID.hpp"

// -----------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------
PID::PID(double kp_, double ki_, double kd_)
    : kp(kp_),
      ki(ki_),
      kd(kd_),
      integral(0.0),
      prev_error(0.0)
{
}

// -----------------------------------------------------------------------------
// Main PID update
// -----------------------------------------------------------------------------
double PID::Update(double setpoint, double measurement, double dt, double& control_output)
{
    double error = setpoint - measurement;

    // Proportional term
    double p = kp * error;

    // Integral term
    integral += error * dt;
    double i = ki * integral;

    // Derivative term
    double derivative = (dt > 0.0) ? (error - prev_error) / dt : 0.0;
    double d = kd * derivative;

    prev_error = error;

    // Compute final output
    control_output = p + i + d;

    return control_output;
}

// -----------------------------------------------------------------------------
// Reset internal state
// -----------------------------------------------------------------------------
void PID::Reset() noexcept
{
    integral = 0.0;
    prev_error = 0.0;
}