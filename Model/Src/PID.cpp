/******************************************************************************
 * @file        PID.cpp
 * @brief       Industrial-grade PID controller implementation.
 *
 * @details     Implements the parallel-form PID algorithm:
 *
 *                u(t) = Kp·e(t)  +  Ki·∫e(t)dt  +  Kd·(de/dt)
 *
 *              Features:
 *                - Derivative-on-error (standard form)
 *                - Forward Euler integration of the integral term
 *                - Backward difference approximation of the derivative
 *                - Output clamping to [output_min, output_max]
 *                - Anti-windup: integrator accumulation is prevented when the
 *                  pre-clamp output would saturate the actuator, keeping the
 *                  integral from growing unboundedly during sustained error
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
#include <algorithm>
#include <iostream>

// -----------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------
PID::PID(const PIDConfig& config)
    : config_(config),
      integral_(0.0),
      prev_error_(0.0)
{
}

// -----------------------------------------------------------------------------
// Update — advance the controller one time step
// -----------------------------------------------------------------------------
double PID::Update(double setpoint, double measurement, double dt)
{
    // Guard against non-positive time steps; the derivative would be undefined
    // and a zero or negative dt indicates a bug in the calling code.
    if (dt <= 0.0)
    {
        std::cerr << "[PID] WARNING: dt=" << dt
                  << " is non-positive. Derivative term zeroed for this step.\n";
        dt = 0.0; // Proceed safely; derivative contribution will be zero.
    }

    // ----- Error signal -------------------------------------------------------
    const double error = setpoint - measurement;

    // ----- Proportional term --------------------------------------------------
    const double p_term = config_.kp * error;

    // ----- Integral term (with optional anti-windup) --------------------------
    // Tentatively accumulate the integral.
    const double integral_candidate = integral_ + error * dt;

    // Compute un-clamped output to test for saturation before committing.
    double derivative = 0.0;
    if (dt > 0.0)
    {
        derivative = (error - prev_error_) / dt;
    }

    const double i_term_candidate = config_.ki * integral_candidate;
    const double d_term           = config_.kd * derivative;
    const double raw_output       = p_term + i_term_candidate + d_term;

    if (config_.anti_windup)
    {
        // Only commit the integral update if the output is NOT saturated,
        // OR if accumulating would actually drive output back into range.
        const bool saturated_high = raw_output > config_.output_max;
        const bool saturated_low  = raw_output < config_.output_min;
        const bool windup_high    = saturated_high && (error > 0.0);
        const bool windup_low     = saturated_low  && (error < 0.0);

        if (!windup_high && !windup_low)
        {
            integral_ = integral_candidate;
        }
        // else: hold integral at current value — prevents unbounded wind-up.
    }
    else
    {
        integral_ = integral_candidate;
    }

    // ----- Final output with clamping -----------------------------------------
    const double i_term = config_.ki * integral_;
    double output = p_term + i_term + d_term;
    output = std::clamp(output, config_.output_min, config_.output_max);

    // ----- Store state for next call ------------------------------------------
    prev_error_ = error;

    return output;
}

// -----------------------------------------------------------------------------
// Reset — clear all stateful data
// -----------------------------------------------------------------------------
void PID::Reset() noexcept
{
    integral_   = 0.0;
    prev_error_ = 0.0;
}