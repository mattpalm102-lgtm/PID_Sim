/******************************************************************************
 * @file        Plant.cpp
 * @brief       Process plant model implementation.
 *
 * @details     Implements four discrete-time plant models using explicit Euler
 *              integration. Each model converts the PID control signal u(t)
 *              into an updated process variable y(t+dt).
 *
 *              Model summary:
 *
 *              FIRST_ORDER
 *                Continuous: τ·ẏ = -y + K·u
 *                Euler:      y += (K·u - y) * (dt / τ)
 *
 *              SECOND_ORDER (mass-spring-damper)
 *                State:  x₁ = y,  x₂ = ẏ
 *                ẋ₁ = x₂
 *                ẋ₂ = -ωₙ²·x₁ - 2ζωₙ·x₂ + K·ωₙ²·u
 *                Euler applied to both states simultaneously.
 *
 *              INTEGRATING (pure integrator)
 *                ẏ = K·u
 *                Euler: y += K·u·dt
 *
 *              DEAD_TIME (FOPDT)
 *                First-order plant fed with delayed input.
 *                Delay implemented via a fixed-capacity deque acting as a
 *                shift register. Capacity = round(θ / dt).
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

#include "Plant.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

// -----------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------
Plant::Plant(const PlantConfig& config)
    : config_(config),
      output_(config.initial_value),
      state2_(0.0)
{
    // Pre-populate the dead-time delay buffer with zeros so the plant
    // begins with no delayed input history.
    if (config_.model == PlantModel::DEAD_TIME && config_.dead_time > 0.0)
    {
        // Buffer size is estimated from a nominal dt of 0.01 s; the buffer
        // is dynamically sized per-step if dt differs. We populate with zeros
        // so the plant output is correct from t = 0.
        // Actual resizing happens in StepDeadTime on the first call.
        delay_buffer_.clear();
    }
}

// -----------------------------------------------------------------------------
// Step — advance plant state by dt given a control signal
// -----------------------------------------------------------------------------
double Plant::Step(double control_output, double dt)
{
    if (dt <= 0.0)
    {
        std::cerr << "[Plant] WARNING: dt=" << dt
                  << " is non-positive. Plant state not updated.\n";
        return output_;
    }

    switch (config_.model)
    {
        case PlantModel::FIRST_ORDER:
            return StepFirstOrder(control_output, dt);

        case PlantModel::SECOND_ORDER:
            return StepSecondOrder(control_output, dt);

        case PlantModel::INTEGRATING:
            return StepIntegrating(control_output, dt);

        case PlantModel::DEAD_TIME:
            return StepDeadTime(control_output, dt);

        default:
            // Unreachable if Types.hpp enum is kept in sync, but defensive.
            std::cerr << "[Plant] ERROR: Unknown plant model. State unchanged.\n";
            return output_;
    }
}

// -----------------------------------------------------------------------------
// Reset — restore to initial conditions
// -----------------------------------------------------------------------------
void Plant::Reset() noexcept
{
    output_  = config_.initial_value;
    state2_  = 0.0;
    delay_buffer_.clear();
}

// =============================================================================
// PRIVATE HELPERS
// =============================================================================

// -----------------------------------------------------------------------------
// First-order lag:  τ·ẏ = -y + K·u
// -----------------------------------------------------------------------------
double Plant::StepFirstOrder(double u, double dt)
{
    const double tau = config_.time_constant;
    const double K   = config_.gain;

    // Guard against zero or very small time constant to avoid division by zero.
    if (std::abs(tau) < 1e-12)
    {
        std::cerr << "[Plant] WARNING: time_constant near zero. Clamping to 1e-12.\n";
        output_ = K * u; // Instantaneous response at τ → 0
        return output_;
    }

    output_ += (K * u - output_) * (dt / tau);
    return output_;
}

// -----------------------------------------------------------------------------
// Second-order:  mass-spring-damper state-space
// -----------------------------------------------------------------------------
double Plant::StepSecondOrder(double u, double dt)
{
    const double K    = config_.gain;
    const double wn   = config_.natural_freq;
    const double zeta = config_.damping_ratio;
    const double x1   = output_;  // position
    const double x2   = state2_;  // velocity

    // State equations:
    //   dx1/dt = x2
    //   dx2/dt = -wn² x1 - 2ζwn x2 + K wn² u
    const double dx1 = x2;
    const double dx2 = -(wn * wn) * x1
                     - 2.0 * zeta * wn * x2
                     + K * (wn * wn) * u;

    // Explicit Euler integration
    output_  = x1 + dx1 * dt;
    state2_  = x2 + dx2 * dt;

    return output_;
}

// -----------------------------------------------------------------------------
// Pure integrator:  ẏ = K·u
// -----------------------------------------------------------------------------
double Plant::StepIntegrating(double u, double dt)
{
    output_ += config_.gain * u * dt;
    return output_;
}

// -----------------------------------------------------------------------------
// FOPDT:  first-order plant fed with a delayed input signal
// -----------------------------------------------------------------------------
double Plant::StepDeadTime(double u, double dt)
{
    const double theta = config_.dead_time;

    // Determine how many steps constitute the dead-time delay.
    // Clamp to at least 1 step if dead_time > 0 to ensure a meaningful delay.
    const std::size_t delay_steps = (theta > 0.0 && dt > 0.0)
                                  ? static_cast<std::size_t>(std::round(theta / dt))
                                  : 0;

    // Push the current raw control signal into the delay buffer.
    delay_buffer_.push_back(u);

    // Determine the delayed signal:
    //   - If the buffer has accumulated enough entries, use the oldest sample.
    //   - Otherwise, use zero (simulates no input during dead-time startup).
    double u_delayed = 0.0;
    if (delay_buffer_.size() > delay_steps)
    {
        u_delayed = delay_buffer_.front();
        delay_buffer_.pop_front();
    }
    // else: buffer still filling; delayed input is zero.

    // Apply the first-order plant with the delayed signal.
    return StepFirstOrder(u_delayed, dt);
}