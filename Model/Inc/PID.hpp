/******************************************************************************
 * @file        PID.hpp
 * @brief       Industrial-grade PID controller interface.
 *
 * @details     Provides a fully configurable PID controller with:
 *                - Proportional, integral, and derivative control terms
 *                - Output clamping (saturation limiting)
 *                - Integrator anti-windup via back-calculation clamping
 *                - Numerical guard against zero or near-zero time steps
 *                - Full state reset for repeated simulation runs
 *
 *              The controller implements the standard parallel (ideal) PID
 *              form:
 *
 *                u(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·(de/dt)
 *
 *              where e(t) = setpoint − measurement.
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

#ifndef PIDSIM_PID_H
#define PIDSIM_PID_H

#include "Types.hpp"

/**
 * @class PID
 * @brief Parallel-form PID controller with output clamping and anti-windup.
 *
 * ### Typical usage
 * @code
 *   PIDConfig cfg;
 *   cfg.kp = 2.0; cfg.ki = 0.5; cfg.kd = 0.1;
 *   PID controller(cfg);
 *
 *   for (auto& step : time_steps) {
 *       double u = controller.Update(setpoint, measurement, dt);
 *   }
 * @endcode
 *
 * @note This class is NOT thread-safe. Use one instance per thread if
 *       parallel simulation is required.
 */
class PID
{
public:
    // -------------------------------------------------------------------------
    // ----- CONSTRUCTOR / DESTRUCTOR ------------------------------------------
    // -------------------------------------------------------------------------

    /**
     * @brief Constructs a PID controller from a validated configuration.
     * @param config  Fully populated PIDConfig struct. Gains and clamp limits
     *                are assumed to have been validated by the caller (CLI).
     */
    explicit PID(const PIDConfig& config);

    /// Default destructor — no dynamic resources.
    ~PID() = default;

    // -------------------------------------------------------------------------
    // ----- PUBLIC METHODS ----------------------------------------------------
    // -------------------------------------------------------------------------

    /**
     * @brief  Advances the controller by one time step and returns the control output.
     *
     * @details Computes P, I, and D terms, applies anti-windup if enabled,
     *          and clamps the result to [output_min, output_max].
     *
     * @param  setpoint     Desired process variable value (reference signal).
     * @param  measurement  Current process variable reading from the plant.
     * @param  dt           Elapsed time since last call (seconds). Must be > 0.
     *
     * @return Clamped controller output u(t).
     *
     * @note   If dt ≤ 0, the derivative term is zeroed and a warning is issued
     *         via stderr; the update still proceeds safely.
     */
    double Update(double setpoint, double measurement, double dt);

    /**
     * @brief  Resets all integrator and derivative state to zero.
     *
     * @details Call this before starting a new simulation run to ensure the
     *          controller begins from a clean initial state.
     */
    void Reset() noexcept;

    // -------------------------------------------------------------------------
    // ----- ACCESSORS ---------------------------------------------------------
    // -------------------------------------------------------------------------

    /// Returns current configuration (read-only).
    const PIDConfig& GetConfig() const noexcept { return config_; }

    /// Returns the accumulated integral term value (useful for diagnostics).
    double GetIntegral() const noexcept { return integral_; }

    /// Returns the error from the previous Update call.
    double GetPreviousError() const noexcept { return prev_error_; }

private:
    // -------------------------------------------------------------------------
    // ----- PRIVATE DATA MEMBERS ----------------------------------------------
    // -------------------------------------------------------------------------

    PIDConfig config_;      ///< Immutable controller configuration
    double    integral_;    ///< Accumulated integral state ∫e(t)dt
    double    prev_error_;  ///< Error value from the previous time step (for derivative)
};

#endif // PIDSIM_PID_H