/******************************************************************************
 * @file        Plant.hpp
 * @brief       Simulated process plant interface.
 *
 * @details     Models a physical process (plant) that the PID controller
 *              attempts to regulate. Supported models:
 *
 *                FIRST_ORDER   τ·dy/dt = -y + K·u
 *                              → Euler step: y += (dt/τ)·(-y + K·u)
 *
 *                SECOND_ORDER  mass-spring-damper with natural frequency ωₙ
 *                              and damping ratio ζ. State-space form:
 *                                ẋ₁ = x₂
 *                                ẋ₂ = -ωₙ²·x₁ - 2ζωₙ·x₂ + K·ωₙ²·u
 *                              → Integrated with Euler steps on [x₁, x₂].
 *
 *                INTEGRATING   dy/dt = K·u
 *                              → y += K·u·dt  (pure integration)
 *
 *                DEAD_TIME     First-order plant with a circular delay buffer
 *                              implementing transport delay θ (FOPDT model).
 *
 *              All models use explicit Euler integration. For the simulation
 *              step sizes recommended by the UI (≤ 0.1 s on typical plants)
 *              this is entirely adequate; the step-size guidance in the CLI
 *              ensures numerical stability for all built-in plant models.
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

#ifndef PIDSIM_PLANT_H
#define PIDSIM_PLANT_H

#include "Types.hpp"
#include <deque>

/**
 * @class Plant
 * @brief Discrete-time simulation of a process plant driven by a control signal.
 *
 * Instantiate with a PlantConfig, then call Step() once per simulation tick.
 * Call Reset() between successive simulation runs.
 */
class Plant
{
public:
    // -------------------------------------------------------------------------
    // ----- CONSTRUCTOR / DESTRUCTOR ------------------------------------------
    // -------------------------------------------------------------------------

    /**
     * @brief  Constructs the plant and initialises all internal state.
     * @param  config  Validated PlantConfig describing the process model.
     */
    explicit Plant(const PlantConfig& config);

    /// Default destructor.
    ~Plant() = default;

    // -------------------------------------------------------------------------
    // ----- PUBLIC METHODS ----------------------------------------------------
    // -------------------------------------------------------------------------

    /**
     * @brief  Advances the plant by one time step given a control signal.
     *
     * @param  control_output  PID controller output u(t) applied to the plant.
     * @param  dt              Elapsed time since last call (seconds). Must be > 0.
     *
     * @return Updated process variable y(t+dt).
     *
     * @note   If dt ≤ 0 the function returns the current output unchanged and
     *         emits a warning to stderr.
     */
    double Step(double control_output, double dt);

    /**
     * @brief  Resets the plant state to the configured initial value.
     *
     * Also clears any dead-time delay buffer. Must be called before each new
     * simulation run to guarantee reproducibility.
     */
    void Reset() noexcept;

    // -------------------------------------------------------------------------
    // ----- ACCESSORS ---------------------------------------------------------
    // -------------------------------------------------------------------------

    /// Returns the current process variable output without advancing state.
    double GetOutput() const noexcept { return output_; }

    /// Returns the plant configuration (read-only).
    const PlantConfig& GetConfig() const noexcept { return config_; }

private:
    // -------------------------------------------------------------------------
    // ----- PRIVATE HELPERS ---------------------------------------------------
    // -------------------------------------------------------------------------

    /** @brief Euler step for first-order lag model. */
    double StepFirstOrder(double u, double dt);

    /** @brief Euler step for second-order mass-spring-damper model. */
    double StepSecondOrder(double u, double dt);

    /** @brief Euler step for pure-integrating model. */
    double StepIntegrating(double u, double dt);

    /** @brief Euler step for FOPDT (first-order + dead-time) model. */
    double StepDeadTime(double u, double dt);

    // -------------------------------------------------------------------------
    // ----- PRIVATE DATA MEMBERS ----------------------------------------------
    // -------------------------------------------------------------------------

    PlantConfig config_;    ///< Plant model parameters

    double output_;         ///< Current process variable y(t)
    double state2_;         ///< Second state variable x₂ (second-order model only)

    /// Circular delay buffer for FOPDT dead-time implementation.
    std::deque<double> delay_buffer_;
};

#endif // PIDSIM_PLANT_H