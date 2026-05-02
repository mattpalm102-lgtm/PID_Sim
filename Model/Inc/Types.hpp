/******************************************************************************
 * @file        Types.hpp
 * @brief       Shared type definitions for the PID simulation framework.
 *
 * @details     Defines all fundamental data structures, enumerations, and
 *              constants used throughout the PID simulation application.
 *              This file serves as the single source of truth for shared
 *              domain types, ensuring consistency across all modules and
 *              eliminating magic numbers from the codebase.
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

#ifndef PIDSIM_TYPES_H
#define PIDSIM_TYPES_H

#include <string>
#include <vector>
#include <limits>

// =============================================================================
// SECTION: Version & Application Constants
// =============================================================================

/// Application name string shown in CLI banners and help text.
constexpr const char* APP_NAME    = "PID Simulation Controller";

/// Semantic version string for this release.
constexpr const char* APP_VERSION = "0.1.0";

/// Author attribution string.
constexpr const char* APP_AUTHOR  = "Matt Palmer";

// =============================================================================
// SECTION: Numeric Limits & Safety Guards
// =============================================================================

/// Minimum allowable simulation time step (seconds).
/// Values smaller than this risk numerical instability in the integrator.
constexpr double DT_MIN = 1e-6;

/// Maximum allowable simulation time step (seconds).
/// Values larger than this degrade simulation fidelity for most plant models.
constexpr double DT_MAX = 10.0;

/// Maximum allowable simulation duration (seconds).
constexpr double SIM_DURATION_MAX = 3600.0;

/// Minimum allowable simulation duration (seconds).
constexpr double SIM_DURATION_MIN = 0.001;

/// Upper bound on absolute PID gain magnitudes (Kp, Ki, Kd).
/// Gains beyond this value are almost always the result of user error.
constexpr double GAIN_MAX = 1e6;

/// Lower bound on absolute PID gain magnitudes (permits zero for P/I/D-only modes).
constexpr double GAIN_MIN = 0.0;

/// Maximum allowable output clamp magnitude.
constexpr double OUTPUT_CLAMP_MAX = 1e9;

/// Maximum allowable setpoint magnitude.
constexpr double SETPOINT_MAX = 1e9;

/// Minimum setpoint magnitude (most negative value allowed).
constexpr double SETPOINT_MIN = -1e9;

// =============================================================================
// SECTION: Plant Model Enumeration
// =============================================================================

/**
 * @enum  PlantModel
 * @brief Identifies the mathematical plant model used during simulation.
 *
 * Each variant corresponds to a different physical system archetype.
 * The plant converts a control signal (output of the PID) into a new
 * process variable measurement at each time step.
 */
enum class PlantModel
{
    FIRST_ORDER,    ///< First-order lag:  τ·dy/dt = -y + K·u
    SECOND_ORDER,   ///< Second-order underdamped/overdamped mass-spring-damper
    INTEGRATING,    ///< Pure integrator:  dy/dt = K·u  (e.g. tank level)
    DEAD_TIME       ///< First-order plus dead-time (FOPDT) with transport delay
};

// =============================================================================
// SECTION: PID Configuration
// =============================================================================

/**
 * @struct PIDConfig
 * @brief  Complete, validated configuration for a single PID controller instance.
 *
 * All fields are validated by the CLI before being passed to the PID object.
 * Default values are chosen to produce a stable, lightly-tuned response on a
 * generic first-order plant.
 */
struct PIDConfig
{
    double kp           = 1.0;      ///< Proportional gain  [dimensionless]
    double ki           = 0.1;      ///< Integral gain      [1/s]
    double kd           = 0.05;     ///< Derivative gain    [s]

    double output_min   = -100.0;   ///< Lower clamp on controller output
    double output_max   =  100.0;   ///< Upper clamp on controller output

    bool   anti_windup  = true;     ///< Enable integrator anti-windup clamping
};

// =============================================================================
// SECTION: Plant Configuration
// =============================================================================

/**
 * @struct PlantConfig
 * @brief  Complete configuration for the simulated plant (process) model.
 *
 * Parameters are interpreted differently depending on the selected PlantModel.
 * See Plant.hpp for per-model semantics.
 */
struct PlantConfig
{
    PlantModel model        = PlantModel::FIRST_ORDER;

    double gain             = 1.0;  ///< DC process gain K
    double time_constant    = 1.0;  ///< Primary time constant τ (seconds)
    double damping_ratio    = 0.7;  ///< Damping ratio ζ (second-order only)
    double natural_freq     = 1.0;  ///< Natural frequency ωₙ rad/s (second-order)
    double dead_time        = 0.0;  ///< Transport delay θ in seconds (FOPDT only)

    double initial_value    = 0.0;  ///< Process variable value at t = 0
};

// =============================================================================
// SECTION: Simulation Configuration
// =============================================================================

/**
 * @struct SimConfig
 * @brief  High-level parameters governing the simulation run.
 */
struct SimConfig
{
    double setpoint         = 10.0;     ///< Desired process variable (reference)
    double duration         = 20.0;     ///< Total simulation time (seconds)
    double dt               = 0.01;     ///< Fixed time step (seconds)

    bool   print_csv        = false;    ///< Emit raw CSV output instead of table
    bool   show_plot        = true;     ///< Render ASCII step-response chart
};

// =============================================================================
// SECTION: Simulation Result Data
// =============================================================================

/**
 * @struct SimulationPoint
 * @brief  Captured state of the simulation at a single time step.
 */
struct SimulationPoint
{
    double time             = 0.0;  ///< Simulation time (seconds)
    double setpoint         = 0.0;  ///< Reference / desired value
    double process_value    = 0.0;  ///< Plant output (measured variable)
    double error            = 0.0;  ///< setpoint − process_value
    double control_output   = 0.0;  ///< PID controller output (plant input)
};

/**
 * @struct SimulationMetrics
 * @brief  Derived performance metrics computed after a simulation run.
 *
 * All time values are measured from t = 0 in seconds.
 * Infinity indicates the criterion was never met within the run duration.
 */
struct SimulationMetrics
{
    double rise_time        = std::numeric_limits<double>::infinity();  ///< 10%→90% of setpoint
    double settling_time    = std::numeric_limits<double>::infinity();  ///< |error| ≤ 2% of setpoint
    double overshoot_pct    = 0.0;                                      ///< Peak overshoot as % of setpoint
    double steady_state_error = 0.0;                                    ///< Final |error| at end of run
    double peak_value       = 0.0;                                      ///< Maximum process_value observed
};

// =============================================================================
// SECTION: Simulation Result Container
// =============================================================================

/**
 * @struct SimulationResult
 * @brief  Aggregated output of a completed simulation run.
 */
struct SimulationResult
{
    std::vector<SimulationPoint> data;  ///< Time-series simulation data
    SimulationMetrics            metrics; ///< Computed performance metrics
    bool                         success = false; ///< True if run completed without error
    std::string                  error_message;   ///< Non-empty on failure
};

#endif // PIDSIM_TYPES_H