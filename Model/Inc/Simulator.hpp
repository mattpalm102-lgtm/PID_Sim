/******************************************************************************
 * @file        Simulator.hpp
 * @brief       PID simulation engine.
 *
 * @details     Coordinates interaction between PID controller and plant model
 *              over discrete time steps.
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

#ifndef PIDSIM_SIMULATOR_H
#define PIDSIM_SIMULATOR_H

#include "Types.hpp"
#include "PID.hpp"
#include "Plant.hpp"

/**
 * @class Simulator
 * @brief Fixed-step closed-loop PID simulation engine.
 *
 * ### Example
 * @code
 *   Simulator sim(pid_cfg, plant_cfg, sim_cfg);
 *   SimulationResult result = sim.Run();
 *   if (result.success) { ... }
 * @endcode
 */
class Simulator
{
public:
    // -------------------------------------------------------------------------
    // ----- CONSTRUCTOR / DESTRUCTOR ------------------------------------------
    // -------------------------------------------------------------------------

    /**
     * @brief  Constructs the simulation engine with all required configurations.
     *
     * @param  pid_config    Validated PID gain and clamp parameters.
     * @param  plant_config  Validated plant model parameters.
     * @param  sim_config    Validated simulation run parameters (duration, dt, etc.).
     */
    Simulator(const PIDConfig&   pid_config,
              const PlantConfig& plant_config,
              const SimConfig&   sim_config);

    /// Default destructor.
    ~Simulator() = default;

    // -------------------------------------------------------------------------
    // ----- PUBLIC METHODS ----------------------------------------------------
    // -------------------------------------------------------------------------

    /**
     * @brief  Executes the simulation and returns the full result set.
     *
     * @details Runs the fixed-step PID–Plant feedback loop from t = 0 to
     *          t = sim_config.duration, recording every time step.
     *          On completion, computes SimulationMetrics and packages all
     *          data into the returned SimulationResult.
     *
     * @return SimulationResult containing time-series data and metrics.
     *         result.success == false if an internal error prevented completion.
     */
    SimulationResult Run();

private:
    // -------------------------------------------------------------------------
    // ----- PRIVATE HELPERS ---------------------------------------------------
    // -------------------------------------------------------------------------

    /**
     * @brief  Computes performance metrics from completed time-series data.
     * @param  data     Reference to the completed simulation data vector.
     * @return Populated SimulationMetrics struct.
     */
    SimulationMetrics ComputeMetrics(const std::vector<SimulationPoint>& data) const;

    // -------------------------------------------------------------------------
    // ----- PRIVATE DATA MEMBERS ----------------------------------------------
    // -------------------------------------------------------------------------

    PIDConfig   pid_config_;    ///< PID controller configuration
    PlantConfig plant_config_;  ///< Plant model configuration
    SimConfig   sim_config_;    ///< Simulation run configuration
};

#endif // PIDSIM_SIMULATOR_H