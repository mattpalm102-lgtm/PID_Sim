/******************************************************************************
 * @file        Simulator.cpp
 * @brief       PID closed-loop simulation engine implementation.
 *
 * @details     Runs the fixed-step feedback loop:
 *
 *                for t = 0, dt, 2dt, ..., duration:
 *                  u      = PID.Update(setpoint, plant.GetOutput(), dt)
 *                  y      = Plant.Step(u, dt)
 *                  record SimulationPoint{t, setpoint, y, error, u}
 *
 *              After the loop, ComputeMetrics() derives:
 *                - Rise time  (10% → 90% of setpoint)
 *                - Settling time (|error| ≤ 2% of |setpoint|, never re-violated)
 *                - Overshoot percentage
 *                - Steady-state error (final error at end of run)
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

#include "Simulator.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>

// -----------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------
Simulator::Simulator(const PIDConfig&   pid_config,
                     const PlantConfig& plant_config,
                     const SimConfig&   sim_config)
    : pid_config_(pid_config),
      plant_config_(plant_config),
      sim_config_(sim_config)
{
}

// -----------------------------------------------------------------------------
// Run — execute the simulation loop and return results
// -----------------------------------------------------------------------------
SimulationResult Simulator::Run()
{
    SimulationResult result;

    // Validate configuration before committing to the run.
    if (sim_config_.dt <= 0.0)
    {
        result.success       = false;
        result.error_message = "Simulation dt must be > 0.";
        return result;
    }
    if (sim_config_.duration <= 0.0)
    {
        result.success       = false;
        result.error_message = "Simulation duration must be > 0.";
        return result;
    }

    // Instantiate and reset subsystems.
    PID   pid(pid_config_);
    Plant plant(plant_config_);

    pid.Reset();
    plant.Reset();

    // Pre-allocate vector to avoid repeated heap allocation during the loop.
    const auto num_steps = static_cast<std::size_t>(
        sim_config_.duration / sim_config_.dt) + 1;
    result.data.reserve(num_steps);

    // -------------------------------------------------------------------------
    // Main simulation loop
    // -------------------------------------------------------------------------
    double t = 0.0;
    while (t <= sim_config_.duration + sim_config_.dt * 0.5)
    {
        const double measurement = plant.GetOutput();
        const double u           = pid.Update(sim_config_.setpoint, measurement, sim_config_.dt);
        const double y           = plant.Step(u, sim_config_.dt);
        const double error       = sim_config_.setpoint - y;

        SimulationPoint pt;
        pt.time           = t;
        pt.setpoint       = sim_config_.setpoint;
        pt.process_value  = y;
        pt.error          = error;
        pt.control_output = u;

        result.data.push_back(pt);

        t += sim_config_.dt;
    }

    // -------------------------------------------------------------------------
    // Compute post-run metrics
    // -------------------------------------------------------------------------
    result.metrics = ComputeMetrics(result.data);
    result.success = true;

    return result;
}

// -----------------------------------------------------------------------------
// ComputeMetrics — derive performance KPIs from the completed time series
// -----------------------------------------------------------------------------
SimulationMetrics Simulator::ComputeMetrics(
    const std::vector<SimulationPoint>& data) const
{
    SimulationMetrics m;

    if (data.empty())
    {
        return m;
    }

    const double sp = sim_config_.setpoint;

    // Avoid division by zero for a zero setpoint; use absolute error thresholds.
    const double sp_abs = std::abs(sp);
    const double settling_band = (sp_abs > 1e-9) ? 0.02 * sp_abs : 0.02;

    // Rise time: time for process variable to travel from 10% to 90% of setpoint.
    // Measured from initial value (t=0) toward setpoint direction.
    const double initial_pv = data.front().process_value;
    const double rise_10    = initial_pv + 0.10 * (sp - initial_pv);
    const double rise_90    = initial_pv + 0.90 * (sp - initial_pv);

    bool crossed_10 = false;
    double t_10     = 0.0;

    for (const auto& pt : data)
    {
        const double pv = pt.process_value;
        m.peak_value = std::max(m.peak_value, pv);

        // Rise time: 10% crossing
        if (!crossed_10)
        {
            if ((sp >= initial_pv && pv >= rise_10) ||
                (sp <  initial_pv && pv <= rise_10))
            {
                crossed_10 = true;
                t_10 = pt.time;
            }
        }
        // Rise time: 90% crossing (first time)
        if (crossed_10 && m.rise_time == std::numeric_limits<double>::infinity())
        {
            if ((sp >= initial_pv && pv >= rise_90) ||
                (sp <  initial_pv && pv <= rise_90))
            {
                m.rise_time = pt.time - t_10;
            }
        }
    }

    // Settling time: last time the error magnitude exceeded the 2% band.
    // Walk the data in reverse to find the final exit from the band.
    m.settling_time = std::numeric_limits<double>::infinity();
    for (auto it = data.rbegin(); it != data.rend(); ++it)
    {
        if (std::abs(it->error) > settling_band)
        {
            // This is the last point outside the band; settling occurs after.
            m.settling_time = it->time;
            break;
        }
    }
    // If we never left the band, settling time = 0 (always within band).
    if (m.settling_time == std::numeric_limits<double>::infinity())
    {
        m.settling_time = 0.0;
    }

    // Overshoot percentage: (peak_value - setpoint) / setpoint * 100
    // Only meaningful when setpoint > initial_pv (step-up case).
    if (sp_abs > 1e-9 && sp > initial_pv)
    {
        const double overshoot = m.peak_value - sp;
        m.overshoot_pct = (overshoot > 0.0) ? (overshoot / sp_abs) * 100.0 : 0.0;
    }
    else if (sp_abs > 1e-9 && sp < initial_pv)
    {
        // Step-down case: overshoot is when pv drops below setpoint.
        const double min_pv = data.front().process_value;
        double actual_min = min_pv;
        for (const auto& pt : data) actual_min = std::min(actual_min, pt.process_value);
        const double undershoot = sp - actual_min;
        m.overshoot_pct = (undershoot > 0.0) ? (undershoot / sp_abs) * 100.0 : 0.0;
    }

    // Steady-state error: absolute error at the final data point.
    m.steady_state_error = std::abs(data.back().error);

    return m;
}