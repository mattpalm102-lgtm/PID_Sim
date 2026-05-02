/******************************************************************************
 * @file        pid.hpp
 * @brief       PID controller interface and definitions.
 *
 * @details     Provides a configurable PID controller implementation with
 *              support for proportional, integral, and derivative control.
 *              Designed for deterministic simulation.
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
 
#include <vector>

class PID {

    public:
        // ------------------------------------------------------------------
        // ----- CONSTRUCTOR / DESTRUCTOR -----------------------------------
        // ------------------------------------------------------------------

        PID(double kp_, double ki_, double kd_);
        ~PID() = default;
        
        // ------------------------------------------------------------------
        // ----- PUBLIC METHODS ---------------------------------------------
        // ------------------------------------------------------------------

        double Update(double setpoint, double measurement, double dt, double& control_output);
        void Reset() noexcept;


    private:
        // ------------------------------------------------------------------
        // ----- PRIVATE DATA MEMBERS ---------------------------------------
        // ------------------------------------------------------------------
        
        double kp;
        double ki;
        double kd;
    
        double integral_;
        double prev_error_;

};

#endif //PIDSIM_PID_H