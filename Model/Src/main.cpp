/******************************************************************************
 * @file        main.cpp
 * @brief       Application entry point for the PID simulation controller.
 *
 * @details     Orchestrates the top-level application flow:
 *
 *                1. Initialise the CLI with standard I/O.
 *                2. Print the application banner.
 *                3. Collect validated configurations from the user (PID,
 *                   Plant, Simulation).
 *                4. Construct and run the Simulator.
 *                5. Display results (table, ASCII chart, optional CSV).
 *                6. Offer the user the opportunity to run again.
 *
 *              All error handling is performed by the CLI (input validation)
 *              and Simulator (runtime guards). main() itself remains clean
 *              and does not contain business logic.
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

#include "CLI.hpp"
#include "Simulator.hpp"
#include "Types.hpp"

#include <iostream>

// -----------------------------------------------------------------------------
// main
// -----------------------------------------------------------------------------
int main()
{
    // Construct the CLI bound to standard terminal I/O.
    CLI cli(std::cin, std::cout);

    cli.PrintBanner();

    bool run_again = true;

    while (run_again)
    {
        // Collect configuration from the user via validated prompts.
        const PIDConfig   pid_cfg   = cli.PromptPIDConfig();
        const PlantConfig plant_cfg = cli.PromptPlantConfig();
        const SimConfig   sim_cfg   = cli.PromptSimConfig();

        std::cout << "\n  Running simulation...\n";

        // Construct and execute the simulation.
        Simulator sim(pid_cfg, plant_cfg, sim_cfg);
        const SimulationResult result = sim.Run();

        // Display numerical results table.
        cli.DisplayResults(result);

        // Render ASCII chart if the user requested it.
        if (sim_cfg.show_plot)
        {
            cli.DisplayAsciiPlot(result);
        }

        // Export CSV if requested.
        if (sim_cfg.print_csv)
        {
            cli.ExportCSV(result);
        }

        // Offer the user another run.
        run_again = cli.AskRunAgain();
    }

    std::cout << "\n  Thank you for using " << APP_NAME << ". Goodbye.\n\n";
    return 0;
}