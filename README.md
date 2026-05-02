# PID Control System Simulation (C++)

A modular, high-fidelity **PID controller simulation framework** written in modern C++.  
This project provides a command-line tool for simulating and analyzing PID-controlled systems using configurable plant models.

---

## Overview

This project simulates how a **PID (Proportional–Integral–Derivative) controller** behaves when controlling a dynamic system ("plant").

It is designed to help users:

- Understand PID behavior intuitively
- Test different tuning parameters (Kp, Ki, Kd)
- Observe system response over time
- Model real-world control systems in a simplified environment

---

## What is a PID Controller?

A PID controller continuously adjusts an output to reduce the error between a desired value (setpoint) and a measured system value.

It uses three terms:

- **Proportional (P):** reacts to current error
- **Integral (I):** reacts to accumulated error over time
- **Derivative (D):** reacts to rate of change of error

This is widely used in:
- Motor control
- Robotics
- Aerospace systems
- Temperature regulation
- Industrial automation

---

## Project Structure
Model/
├── Inc/ # Public headers (interfaces)
├── Src/ # Implementation + CLI application
├── CMakeLists.txt


Core components:

- `PID` → Controller logic
- `Plant` → Simulated physical system
- `Simulator` → Runs time-stepped simulation
- `CLI` → User interface for running simulations

---

## Build Instructions

### Requirements

- C++17 compatible compiler (GCC, Clang, MSVC)
- CMake 3.20+

---

### Build Steps

```bash
# Step 1: Enter project directory
cd Model

# Step 2: Create build folder
mkdir build
cd build

# Step 3: Configure project
cmake ..

# Step 4: Build
cmake --build .