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

## Project Structure
```!
Model/
├── Inc/ # Public headers (interfaces)
├── Src/ # Implementation + CLI application
├── Test/ # Google Test unit test suite
├── CMakeLists.txt
```

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
git clone https://github.com/mattpalm102-lgtm/PID_Sim.git
cd PID_Sim
mkdir build && cd build
cmake ..
cmake --build .
```
### Running the Simulator

After building:

```bash
./pidsim
```

## Contributing

PRs are welcome. Keep changes focused and readable.

If you add functionality:

Include tests
Keep interfaces clean
Avoid tightly coupling components

## License

GNU Lesser General Public (see LICENSE)