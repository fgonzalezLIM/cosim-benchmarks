# Linear oscillator
Benchmark problem: linear, 2-degree-of-freedom mechanical system

## Executable scripts

|  | |  |
| :---- | :--:| :----------- |
| `main_analytic.m` | : | Evaluates the analytical problem solution. |
| `main_cosim.m` | : | Performs the co-simulation of the assembly. |
| `main_monolithic.m` | : | Performs the monolithic integration of the system motion. |

## Functions

|  | |  |
| :---- | :--:| :----------- |
| `evalMechEnergy.m` | : | Post-process function, computes the mechanical energy of the oscillator after its integration has been completed. |
| `getOscillatorProperties.m` | : | Generates a structure with the mechanical properties and initial conditions of the oscillator. |
| `managerCreate.m` | : | User defined function. Creates the manager structure that orchestrates the co-simulation. |
| `managerEval.m` | : | User defined function. Defines the behaviour of the orchestrator. |
| `mass_ss.m` | : | Class definition: subsystem in oscillator. |
| `plotResults.m` | : | Plots positions, velocities, and energy. |
| `simulate_analytic.m` | : | Performs the analytic solution of the system dynamics. |
| `simulate_monolithic.m` | : | Performs the monolithic integration of the system dynamics. |

