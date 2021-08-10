# Hydraulic manipulator
Benchmark problem: 2-degree-of-freedom planar manipulator actuated by a hydraulic system.

## Executable scripts

|  | |  |
| :---- | :--:| :----------- |
| `main_cosim.m` | : | Performs the co-simulation of the assembly. |
| `main_monolithic.m` | : | Performs the monolithic integration of the system motion. |

## Functions

|  | |  |
| :---- | :--:| :----------- |
| `getManipulatorProperties.m` | : | Generates a structure with the mechanical and hydraulic properties, and the initial conditions of the manipulator. |
| `hydraulic_ss.m` | : | Class definition: hydraulics subsystem. |
| `managerCreate.m` | : | User defined function. Creates the manager structure that orchestrates the co-simulation. |
| `managerEval.m` | : | User defined function. Defines the behaviour of the orchestrator. |
| `manipulator_ss.m` | : | Class definition: subsystem that describes the mechanics of the manipulator. |
| `plotResults.m` | : | Plots actuator length and rate. |
| `simulate_monolithic.m` | : | Performs the monolithic integration of the system dynamics. |

