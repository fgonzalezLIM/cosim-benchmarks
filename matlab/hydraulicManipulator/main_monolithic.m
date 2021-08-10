% This script performs the numerical integration of a double pendulum manipulator with
% hydraulic actuation. The approach is similar to the one introduced in:
%
% M. A. Naya, J. Cuadrado, D. Dopico, U. Lugrís
% An efficient unified method for the combined simulation of multibody and hydraulic 
% dynamics: comparison with simplified and co-integration approaches
% The Archive of Mechanical Engineering, 58(2):223-243, 2011.
% d.o.i.: 10.2478/v10180-011-0016-4
%
% The system dynamics is solved following a monolithic approach (mechanics and hydraulics 
% are solved together).
%
% The pendulum is composed of a rod with a uniformly distributed mass and a massless rod, 
% plus two point masses, one at the end of each rod.
%
% The mechanical system is modelled with the x and y coordinates of points  1 (centre of 
% first rod), 2 (tip of first rod), and 3 (tip of second rod).
% The actuator length s is also used as coordinate.
% The mass matrix is singular, but this does not seem to bother the algorithm too much.

clear variables
close all

% ____________________________________________________________________________ Extend path
addpath("../common");   % Library functions
addpath("./hyd");       % Hydraulics functions
addpath("./mono");      % Monolithic solution functions

% ________________________________________________________________________ Define scenario
scenario    = 0;        % Manoeuvre to be solved - 0-steps, 1-sinusoidal
finalT      = 10.0;     % Final time of motion (s)
H           = 1.0e-3;   % Integration step-size (s)

% Options structure for the integration
OPTIONS.saveEvery   =   1;     % Storage rate

% The integration formula is intertwined with the dynamics equations and cannot be
% modified. The trapezoidal rule is used.
OPTIONS.maxIter     = 100;      % Maximum number of Newton-Raphson iterations
OPTIONS.maxError    = 1.0e-5;   % Maximum error in Newton-Raphson iteration
OPTIONS.nProjs      = 3;        % Number of velocity and accel. projections
OPTIONS.alpha       = 1.0e12;   % Penalty factor for augmented Lagrangian method

% _______________________________________________________________________________ Solution
SYS = getManipulatorProperties(scenario);
RES = simulate_monolithic(H, finalT, OPTIONS, SYS);

% ___________________________________________________________________ Restore initial path
rmpath("./mono");
rmpath("./hyd");
rmpath("../common");
