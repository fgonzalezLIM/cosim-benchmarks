% This script simulates the motion of a 2-d.o.f. linear oscillator.
%
% A monolithic simulation approach is used.


%                |-> x1             |-> x2
%   |         |----|             |----|         |
%   |--/\/\/--| m1 |----/\/\/----| m2 |--/\/\/--|
%   | k1, c1  |----|   k_c, c_c  |----|  k2, c2 |

clear variables
close all

% __________________________________________________ Extend path to find library functions
addpath("../common");

% ________________________________________________________________________ Define scenario

scenario    = 0;        % Simulation case to be solved - available: 0, 1.
finalT      = 10.0;     % Final time of motion (s)
H           = 1.0e-3;   % Integration step-size (s)

% Options structure for the integration
OPTIONS.saveEvery   =   10;     % Storage rate
OPTIONS.integrator  =   2;      % 1: Forward-Euler; 2: Trapezoidal rule

% If using TR, add these integration parameters
if OPTIONS.integrator == 2
    OPTIONS.maxIter     = 10;       % Maximum number of iterations
    OPTIONS.maxError    = 1.0e-7;   % Maximum x increase between correction iterations
end

% _______________________________________________________________________________ Solution
SYS = getOscillatorProperties(scenario);
RES = simulate_monolithic(H, finalT, OPTIONS, SYS);

% ___________________________________________________________________ Restore initial path
rmpath("../common");