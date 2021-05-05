% This script evaluates the analytical solution of the motion of a 2-d.o.f. linear
% oscillator.

%                |-> x1             |-> x2
%   |         |----|             |----|         |
%   |--/\/\/--| m1 |----/\/\/----| m2 |--/\/\/--|
%   | k1, c1  |----|   k_c, c_c  |----|  k2, c2 |

clear variables
close all

% ________________________________________________________________________ Define scenario

scenario    = 0;        % Simulation case to be solved - available: 0, 1.
finalT      = 10.0;     % Final time of motion (s)
H           = 0.01;     % Sampling interval for results storage (s)

% _______________________________________________________________________________ Solution
SYS = getOscillatorProperties(scenario);
RES = simulate_analytic(H, finalT, SYS);