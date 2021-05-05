function [SYS] = getOscillatorProperties(scenario)

% This function provides the parameters of a 2 d.o.f. linear oscillator.
%
% Input
%  scenario:    Simulation case
%
% Output
%  SYS:         Structure with system properties, including;
%   .c          Subsystem damping [c1; c2]
%   .cc         Coupling damping
%   .k          Subsystem stiffness [k1; k2]
%   .kc         Coupling stiffness
%   .m          Subsystem mass [m1; m2]
%   .x0         Initial position of masses w.r.t. equilibrium [x10; x20]
%   .xd0        Initial velocities of masses [x1d0; x2d0]

%                |-> x1             |-> x2
%   |         |----|             |----|         |
%   |--/\/\/--| m1 |----/\/\/----| m2 |--/\/\/--|
%   | k1, c1  |----|   k_c, c_c  |----|  k2, c2 |

% ____________________________________________________________________ Physical properties

% Mass
SYS.m 	= [1.0; 1.0];

% Stiffness
SYS.k 	= [10.0; 1000.0];
SYS.kc  = 100.0;

% Damping
if (scenario == 0)
    SYS.c 	= [0.0; 0.0];
    SYS.cc  = 0.0;
elseif (scenario == 1)
    SYS.c 	= [0.01; 0.01];
    SYS.cc  = 0.01;
else
    error("Undefined scenario");
end

% _____________________________________________________________________ Initial conditions

% Initial positions and velocities [x1, x2], [xd1, xd2]
SYS.x0 	= [0.0; 0.0];
SYS.xd0	= [100; -100];