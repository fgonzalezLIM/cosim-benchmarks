% This function provides an analytic solution for the motion of a 2 d.o.f. linear 
% oscillator
%
% Input
%  H:           Sampling interval for results storage (s)
%  finalT:      Final time of motion (s)
%  SYS:         Structure with system properties (see getOscillatorProperties.m)
%
% Output
%  RES:         Structure with results, including:
%   .t          Timestamps (s)
%   .x          Positions (m)
%   .xd         Velocities (m/s)

%                |-> x1             |-> x2
%   |         |----|             |----|         |
%   |--/\/\/--| m1 |----/\/\/----| m2 |--/\/\/--|
%   | k1, c1  |----|   k_c, c_c  |----|  k2, c2 |


function RES = simulate_analytic(H, finalT, SYS)

% ___________________________________________________ System parameters and initial state

% Initial positions and velocities
z0 = [SYS.x0(1); SYS.xd0(1); SYS.x0(2); SYS.xd0(2)];

% Parameters
m1  = SYS.m(1);     m2  = SYS.m(2);
k1  = SYS.k(1);     k2  = SYS.k(2);
c1  = SYS.c(1);     c2  = SYS.c(2);  
k12 = SYS.kc;       c12 = SYS.cc;

% ______________________________________________________________________________ Solution

% Number of timesteps
deltastep   = H;
totaltime   = finalT;
nsteps      = ceil(totaltime/deltastep);

% Vector of positions and velocities for storage
z = zeros(4, nsteps);

% Lead matrix of the system
A = [0.0            1.0             0.0             0.0;
    -(k1 + k12)/m1  -(c1 + c12)/m1  k12/m1         c12/m1;
    0.0             0.0             0.0             1.0;
    k12/m2         c12/m2          -(k2 + k12)/m2  -(c2 + c12)/m2];

% Solution evaluation
t  = zeros(1, nsteps)';
t0 = 0.0;

for i=1:nsteps
    t(i) = t0 + deltastep*(i-1);
    z(:,i) = expm(A*(t(i)-t0)) * z0;
end

% ________________________________________________________________________ Output results
RES.t               = t;
RES.x               = [z(1,:);z(3,:)]';
RES.xd              = [z(2,:);z(4,:)]';

end